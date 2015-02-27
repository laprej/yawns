//=========================================================================
//  CYAWNS.CC - part of
//
//                  OMNeT++/OMNEST
//           Discrete System Simulation in C++
//
//  Author: Justin LaPre and Chris Carothers
//          Dept. of Computer Science,
//          Rensselaer Poly. Inst.
//          Troy, New York, U.S.A
//
//=========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 2003-2008 Andras Varga
  Copyright (C) 2006-2008 OpenSim Ltd.

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/


#include "cmessage.h"
#include "cmodule.h"
#include "cgate.h"
#include "cenvir.h"
#include "cconfiguration.h"
#include "cyawns.h"
#include "clinkdelaylookahead.h"
#include "cparsimpartition.h"
#include "cparsimcomm.h"
#include "ccommbuffer.h"
#include "messagetags.h"
#include "globals.h"
#include "cconfigoption.h"
#include "regmacros.h"
#include "cplaceholdermod.h"
#include "cproxygate.h"
#include "cchannel.h"
#include "mpi.h"
#include <assert.h>

NAMESPACE_BEGIN

Register_Class(cYAWNS);

Register_GlobalConfigOption(CFGID_PARSIM_YAWNS_LOOKAHEAD_CLASS, "parsim-yawns-lookahead-class", CFG_STRING, "cLinkDelayLookahead", "When cYAWNS is selected as parsim synchronization class: specifies the C++ class that calculates lookahead. The class should subclass from cNMPLookahead.");
extern cConfigOption *CFGID_PARSIM_DEBUG; // registered in cparsimpartition.cc

#define YAWNS_BATCH 16

cYAWNS::cYAWNS() : cParsimProtocolBase()
{
    numSeg = 0;
    segInfo = NULL;

    debug = ev.getConfig()->getAsBool(CFGID_PARSIM_DEBUG);
    std::string lookhClass = ev.getConfig()->getAsString(CFGID_PARSIM_YAWNS_LOOKAHEAD_CLASS);
    lookaheadcalc = dynamic_cast<cNMPLookahead *>(createOne(lookhClass.c_str()));
    if (!lookaheadcalc) \
         throw cRuntimeError("Class \"%s\" is not subclassed from cNMPLookahead", lookhClass.c_str());
    GVT = 0;
    tw_net_minimum = SimTime::getMaxTime();
}

cYAWNS::~cYAWNS()
{
    delete lookaheadcalc;

    // segInfo[*].eitEvent/eotEvent messages already deleted with msgQueue.clear()
    delete [] segInfo;
}

void cYAWNS::setContext(cSimulation *sim, cParsimPartition *seg, cParsimCommunications *co)
{
    cParsimProtocolBase::setContext(sim, seg, co);
    lookaheadcalc->setContext(sim, seg, co);
}

void cYAWNS::startRun()
{
    ev << "starting YAWNS Protocol...\n";

    delete [] segInfo;

    numSeg = comm->getNumPartitions();
    segInfo = new PartitionInfo[numSeg];
    int myProcId = comm->getProcId();

    char buf[30];
    int i;

    // temporarily initialize everything to zero.
    for (i=0; i<numSeg; i++)
    {
        segInfo[i].eotEvent = NULL;
        segInfo[i].eitEvent = NULL;
        segInfo[i].lastEotSent = 0.0;
    }

    // start lookahead calculator too
    lookaheadcalc->startRun();

    LA = SimTime::getMaxTime();
    // Store the lowest lookahead value
    for (int i = 0; i < numSeg; i++) {
        SimTime x = lookaheadcalc->getCurrentLookahead(i);
        if (x > 0) {
            if (LA > x) {
                LA = x;
                printf("Setting lookahead to %s\n", LA.str().c_str());
            }
        }
    }

    comm->setNumSent(0);
    comm->setNumRecv(0);

    const char *s = ev.getConfig()->getConfigValue("sim-time-limit");
    if (s) {
        printf("sim-time-limit is %s\n", s);
        endOfTime = endOfTime.parse(s);
        printf("%s\n", endOfTime.str().c_str());
    }

    ev << "  setup done.\n";
}

void cYAWNS::endRun()
{
    lookaheadcalc->endRun();
}

void cYAWNS::processOutgoingMessage(cMessage *msg, int destProcId, int destModuleId, int destGateId, void *data)
{
    if (tw_net_minimum > msg->getArrivalTime()) {
        tw_net_minimum = msg->getArrivalTime();
    }
    simtime_t LA = lookaheadcalc->getCurrentLookahead(destProcId);
    assert(msg->getArrivalTime() > SimTime() + LA);

  cCommBuffer *buffer = comm->createCommBuffer();

  // send cMessage
  buffer->pack(destModuleId);
  buffer->pack(destGateId);
  buffer->packObject(msg);
  comm->send(buffer, TAG_CMESSAGE, destProcId);
  comm->recycleCommBuffer(buffer);
  comm->setNumSent(comm->getNumSent() + 1);
}

void cYAWNS::processReceivedBuffer(cCommBuffer *buffer, int tag, int sourceProcId)
{
    int destModuleId;
    int destGateId;
    cMessage *msg;
    simtime_t eit;

    switch (tag)
    {
        case TAG_CMESSAGE_WITH_NULLMESSAGE:
            printf("Error: YAWNS protocol recv'ed a CMESSAGE with NULLLMESSAGE!\n");
            exit(-1);
            break;

        case TAG_CMESSAGE:
            buffer->unpack(destModuleId);
            buffer->unpack(destGateId);
            msg = (cMessage *)buffer->unpackObject();
            processReceivedMessage(msg, destModuleId, destGateId, sourceProcId);
            comm->setNumRecv(comm->getNumRecv() + 1);
            break;

        case TAG_NULLMESSAGE:
            printf("Error: YAWNS protocol recv'ed a NULLLMESSAGE!\n");
            exit(-1);
            break;

        default:
            partition->processReceivedBuffer(buffer, tag, sourceProcId);
            break;
    }
    buffer->assertBufferEmpty();
}

enum gvt_status { TW_GVT_NORMAL, TW_GVT_COMPUTE };

static gvt_status local_gvt_status = TW_GVT_NORMAL;

static unsigned long gvt_cnt = 0;
static unsigned long all_reduce_cnt = 0;
static unsigned g_tw_gvt_interval = 16;

void
cYAWNS::tw_gvt_step1(void)
{
    if(local_gvt_status == TW_GVT_COMPUTE ||
       ++gvt_cnt < g_tw_gvt_interval)
        return;

    local_gvt_status = TW_GVT_COMPUTE;
}

void
cYAWNS::tw_gvt_step2(void)
{
    int64_t local_white = 0;
    int64_t total_white = 0;

    SimTime pq_min = SimTime::getMaxTime();
    SimTime net_min = SimTime::getMaxTime();

    SimTime lvt = SimTime::getMaxTime();
    SimTime gvt;

    if(local_gvt_status != TW_GVT_COMPUTE)
        return;

    while(1)
    {
        printf("GVT Step 2: %d: Before NB Recv \n", comm->getProcId());
        receiveNonblocking();
        printf("GVT Step 2: %d: After NB Recv and Before MPI_Allreduce \n", comm->getProcId());
        // send message counts to create consistent cut
        local_white = comm->getNumSent() - comm->getNumRecv();
        printf("GVT Step 2: %d: Computed Sent (%lld) - Recv(%lld) as %lld \n", comm->getProcId(),
               comm->getNumSent(), comm->getNumRecv(), local_white );
        all_reduce_cnt++;
        if(MPI_Allreduce(
                         &local_white,
                         &total_white,
                         1,
                         MPI_LONG_LONG,
                         MPI_SUM,
                         MPI_COMM_WORLD) != MPI_SUCCESS) {
            printf("MPI_Allreduce for GVT failed");
            exit(-1);
        }
        printf("GVT Step 2: %d: After MPI_Allreduce, total white %lld \n", comm->getProcId(), total_white );
        if(total_white == 0)
            break;
    }

    printf("GVT Step 2: %d: Complete MPI_Allreduce \n", comm->getProcId() );

    if (cMessage *cmsg = sim->msgQueue.peekFirst()) {
        pq_min = cmsg->getArrivalTime();
    }

    net_min = tw_net_minimum;

    if(lvt > pq_min)
        lvt = pq_min;
    if(lvt > net_min)
        lvt = net_min;

    all_reduce_cnt++;

    int64_t lvt_raw = lvt.raw();
    int64_t gvt_raw = gvt.raw();

    if(MPI_Allreduce(
                     &lvt_raw,
                     &gvt_raw,
                     1,
                     MPI_LONG_LONG, // SimTime is internally an int64
                     MPI_MIN,
                     MPI_COMM_WORLD) != MPI_SUCCESS) {
        printf("MPI_Allreduce for GVT failed");
        exit(-1);
    }
    printf("GVT Step 2: %d: Completed GVT MPI_Allreduce \n", comm->getProcId() );
    gvt.setRaw(gvt_raw);

    comm->setNumSent(0);
    comm->setNumRecv(0);
    local_gvt_status = TW_GVT_NORMAL;

    gvt_cnt = 0;

    // Set the GVT for this instance
    if (gvt >= GVT)
    {
        GVT = gvt;
    }
    else
    {
        printf("GVT went backwards - need help \n");
        exit(-1);
    }

    printf("GVT Step 2: %d: New GVT is %lf \n", comm->getProcId(), gvt.dbl());

    tw_net_minimum = SimTime::getMaxTime();

    //    g_tw_gvt_done++;
}

cMessage *cYAWNS::getNextEvent()
{
    static unsigned batch = 0;

    cMessage *msg;
    while (true)
    {
        batch++;
        if (batch == YAWNS_BATCH) {
            batch = 0;
            tw_gvt_step1();
            tw_gvt_step2();
        }
        if (GVT > endOfTime) {
            return NULL;
        }

        msg = sim->msgQueue.peekFirst();
        if (!msg) continue;
        if (msg->getArrivalTime() > GVT + LA) {
            // We can't move forward so compute GVT
            batch = YAWNS_BATCH - 1;
            local_gvt_status = TW_GVT_COMPUTE;
            continue;
        }
        return msg;
    }

    return msg;
}

void cYAWNS::rescheduleEvent(cMessage *msg, simtime_t t)
{
    sim->msgQueue.remove(msg);  // also works if the event is not currently scheduled
    msg->setArrivalTime(t);
    sim->msgQueue.insert(msg);
}

NAMESPACE_END

