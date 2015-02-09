//=========================================================================
//  CNULLMESSAGEPROT.CC - part of
//
//                  OMNeT++/OMNEST
//           Discrete System Simulation in C++
//
//  Author: Andras Varga, 2003
//          Dept. of Electrical and Computer Systems Engineering,
//          Monash University, Melbourne, Australia
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

NAMESPACE_BEGIN

Register_Class(cYAWNS);

Register_GlobalConfigOption(CFGID_PARSIM_YAWNS_LOOKAHEAD_CLASS, "parsim-yawns-lookahead-class", CFG_STRING, "cLinkDelayLookahead", "When cYAWNS is selected as parsim synchronization class: specifies the C++ class that calculates lookahead. The class should subclass from cNMPLookahead.");
extern cConfigOption *CFGID_PARSIM_DEBUG; // registered in cparsimpartition.cc

cYAWNS::cYAWNS() : cParsimProtocolBase()
{
    numSeg = 0;
    segInfo = NULL;

    debug = ev.getConfig()->getAsBool(CFGID_PARSIM_DEBUG);
    std::string lookhClass = ev.getConfig()->getAsString(CFGID_PARSIM_YAWNS_LOOKAHEAD_CLASS);
    lookaheadcalc = dynamic_cast<cNMPLookahead *>(createOne(lookhClass.c_str()));
    if (!lookaheadcalc) \
         throw cRuntimeError("Class \"%s\" is not subclassed from cNMPLookahead", lookhClass.c_str());
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
    ev << "starting Null Message Protocol...\n";

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

    // Note boot sequence: first we have to schedule all "resend-EOT" events,
    // so that the simulation will start by sending out null messages --
    // otherwise we'd end up sitting blocked on an EIT event forever!

    // create "resend-EOT" events and schedule them to zero (1st thing to do)
    ev << "  scheduling 'resend-EOT' events...\n";
    for (i=0; i<numSeg; i++)
    {
        if (i!=myProcId)
        {
            sprintf(buf,"resendEOT-%d", i);
            cMessage *eotMsg =  new cMessage(buf);
            eotMsg->setContextPointer((void *)(long)i);  // khmm...
            segInfo[i].eotEvent = eotMsg;
            rescheduleEvent(eotMsg, 0.0);
        }
    }

    // create EIT events and schedule them to zero (null msgs will bump them)
    ev << "  scheduling 'EIT' events...\n";
    for (i=0; i<numSeg; i++)
    {
        if (i!=myProcId)
        {
            sprintf(buf,"EIT-%d", i);
            cMessage *eitMsg =  new cMessage(buf);
            segInfo[i].eitEvent = eitMsg;
            rescheduleEvent(eitMsg, 0.0);
        }
    }

    // start lookahead calculator too
    lookaheadcalc->startRun();

    ev << "  setup done.\n";
}

void cYAWNS::endRun()
{
    lookaheadcalc->endRun();
}

void cYAWNS::processOutgoingMessage(cMessage *msg, int destProcId, int destModuleId, int destGateId, void *data)
{
  cCommBuffer *buffer = comm->createCommBuffer();

  // send cMessage
  buffer->pack(destModuleId);
  buffer->pack(destGateId);
  buffer->packObject(msg);
  comm->send(buffer, TAG_CMESSAGE, destProcId);
  comm->recycleCommBuffer(buffer);
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
            // buffer->unpack(eit);
            // processReceivedEIT(sourceProcId, eit);
            // buffer->unpack(destModuleId);
            // buffer->unpack(destGateId);
            // msg = (cMessage *)buffer->unpackObject();
            // processReceivedMessage(msg, destModuleId, destGateId, sourceProcId);
	  printf("Error: YAWNS protocol recv'ed a CMESSAGE with NULLLMESSAGE!\n");
	  exit(-1);
            break;

        case TAG_CMESSAGE:
            buffer->unpack(destModuleId);
            buffer->unpack(destGateId);
            msg = (cMessage *)buffer->unpackObject();
            processReceivedMessage(msg, destModuleId, destGateId, sourceProcId);
            break;

        case TAG_NULLMESSAGE:
	  // buffer->unpack(eit);
	  //processReceivedEIT(sourceProcId, eit);
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

static unsigned gvt_cnt = 0;
static unsigned g_tw_gvt_interval = 16;

void
tw_gvt_step1(void)
{
    if(local_gvt_status == TW_GVT_COMPUTE ||
       ++gvt_cnt < g_tw_gvt_interval)
        return;

    local_gvt_status = TW_GVT_COMPUTE;
}

void
tw_gvt_step2(void)
{
    long long local_white = 0;
    long long total_white = 0;

    SimTime pq_min = SimTime::getMaxTime();
    SimTime net_min = SimTime::getMaxTime();

    SimTime lvt;
    SimTime gvt;

    simtime_t_cref start = simTime();

    if(local_gvt_status != TW_GVT_COMPUTE)
        return;

    while(1)
    {
        tw_net_read(me);

        // send message counts to create consistent cut
        local_white = me->s_nwhite_sent - me->s_nwhite_recv;
        all_reduce_cnt++;
        if(MPI_Allreduce(
                         &local_white,
                         &total_white,
                         1,
                         MPI_LONG_LONG,
                         MPI_SUM,
                         MPI_COMM_WORLD) != MPI_SUCCESS)
            tw_error(TW_LOC, "MPI_Allreduce for GVT failed");

        if(total_white == 0)
            break;
    }

    pq_min = tw_pq_minimum(me->pq);
    net_min = tw_net_minimum(me);

    lvt = me->trans_msg_ts;
    if(lvt > pq_min)
        lvt = pq_min;
    if(lvt > net_min)
        lvt = net_min;

    all_reduce_cnt++;
    if(MPI_Allreduce(
                     &lvt,
                     &gvt,
                     1,
                     MPI_DOUBLE,
                     MPI_MIN,
                     MPI_COMM_WORLD) != MPI_SUCCESS)
        tw_error(TW_LOC, "MPI_Allreduce for GVT failed");

    gvt = ROSS_MIN(gvt, me->GVT_prev);

    if(gvt != me->GVT_prev)
    {
        g_tw_gvt_no_change = 0;
    } else
    {
        g_tw_gvt_no_change++;
        if (g_tw_gvt_no_change >= g_tw_gvt_max_no_change) {
            tw_error(
                     TW_LOC,
                     "GVT computed %d times in a row"
                     " without changing: GVT = %14.14lf, PREV %14.14lf"
                     " -- GLOBAL SYNCH -- out of memory!",
                     g_tw_gvt_no_change, gvt, me->GVT_prev);
        }
    }

    if (me->GVT > gvt)
    {
        tw_error(TW_LOC, "PE %u GVT decreased %g -> %g",
                 me->id, me->GVT, gvt);
    }

    if (gvt / g_tw_ts_end > percent_complete && (g_tw_mynode == g_tw_masternode)) {
        gvt_print(gvt);
    }

    me->s_nwhite_sent = 0;
    me->s_nwhite_recv = 0;
    me->trans_msg_ts = DBL_MAX;
    me->GVT_prev = DBL_MAX; // me->GVT;
    me->GVT = gvt;
    me->gvt_status = TW_GVT_NORMAL;

    gvt_cnt = 0;

    // update GVT timing stats
    me->stats.s_gvt += tw_clock_read() - start;

    // only FC if OPTIMISTIC
    if( g_tw_synchronization_protocol == OPTIMISTIC )
    {
        start = tw_clock_read();
        tw_pe_fossil_collect(me);
        me->stats.s_fossil_collect += tw_clock_read() - start;
    }

    g_tw_gvt_done++;
}

cMessage *cYAWNS::getNextEvent()
{
    // our EIT and resendEOT messages are always scheduled, so the FES can
    // only be empty if there are no other partitions at all -- "no events" then
    // means we're finished.
    if (sim->msgQueue.isEmpty())
        return NULL;

    // we could do a receiveNonblocking() call here to look at our mailbox,
    // but for performance reasons we don't -- it's enough to read it
    // (receiveBlocking()) when we're stuck on an EIT. Or should we do it
    // every 1000 events or so? If MPI receive buffer fills up it can cause
    // deadlock.
    //receiveNonblocking();

    cMessage *msg;
    while (true)
    {
      msg = sim->msgQueue.peekFirst();
      // Do something here
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

