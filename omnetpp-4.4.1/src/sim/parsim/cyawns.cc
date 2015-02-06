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
    laziness = ev.getConfig()->getAsDouble(CFGID_PARSIM_YAWNS_LAZINESS);
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
            cMessage *eotMsg =  new cMessage(buf,MK_PARSIM_RESENDEOT);
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
            cMessage *eitMsg =  new cMessage(buf,MK_PARSIM_EIT);
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

// void cYAWNS::processReceivedEIT(int sourceProcId, simtime_t eit)
// {
//     cMessage *eitMsg = segInfo[sourceProcId].eitEvent;

//     {if (debug) ev.printf("null msg received from %d, EIT=%s, rescheduling EIT event\n", sourceProcId, SIMTIME_STR(eit));}

//     // sanity check
//     ASSERT(eit > eitMsg->getArrivalTime());

//     // reschedule it to the EIT just received
//     rescheduleEvent(eitMsg, eit);
// }

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
        if (msg->getKind() == MK_PARSIM_RESENDEOT)
        {
            // send null messages if window closed for a partition
            int procId = (long) msg->getContextPointer();  // khmm...
            sendNullMessage(procId, msg->getArrivalTime());
        }
        else if (msg->getKind() == MK_PARSIM_EIT)
        {
            // wait until it gets out of the way (i.e. we get a higher EIT)
            {if (debug) ev.printf("blocking on EIT event `%s'\n", msg->getName());}
            if (!receiveBlocking())
                return NULL;
        }
        else
        {
            // just a normal event -- go ahead with it
            break;
        }
    }
    return msg;
}

// void cYAWNS::sendNullMessage(int procId, simtime_t now)
// {
//     // calculate EOT and sending of next null message
//     simtime_t lookahead = lookaheadcalc->getCurrentLookahead(procId);
//     simtime_t eot = now + lookahead;

//     // ensure that even with eager resend, we only send out EOTs that
//     // differ from previous one!
//     if (eot == segInfo[procId].lastEotSent)
//         return;
//     if (eot < segInfo[procId].lastEotSent)
//         throw cRuntimeError("cYAWNS error: attempt to decrease EOT");
//     segInfo[procId].lastEotSent = eot;

//     // calculate time of next null message sending, and schedule "resend-EOT" event
//     simtime_t eotResendTime = now + lookahead*laziness;
//     rescheduleEvent(segInfo[procId].eotEvent, eotResendTime);

//     {if (debug) ev.printf("sending null msg to %d, lookahead=%s, EOT=%s; next resend at %s\n",procId,SIMTIME_STR(lookahead),SIMTIME_STR(eot),SIMTIME_STR(eotResendTime));}

//     // send out null message
//     cCommBuffer *buffer = comm->createCommBuffer();
//     buffer->pack(eot);
//     comm->send(buffer, TAG_NULLMESSAGE, procId);
//     comm->recycleCommBuffer(buffer);
// }

void cYAWNS::rescheduleEvent(cMessage *msg, simtime_t t)
{
    sim->msgQueue.remove(msg);  // also works if the event is not currently scheduled
    msg->setArrivalTime(t);
    sim->msgQueue.insert(msg);
}

NAMESPACE_END

