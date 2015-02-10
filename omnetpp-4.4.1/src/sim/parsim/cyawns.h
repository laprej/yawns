//=========================================================================
//  CYAWNS.H - part of
//
//                  OMNeT++/OMNEST
//           Discrete System Simulation in C++
//
//  Author: Christopher D. Carothers
//          Dept. of Computer Science
//          RPI, Troy, NY
//
//=========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 2003-2008 Andras Varga
  Copyright (C) 2006-2008 OpenSim Ltd.

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/

#ifndef __CYAWNS_H__
#define __CYAWNS_H__

#include "cparsimprotocolbase.h"
#include "cmessage.h"  // MK_PARSIM_BEGIN

NAMESPACE_BEGIN

// forward declarations
class cCommBuffer;
class cNMPLookahead;

/**
 * Implements the "YAWNS".
 * Lookahead calculation is encapsulated into a separate object,
 * subclassed from cNMPLookahead.
 *
 * @ingroup Parsim
 */
class SIM_API cYAWNS : public cParsimProtocolBase
{
  protected:
    struct PartitionInfo
    {
        // EIT = Earliest Input Time
        cMessage *eitEvent;  // EIT received from partition
        // EOT = Earliest Output Time
        cMessage *eotEvent;  // events which marks that a null message should be sent out
        simtime_t lastEotSent; // last EOT value that was sent
    };

    // partition information
    int numSeg;              // number of partitions
    PartitionInfo *segInfo;  // partition info array, size numSeg

    bool debug;

    cNMPLookahead *lookaheadcalc;

  protected:
    // process buffers coming from other partitions
    virtual void processReceivedBuffer(cCommBuffer *buffer, int tag, int sourceProcId);

    // reschedule event in FES, to the given time
    virtual void rescheduleEvent(cMessage *msg, simtime_t t);

    void tw_gvt_step1(void);
    void tw_gvt_step2(void);

  public:
    /**
     * Constructor.
     */
    cYAWNS();

    /**
     * Destructor.
     */
    virtual ~cYAWNS();

    /**
     * Redefined beacause we have to pass the same data to the lookahead calculator object
     * (cNMPLookahead) too.
     */
    virtual void setContext(cSimulation *sim, cParsimPartition *seg, cParsimCommunications *co);

    /**
     * Called at the beginning of a simulation run.
     */
    virtual void startRun();

    /**
     * Called at the end of a simulation run.
     */
    virtual void endRun();

    /**
     * Scheduler function. The null message algorithm is embedded here.
     */
    virtual cMessage *getNextEvent();

    /**
     * In addition to its normal task (sending out the cMessage to the
     * given partition), it also does lookahead calculation and optional
     * piggybacking of null message on the cMessage.
     */
    virtual void processOutgoingMessage(cMessage *msg, int procId, int moduleId, int gateId, void *data);
};

NAMESPACE_END


#endif
