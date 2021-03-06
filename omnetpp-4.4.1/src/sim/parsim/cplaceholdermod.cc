//==========================================================================
//   CPLACEHOLDERMOD.CC  -  header for
//
//                    OMNeT++/OMNEST
//           Discrete System Simulation in C++
//
//  Author: Andras Varga, 2003
//          Dept. of Electrical and Computer Systems Engineering,
//          Monash University, Melbourne, Australia
//
//==========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 1992-2008 Andras Varga
  Copyright (C) 2006-2008 OpenSim Ltd.

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/

#include "cplaceholdermod.h"
#include "cproxygate.h"

#include "cfilecomm.h"
#include "cnamedpipecomm.h"
#include "cmpicomm.h"
#include "cnosynchronization.h"
#include "cnullmessageprot.h"
#include "cyawns.h"
#include "cispeventlogger.h"
#include "cidealsimulationprot.h"
#include "clinkdelaylookahead.h"
#include <stdio.h>

NAMESPACE_BEGIN

cPlaceholderModule::cPlaceholderModule()
{
}

cPlaceholderModule::~cPlaceholderModule()
{
}

std::string cPlaceholderModule::info() const
{
    std::stringstream out;
    out << "id=" << getId() << ", PLACEHOLDER";
    return out.str();
}

void cPlaceholderModule::arrived(cMessage *msg, cGate *ongate, simtime_t t)
{
    throw cRuntimeError(this, "internal error: arrived() called");
}

void cPlaceholderModule::scheduleStart(simtime_t t)
{
    // do nothing
}

cGate *cPlaceholderModule::createGateObject(cGate::Type type)
{
    if (type==cGate::INPUT)
        return new cProxyGate();
    else
        return cModule::createGateObject(type);
}

//----------------------------------------
// as usual: tribute to smart linkers
void parsim_dummy()
{
    cFileCommunications fc;
    cNamedPipeCommunications npc;
#ifdef WITH_MPI
    cMPICommunications mc;
#endif
    cNoSynchronization ns;
    cNullMessageProtocol np;
    cYAWNS yawns;
    cISPEventLogger iel;
    cIdealSimulationProtocol ip;
    cLinkDelayLookahead ldla;
    // prevent "unused variable" warnings:
    (void)fc; (void)npc; (void)ns; (void)yawns; (void)np; (void)iel; (void)ip; (void)ldla;
}

NAMESPACE_END

