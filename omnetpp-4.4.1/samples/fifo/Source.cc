//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


#include <omnetpp.h>

namespace fifo {

/**
 * Generates messages or jobs; see NED file for more info.
 */
class Source : public cSimpleModule
{
  private:
    cMessage *sendMessageEvent;

  public:
     Source();
     virtual ~Source();

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};

Define_Module(Source);

Source::Source()
{
    sendMessageEvent = NULL;
}

Source::~Source()
{
    cancelAndDelete(sendMessageEvent);
}

void Source::initialize()
{
    sendMessageEvent = new cMessage("sendMessageEvent");
    scheduleAt(simTime(), sendMessageEvent);
}

void Source::handleMessage(cMessage *msg)
{
    ASSERT(msg==sendMessageEvent);

    cMessage *job = new cMessage("job");
    send(job, "out");

    scheduleAt(simTime()+par("sendIaTime").doubleValue(), sendMessageEvent);
}

}; //namespace

