//==========================================================================
//  INSPECT.CC - part of
//
//                     OMNeT++/OMNEST
//            Discrete System Simulation in C++
//
//  Implementation of inspector() functions
//
//==========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 1992-2008 Andras Varga
  Copyright (C) 2006-2008 OpenSim Ltd.

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/

#include <stdarg.h>
#include "omnetpp.h"
#include "tkenv.h"
#include "tklib.h"
#include "inspector.h"
#include "inspfactory.h"

NAMESPACE_BEGIN

cGlobalRegistrationList inspectorfactories;

EXECUTE_ON_SHUTDOWN( inspectorfactories.clear() );


cInspectorFactory *findInspectorFactoryFor(cObject *obj, int type)
{
    cInspectorFactory *best=NULL;
    double bestweight=0;
    cRegistrationList *a = inspectorfactories.getInstance();
    for (int i=0; i<a->size(); i++)
    {
        cInspectorFactory *ifc = static_cast<cInspectorFactory *>(a->get(i));
        if (ifc->supportsObject(obj) &&
            (type==INSP_DEFAULT || ifc->inspectorType()==type) &&
            ifc->qualityAsDefault(obj)>bestweight
           )
        {
            bestweight = ifc->qualityAsDefault(obj);
            best = ifc;
        }
    }
    return best; // may be NULL too
}

NAMESPACE_END
