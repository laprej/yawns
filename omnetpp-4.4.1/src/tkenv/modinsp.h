//==========================================================================
//  MODINSP.H - part of
//
//                     OMNeT++/OMNEST
//            Discrete System Simulation in C++
//
//==========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 1992-2008 Andras Varga
  Copyright (C) 2006-2008 OpenSim Ltd.

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/

#ifndef __MODINSP_H
#define __MODINSP_H

#include <map>
#include "platmisc.h"   // must precede <tk.h> otherwise Visual Studio 2013 fails to compile
#include <tk.h>
#include "csimplemodule.h"
#include "cchannel.h"
#include "cgate.h"
#include "cmessage.h"
#include "cpar.h"
#include "carray.h"
#include "coutvector.h"
#include "cstatistic.h"
#include "cdensityestbase.h"
#include "cdisplaystring.h"
#include "cqueue.h"
#include "ccompoundmodule.h"
#include "cchannel.h"
#include "cdataratechannel.h"

#include "intxtypes.h"
#include "logbuffer.h"
#include "inspector.h"
#include "envirbase.h"
#include "graphlayouter.h"

NAMESPACE_BEGIN


class TModuleWindow : public TInspector
{
   protected:
      char textWidget[128];
      std::set<int> excludedModuleIds;
   public:
      TModuleWindow(cObject *obj,int typ,const char *geom,void *dat=NULL);
      virtual void createWindow();
      virtual void update();

      virtual void printLastLineOf(const LogBuffer& logBuffer);
      virtual void redisplay(const LogBuffer& logBuffer);

      virtual int inspectorCommand(Tcl_Interp *interp, int argc, const char **argv);

      static void printLastLineOf(Tcl_Interp *interp, const char *textWidget, const LogBuffer& logBuffer, const std::set<int>& excludedModuleIds);
      static void redisplay(Tcl_Interp *interp, const char *textWidget, const LogBuffer& logBuffer, cModule *mod, const std::set<int>& excludedModuleIds);
};


class TGraphicalModWindow : public TInspector
{
   protected:
      char canvas[128];
      bool needs_redraw;
      int32 random_seed;
      bool not_drawn;

      struct Point {double x,y;};
      typedef std::map<cModule*,Point> PositionMap;
      PositionMap submodPosMap;  // recalculateLayout() fills this map

   protected:
      void drawSubmodule(Tcl_Interp *interp, cModule *submod, double x, double y, const char *scaling);
      void drawEnclosingModule(Tcl_Interp *interp, cModule *parentmodule, const char *scaling);
      void drawConnection(Tcl_Interp *interp, cGate *gate);

   public:
      TGraphicalModWindow(cObject *obj,int typ,const char *geom,void *dat=NULL);
      ~TGraphicalModWindow();
      virtual void createWindow();
      virtual void update();
      virtual int inspectorCommand(Tcl_Interp *interp, int argc, const char **argv);

      bool needsRedraw() {return needs_redraw;}

      // implementations of inspector commands:
      virtual int getSubmoduleCount(Tcl_Interp *interp, int argc, const char **argv);
      virtual int getSubmodQ(Tcl_Interp *interp, int argc, const char **argv);
      virtual int getSubmodQLen(Tcl_Interp *interp, int argc, const char **argv);

      // helper for layouting code
      void getSubmoduleCoords(cModule *submod, bool& explicitcoords, bool& obeyslayout,
                                               double& x, double& y, double& sx, double& sy);

      // does full layouting, stores results in submodPosMap
      virtual void recalculateLayout();

      // updates submodPosMap (new modules, changed display strings, etc.)
      virtual void refreshLayout();

      // drawing methods:
      virtual void relayoutAndRedrawAll();
      virtual void redrawAll();

      virtual void redrawModules();
      virtual void redrawMessages();
      virtual void redrawNextEventMarker();
      virtual void updateSubmodules();

      // notifications from envir:
      virtual void submoduleCreated(cModule *newmodule);
      virtual void submoduleDeleted(cModule *module);
      virtual void connectionCreated(cGate *srcgate);
      virtual void connectionDeleted(cGate *srcgate);
      virtual void displayStringChanged();
      virtual void displayStringChanged(cModule *submodule);
      virtual void displayStringChanged(cGate *gate);
      virtual void bubble(cModule *mod, const char *text);
};

//
// *** Note: the following inspectors have been replaced with TGenericObjectInspector ***
//
// class TCompoundModInspector: public TInspector
// {
//    protected:
//       bool deep;
//       bool simpleonly;
//    public:
//       TCompoundModInspector(cObject *obj,int typ,const char *geom,void *dat=NULL);
//       virtual void createWindow();
//       virtual void update();
//       virtual void writeBack();
// };
//
// class TSimpleModInspector: public TInspector
// {
//    public:
//       TSimpleModInspector(cObject *obj,int typ,const char *geom,void *dat=NULL);
//       virtual void createWindow();
//       virtual void update();
//       virtual void writeBack();
// };
//
// class TGateInspector: public TInspector
// {
//    public:
//       TGateInspector(cObject *obj,int typ,const char *geom,void *dat=NULL);
//       virtual void createWindow();
//       virtual void update();
//       virtual void writeBack();
// };
//

class TGraphicalGateWindow : public TInspector
{
   protected:
      char canvas[128];
   public:
      TGraphicalGateWindow(cObject *obj,int typ,const char *geom,void *dat=NULL);
      virtual void createWindow();
      virtual void update();
      virtual int inspectorCommand(Tcl_Interp *interp, int argc, const char **argv);

      virtual int redraw(Tcl_Interp *interp, int argc, const char **argv);

      // notifications from envir:
      virtual void displayStringChanged(cGate *gate);
};

NAMESPACE_END


#endif
