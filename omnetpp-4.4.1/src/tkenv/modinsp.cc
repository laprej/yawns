//==========================================================================
//  MODINSP.CC - part of
//
//                     OMNeT++/OMNEST
//            Discrete System Simulation in C++
//
//  Implementation of
//    inspectors
//
//==========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 1992-2008 Andras Varga
  Copyright (C) 2006-2008 OpenSim Ltd.

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "modinsp.h"
#include "cchannel.h"
#include "tkenv.h"
#include "tklib.h"
#include "tkutil.h"
#include "inspfactory.h"
#include "arrow.h"
#include "graphlayouter.h"
#include "layouterenv.h"
#include "forcedirectedgraphlayouter.h"
#include "basicspringembedderlayout.h"
#include "stringtokenizer.h"

NAMESPACE_BEGIN

#define UNKNOWNICON_WIDTH  32
#define UNKNOWNICON_HEIGHT 32

void _dummy_for_modinsp() {}


class TModuleWindowFactory : public cInspectorFactory
{
  public:
    TModuleWindowFactory(const char *name) : cInspectorFactory(name) {}

    bool supportsObject(cObject *obj) {return dynamic_cast<cModule *>(obj)!=NULL;}
    int inspectorType() {return INSP_MODULEOUTPUT;}
    double qualityAsDefault(cObject *object) {return 0.5;}

    TInspector *createInspectorFor(cObject *object,int type,const char *geom,void *data) {
        return new TModuleWindow(object, type, geom, data);
    }
};

Register_InspectorFactory(TModuleWindowFactory);


TModuleWindow::TModuleWindow(cObject *obj,int typ,const char *geom,void *dat) :
    TInspector(obj,typ,geom,dat)
{
}

void TModuleWindow::createWindow()
{
    TInspector::createWindow(); // create window name etc.
    strcpy(textWidget,windowname); strcat(textWidget, ".main.text");

    // create inspector window by calling the specified proc with
    // the object's pointer. Window name will be like ".ptr80003a9d-1"
    Tcl_Interp *interp = getTkenv()->getInterp();
    cModule *mod = static_cast<cModule *>(object);
    const char *createcommand = mod->isSimple() ?
             "createSimpleModuleWindow " : "createCompoundModuleWindow ";
    CHK(Tcl_VarEval(interp, createcommand, windowname, " \"", geometry, "\"", NULL ));
    redisplay(getTkenv()->getLogBuffer());
}

void TModuleWindow::update()
{
    TInspector::update();

    Tcl_Interp *interp = getTkenv()->getInterp();
    CHK(Tcl_VarEval(interp, "moduleWindow:trimlines ", windowname, NULL));
}

void TModuleWindow::printLastLineOf(const LogBuffer& logBuffer)
{
    printLastLineOf(getTkenv()->getInterp(), textWidget, logBuffer, excludedModuleIds);
}

void TModuleWindow::redisplay(const LogBuffer& logBuffer)
{
    redisplay(getTkenv()->getInterp(), textWidget, logBuffer, static_cast<cModule *>(object), excludedModuleIds);
}

void TModuleWindow::printLastLineOf(Tcl_Interp *interp, const char *textWidget, const LogBuffer& logBuffer, const std::set<int>& excludedModuleIds)
{
    const LogBuffer::Entry& entry = logBuffer.getEntries().back();
    if (!entry.moduleIds)
    {
        if (entry.lines.empty())
            textWidget_insert(interp, textWidget, entry.banner, "log");
        else
            textWidget_insert(interp, textWidget, entry.lines.back());
    }
    else if (excludedModuleIds.find(entry.moduleIds[0])==excludedModuleIds.end())
    {
        if (entry.lines.empty())
            textWidget_insert(interp, textWidget, entry.banner, "event");
        else
            textWidget_insert(interp, textWidget, entry.lines.back());
    }
    textWidget_gotoEnd(interp, textWidget);
}

void TModuleWindow::redisplay(Tcl_Interp *interp, const char *textWidget, const LogBuffer& logBuffer, cModule *mod, const std::set<int>& excludedModuleIds)
{
    textWidget_clear(interp, textWidget);

    if (!mod)
        return;

    int inspModuleId = mod->getId();
    const std::list<LogBuffer::Entry>& entries = logBuffer.getEntries();
    for (std::list<LogBuffer::Entry>::const_iterator it=entries.begin(); it!=entries.end(); it++)
    {
        const LogBuffer::Entry& entry = *it;
        if (!entry.moduleIds)
        {
            textWidget_insert(interp, textWidget, entry.banner, "log");
            for (int i=0; i<(int)entry.lines.size(); i++)
                textWidget_insert(interp, textWidget, entry.lines[i]); //?
        }
        else
        {
            // check that this module is covered in entry.moduleIds[] (module path up to the root)
            bool found = false;
            for (int *p = entry.moduleIds; !found && *p; p++)
                if (*p == inspModuleId)
                    found = true;

            // if so, and is not excluded, display log
            if (found && excludedModuleIds.find(entry.moduleIds[0])==excludedModuleIds.end())
            {
                textWidget_insert(interp, textWidget, entry.banner, "event");
                for (int i=0; i<(int)entry.lines.size(); i++)
                    textWidget_insert(interp, textWidget, entry.lines[i]);
            }
        }
    }
    textWidget_gotoEnd(interp, textWidget);
}

int TModuleWindow::inspectorCommand(Tcl_Interp *interp, int argc, const char **argv)
{
    if (argc<1) {Tcl_SetResult(interp, TCLCONST("wrong number of args"), TCL_STATIC); return TCL_ERROR;}

    // supported commands: redisplay, getexcludedmoduleids, setexcludedmoduleids

    if (strcmp(argv[0],"redisplay")==0)
    {
       if (argc!=1) {Tcl_SetResult(interp, TCLCONST("wrong argcount"), TCL_STATIC); return TCL_ERROR;}
       TRY(redisplay(getTkenv()->getLogBuffer()));
       return TCL_OK;
    }
    else if (strcmp(argv[0],"getexcludedmoduleids")==0)
    {
       if (argc!=1) {Tcl_SetResult(interp, TCLCONST("wrong argcount"), TCL_STATIC); return TCL_ERROR;}
       Tcl_Obj *listobj = Tcl_NewListObj(0, NULL);
       for (std::set<int>::iterator it=excludedModuleIds.begin(); it!=excludedModuleIds.end(); it++)
           Tcl_ListObjAppendElement(interp, listobj, Tcl_NewIntObj(*it));
       Tcl_SetObjResult(interp, listobj);
       return TCL_OK;
    }
    else if (strcmp(argv[0],"setexcludedmoduleids")==0)
    {
       if (argc!=2) {Tcl_SetResult(interp, TCLCONST("wrong argcount"), TCL_STATIC); return TCL_ERROR;}
       excludedModuleIds.clear();
       StringTokenizer tokenizer(argv[1]);
       while (tokenizer.hasMoreTokens())
           excludedModuleIds.insert(atoi(tokenizer.nextToken()));
       return TCL_OK;
    }
    return TCL_ERROR;
}

//=======================================================================

class TGraphicalModWindowFactory : public cInspectorFactory
{
  public:
    TGraphicalModWindowFactory(const char *name) : cInspectorFactory(name) {}

    bool supportsObject(cObject *obj) {return dynamic_cast<cModule *>(obj)!=NULL;}
    int inspectorType() {return INSP_GRAPHICAL;}
    double qualityAsDefault(cObject *object) {
        return dynamic_cast<cSimpleModule *>(object) ? 0.9 : 3.0;
    }

    TInspector *createInspectorFor(cObject *object,int type,const char *geom,void *data) {
        return new TGraphicalModWindow(object, type, geom, data);
    }
};

Register_InspectorFactory(TGraphicalModWindowFactory);


TGraphicalModWindow::TGraphicalModWindow(cObject *obj,int typ,const char *geom,void *dat) :
    TInspector(obj,typ,geom,dat)
{
   needs_redraw = false;
   not_drawn = false;

   const cDisplayString blank;
   cModule *parentmodule = static_cast<cModule *>(object);
   const cDisplayString& ds = parentmodule->hasDisplayString() && parentmodule->parametersFinalized() ? parentmodule->getDisplayString() : blank;
   random_seed = resolveLongDispStrArg(ds.getTagArg("bgl",4), parentmodule, 1);
}

TGraphicalModWindow::~TGraphicalModWindow()
{
}

void TGraphicalModWindow::createWindow()
{
   TInspector::createWindow(); // create window name etc.
   strcpy(canvas,windowname); strcat(canvas,".c");

   // create inspector window by calling the specified proc with
   // the object's pointer. Window name will be like ".ptr80003a9d-1"
   Tcl_Interp *interp = getTkenv()->getInterp();
   CHK(Tcl_VarEval(interp, "createGraphicalModWindow ", windowname, " \"", geometry, "\"", NULL ));
}

void TGraphicalModWindow::update()
{
   TInspector::update();

   if (not_drawn) return;

   // redraw modules only if really needed
   if (needs_redraw)
   {
       needs_redraw = false;
       redrawAll();
   }
   else
   {
       redrawNextEventMarker();
       redrawMessages();
       updateSubmodules();
   }
}

void TGraphicalModWindow::relayoutAndRedrawAll()
{
   cModule *mod = (cModule *)object;
   int submodcount = 0;
   int gatecountestimate = mod->gateCount();
   for (cModule::SubmoduleIterator submod(mod); !submod.end(); submod++)
   {
       submodcount++;
       // note: gatecountestimate will count unconnected gates in the gate array as well
       gatecountestimate += submod()->gateCount();
   }

   not_drawn = false;
   if (submodcount>1000 || gatecountestimate>4000)
   {
       Tcl_Interp *interp = getTkenv()->getInterp();
       char problem[200];
       if (submodcount>1000)
           sprintf(problem, "contains more than 1000 submodules (exactly %d)", submodcount);
       else
           sprintf(problem, "may contain a lot of connections (modules have a large number of gates)");
       CHK(Tcl_VarEval(interp,"tk_messageBox -parent ",windowname," -type yesno -title Warning -icon question "
                              "-message {Module '", object->getFullName(), "' ", problem,
                              ", it may take a long time to display the graphics. "
                              "Do you want to proceed with drawing?}", NULL));
       bool answer = (Tcl_GetStringResult(interp)[0]=='y');
       if (answer==false)
       {
           not_drawn = true;
           CHK(Tcl_VarEval(interp, canvas, " delete all",NULL)); // this must be done, still
           return;
       }
   }

   // go to next seed
   random_seed++;
   recalculateLayout();
   redrawModules();
   redrawNextEventMarker();
   redrawMessages();
   updateSubmodules();
}

void TGraphicalModWindow::redrawAll()
{
   refreshLayout();
   redrawModules();
   redrawNextEventMarker();
   redrawMessages();
   updateSubmodules();
}

void TGraphicalModWindow::getSubmoduleCoords(cModule *submod, bool& explicitcoords, bool& obeyslayout,
                                                              double& x, double& y, double& sx, double& sy)
{
    const cDisplayString blank;
    const cDisplayString& ds = submod->hasDisplayString() && submod->parametersFinalized() ? submod->getDisplayString() : blank;

    // get size -- we'll need to return that too, and may be needed for matrix, ring etc. layout
    double boxsx=0, boxsy=0;
    int iconsx=0, iconsy=0;
    if (ds.containsTag("b") || !ds.containsTag("i"))
    {
        boxsx = resolveDoubleDispStrArg(ds.getTagArg("b",0), submod, 40);
        boxsy = resolveDoubleDispStrArg(ds.getTagArg("b",1), submod, 24);
    }
    if (ds.containsTag("i"))
    {
        const char *imgname = ds.getTagArg("i",0);
        const char *imgsize = ds.getTagArg("is",0);
        if (!imgname || !*imgname)
        {
            iconsx = UNKNOWNICON_WIDTH;
            iconsy = UNKNOWNICON_HEIGHT;
        }
        else
        {
            Tcl_Interp *interp = getTkenv()->getInterp();
            Tcl_VarEval(interp, "lookupImage ", imgname, " ", imgsize, NULL);
            Tk_Image img = Tk_GetImage(interp, Tk_MainWindow(interp), Tcl_GetStringResult(interp), NULL, NULL);
            if (!img)
            {
                iconsx = UNKNOWNICON_WIDTH;
                iconsy = UNKNOWNICON_HEIGHT;
            }
            else
            {
                Tk_SizeOfImage(img, &iconsx, &iconsy);
                Tk_FreeImage(img);
            }
        }
    }
    sx = (boxsx>iconsx) ? boxsx : iconsx;
    sy = (boxsy>iconsy) ? boxsy : iconsy;

    // first, see if there's an explicit position ("p=" tag) given
    x = resolveDoubleDispStrArg(ds.getTagArg("p",0), submod, -1);
    y = resolveDoubleDispStrArg(ds.getTagArg("p",1), submod, -1);
    explicitcoords = x!=-1 && y!=-1;

    // set missing coordinates to zero
    if (x==-1) x = 0;
    if (y==-1) y = 0;

    const char *layout = ds.getTagArg("p",2); // matrix, row, column, ring, exact etc.
    obeyslayout = (layout && *layout);

    // modify x,y using predefined layouts
    if (!layout || !*layout)
    {
        // we're happy
    }
    else if (!strcmp(layout,"e") || !strcmp(layout,"x") || !strcmp(layout,"exact"))
    {
        double dx = resolveDoubleDispStrArg(ds.getTagArg("p",3), submod, 0);
        double dy = resolveDoubleDispStrArg(ds.getTagArg("p",4), submod, 0);
        x += dx;
        y += dy;
    }
    else if (!strcmp(layout,"r") || !strcmp(layout,"row"))
    {
        // perhaps we should use the size of the 1st element in the vector?
        double dx = resolveDoubleDispStrArg(ds.getTagArg("p",3), submod, 2*sx);
        x += submod->getIndex()*dx;
    }
    else if (!strcmp(layout,"c") || !strcmp(layout,"col") || !strcmp(layout,"column"))
    {
        double dy = resolveDoubleDispStrArg(ds.getTagArg("p",3), submod, 2*sy);
        y += submod->getIndex()*dy;
    }
    else if (!strcmp(layout,"m") || !strcmp(layout,"matrix"))
    {
        // perhaps we should use the size of the 1st element in the vector?
        int columns = resolveLongDispStrArg(ds.getTagArg("p",3), submod, 5);
        double dx = resolveDoubleDispStrArg(ds.getTagArg("p",4), submod, 2*sx);
        double dy = resolveDoubleDispStrArg(ds.getTagArg("p",5), submod, 2*sy);
        if (columns < 1) columns = 1;
        x += (submod->getIndex() % columns)*dx;
        y += (submod->getIndex() / columns)*dy;
    }
    else if (!strcmp(layout,"i") || !strcmp(layout,"ri") || !strcmp(layout,"ring"))
    {
        // perhaps we should use the size of the 1st element in the vector?
        double rx = resolveDoubleDispStrArg(ds.getTagArg("p",3), submod, (sx+sy)*submod->size()/4);
        double ry = resolveDoubleDispStrArg(ds.getTagArg("p",4), submod, rx);

        x += rx - rx*sin(submod->getIndex()*2*PI/submod->size());
        y += ry - ry*cos(submod->getIndex()*2*PI/submod->size());
    }
    else
    {
        Tcl_Interp *interp = getTkenv()->getInterp();
        CHK(Tcl_VarEval(interp,"messagebox {Error} "
                        "{Error: invalid layout `", layout, "' in `p' tag "
                        "of display string \"", ds.str(), "\"} error ok", NULL));
    }
}

void TGraphicalModWindow::recalculateLayout()
{
    // refresh layout with empty submodPosMap -- everything layouted
    submodPosMap.clear();
    refreshLayout();
}

void TGraphicalModWindow::refreshLayout()
{
    // recalculate layout, using coordinates in submodPosMap as "fixed" nodes --
    // only new nodes are re-layouted

    cModule *parentmodule = static_cast<cModule *>(object);

    // Note trick avoid calling getDisplayString() directly because it'd cause
    // the display string object inside cModule to spring into existence
    const cDisplayString blank;
    const cDisplayString& ds = parentmodule->hasDisplayString() && parentmodule->parametersFinalized() ? parentmodule->getDisplayString() : blank;

    // create and configure layouter object
    Tkenv::LayouterChoice choice = getTkenv()->opt_layouterchoice;
    if (choice==Tkenv::LAYOUTER_AUTO)
    {
        const int LIMIT = 20; // note: on test/anim/dynamic2, Advanced is already very slow with 30-40 modules
        int submodCountLimited = 0;
        for (cModule::SubmoduleIterator submod(parentmodule); !submod.end() && submodCountLimited<LIMIT; submod++)
            submodCountLimited++;
        choice = submodCountLimited>=LIMIT ? Tkenv::LAYOUTER_FAST : Tkenv::LAYOUTER_ADVANCED;
    }
    GraphLayouter *layouter = choice==Tkenv::LAYOUTER_FAST ?
                                    (GraphLayouter *) new BasicSpringEmbedderLayout() :
                                    (GraphLayouter *) new ForceDirectedGraphLayouter();

    layouter->setSeed(random_seed);

    // background size
    int sx = resolveLongDispStrArg(ds.getTagArg("bgb",0), parentmodule, 0);
    int sy = resolveLongDispStrArg(ds.getTagArg("bgb",1), parentmodule, 0);
    int border = 30;
    if (sx!=0 && sx < 2*border)
        border = sx/2;
    if (sy!=0 && sy < 2*border)
        border = sy/2;
    layouter->setSize(sx, sy, border);
    // TODO support "bgp" tag ("background position")
    // TODO: scaling ("bgs") support for layouter.
    // Layouter algorithm is NOT scale-independent, so we should divide ALL coordinates
    // by "scale" before passing them to the layouter, then multiply back the results.

    // loop through all submodules, get their sizes and positions and feed them into layouting engine
    for (cModule::SubmoduleIterator it(parentmodule); !it.end(); it++)
    {
        cModule *submod = it();

        bool explicitcoords, obeyslayout;
        double x, y, sx, sy;
        getSubmoduleCoords(submod, explicitcoords, obeyslayout, x, y, sx, sy);

        // add node into layouter:
        if (explicitcoords)
        {
            // e.g. "p=120,70" or "p=140,30,ring"
            layouter->addFixedNode(submod->getId(), x, y, sx, sy);
        }
        else if (submodPosMap.find(submod)!=submodPosMap.end())
        {
            // reuse coordinates from previous layout
            Point pos = submodPosMap[submod];
            layouter->addFixedNode(submod->getId(), pos.x, pos.y, sx, sy);
        }
        else if (obeyslayout)
        {
            // all modules are anchored to the anchor point with the vector's name
            // e.g. "p=,,ring"
            layouter->addAnchoredNode(submod->getId(), submod->getName(), x, y, sx, sy);
        }
        else
        {
            layouter->addMovableNode(submod->getId(), sx, sy);
        }
    }

    // add connections into the layouter, too
    bool parent = false;
    for (cModule::SubmoduleIterator it(parentmodule); !parent; it++)
    {
        cModule *mod = !it.end() ? it() : (parent=true,parentmodule);

        for (cModule::GateIterator i(mod); !i.end(); i++)
        {
            cGate *gate = i();
            cGate *destgate = gate->getNextGate();
            if (gate->getType()==(parent ? cGate::INPUT : cGate::OUTPUT) && destgate)
            {
                cModule *destmod = destgate->getOwnerModule();
                if (mod==parentmodule && destmod==parentmodule) {
                    // nop
                } else if (destmod==parentmodule) {
                    layouter->addEdgeToBorder(mod->getId());
                } else if (destmod->getParentModule()!=parentmodule) {
                    // connection goes to a module under a different parent!
                    // this in fact violates module encapsulation, but let's
                    // accept it nevertheless. Just skip this connection.
                } else if (mod==parentmodule) {
                    layouter->addEdgeToBorder(destmod->getId());
                } else {
                    layouter->addEdge(mod->getId(), destmod->getId());
                }
            }
        }
    }

    // set up layouter environment (responsible for "Stop" button handling and visualizing the layouting process)
    Tcl_Interp *interp = getTkenv()->getInterp();
    TGraphLayouterEnvironment environment(interp, parentmodule, ds);

    std::string stopButton = std::string(windowName()) + ".toolbar.stop";
    bool isExpressMode = getTkenv()->getSimulationRunMode() == Tkenv::RUNMODE_EXPRESS;
    if (!isExpressMode)
        environment.setWidgetToGrab(stopButton.c_str());

    // enable visualizing only if full re-layouting (no cached coordinates in submodPosMap)
    // if (getTkenv()->opt_showlayouting)  // for debugging
    if (submodPosMap.empty() && getTkenv()->opt_showlayouting)
        environment.setCanvas(canvas);

    layouter->setEnvironment(&environment);
    layouter->execute();
    environment.cleanup();

    // fill the map with the results
    submodPosMap.clear();
    for (cModule::SubmoduleIterator it(parentmodule); !it.end(); it++)
    {
        cModule *submod = it();

        Point pos;
        layouter->getNodePosition(submod->getId(), pos.x, pos.y);
        submodPosMap[submod] = pos;
    }

    random_seed = layouter->getSeed();

    delete layouter;
}

// requires either recalculateLayout() or refreshLayout() called before!
void TGraphicalModWindow::redrawModules()
{
    cModule *parentmodule = static_cast<cModule *>(object);
    Tcl_Interp *interp = getTkenv()->getInterp();

    // then display all submodules
    CHK(Tcl_VarEval(interp, canvas, " delete dx",NULL)); // NOT "delete all" because that'd remove "bubbles" too!
    const cDisplayString blank;
    std::string buffer;
    const char *rawScaling = parentmodule->hasDisplayString() && parentmodule->parametersFinalized() ? parentmodule->getDisplayString().getTagArg("bgs",0) : "";
    const char *scaling = substituteDisplayStringParamRefs(rawScaling, buffer, parentmodule, true);

    for (cModule::SubmoduleIterator it(parentmodule); !it.end(); it++)
    {
        cModule *submod = it();
        assert(submodPosMap.find(submod)!=submodPosMap.end());
        Point& pos = submodPosMap[submod];
        drawSubmodule(interp, submod, pos.x, pos.y, scaling);
    }

    // draw enclosing module
    drawEnclosingModule(interp, parentmodule, scaling);

    // loop through all submodules and enclosing module & draw their connections
    bool atparent = false;
    for (cModule::SubmoduleIterator it(parentmodule); !atparent; it++)
    {
        cModule *mod = !it.end() ? it() : (atparent=true,parentmodule);

        for (cModule::GateIterator i(mod); !i.end(); i++)
        {
            cGate *gate = i();
            if (gate->getType()==(atparent ? cGate::INPUT: cGate::OUTPUT) && gate->getNextGate()!=NULL)
            {
                drawConnection(interp, gate);
            }
        }
    }
    CHK(Tcl_VarEval(interp, canvas, " raise bubble",NULL));
    CHK(Tcl_VarEval(interp, "graphicalModuleWindow:setScrollRegion ", windowname, " 0",NULL));
}

void TGraphicalModWindow::drawSubmodule(Tcl_Interp *interp, cModule *submod, double x, double y, const char *scaling)
{
    char coords[64];
    sprintf(coords,"%g %g ", x, y);
    const char *dispstr = submod->hasDisplayString() && submod->parametersFinalized() ? submod->getDisplayString().str() : "";

    CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawSubmodule ",
                    canvas, " ",
                    ptrToStr(submod), " ",
                    coords,
                    "{", submod->getFullName(), "} ",
                    TclQuotedString(dispstr).get(), " ",
                    "{", scaling, "} ",
                    (submod->isPlaceholder() ? "1" : "0"),
                    NULL));
}

void TGraphicalModWindow::drawEnclosingModule(Tcl_Interp *interp, cModule *parentmodule, const char *scaling)
{
    const char *dispstr = parentmodule->hasDisplayString() && parentmodule->parametersFinalized() ? parentmodule->getDisplayString().str() : "";
    CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawEnclosingModule ",
                       canvas, " ",
                       ptrToStr(parentmodule), " ",
                       "{", parentmodule->getFullPath().c_str(), "} ",
                       TclQuotedString(dispstr).get(), " ",
                       "{", scaling, "} ",
                       NULL ));
}

void TGraphicalModWindow::drawConnection(Tcl_Interp *interp, cGate *gate)
{
    cModule *mod = gate->getOwnerModule();
    cGate *dest_gate = gate->getNextGate();

    char gateptr[32], srcptr[32], destptr[32], chanptr[32], indices[32];

    // check if this is a two way connection (an other connection is pointing back
    // to the this gate's pair from the next gate's pair)
    bool twoWayConnection = false;
    // check if this gate is really part of an in/out gate pair
    // gate      o-------------------->o dest_gate
    // gate_pair o<--------------------o dest_gate_pair
    if (gate->getNameSuffix()[0]) {
      const cGate *gate_pair = mod->gateHalf(gate->getBaseName(),
                                        gate->getType() == cGate::INPUT ? cGate::OUTPUT : cGate::INPUT,
                                        gate->isVector() ? gate->getIndex() : -1);

      if (dest_gate->getNameSuffix()[0]) {
        const cGate *dest_gate_pair = dest_gate->getOwnerModule()->gateHalf(dest_gate->getBaseName(),
                                            dest_gate->getType() == cGate::INPUT ? cGate::OUTPUT : cGate::INPUT,
                                            dest_gate->isVector() ? dest_gate->getIndex() : -1);
          twoWayConnection = dest_gate_pair == gate_pair->getPreviousGate();
      }
    }


    ptrToStr(gate, gateptr);
    ptrToStr(mod, srcptr);
    ptrToStr(dest_gate->getOwnerModule(), destptr);
    sprintf(indices, "%d %d %d %d",
            gate->getIndex(), gate->size(),
            dest_gate->getIndex(), dest_gate->size());
    cChannel *chan = gate->getChannel();
    ptrToStr(chan, chanptr);
    const char *dispstr = (chan && chan->hasDisplayString() && chan->parametersFinalized()) ? chan->getDisplayString().str() : "";

    CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawConnection ",
            canvas, " ",
            gateptr, " ",
            TclQuotedString(dispstr).get(), " ",
            srcptr, " ",
            destptr, " ",
            chanptr, " ",
            indices, " ",
            twoWayConnection ? "1" : "0",
            NULL
             ));
}

void TGraphicalModWindow::redrawMessages()
{
   Tcl_Interp *interp = getTkenv()->getInterp();

   // refresh & cleanup from prev. events
   CHK(Tcl_VarEval(interp, canvas, " delete msg msgname", NULL));

   // this thingy is only needed if animation is going on
   if (!getTkenv()->animating)
       return;

   // loop through all messages in the event queue and display them
   for (cMessageHeap::Iterator msg(simulation.msgQueue); !msg.end(); msg++)
   {
      char msgptr[32];
      ptrToStr(msg(),msgptr);

      cModule *arrivalmod = simulation.getModule( msg()->getArrivalModuleId() );
      if (arrivalmod &&
          arrivalmod->getParentModule()==static_cast<cModule *>(object) &&
          msg()->getArrivalGateId()>=0)
      {
         cGate *arrivalGate = msg()->getArrivalGate();

         // if arrivalgate is connected, msg arrived on a connection, otherwise via sendDirect()
         if (arrivalGate->getPreviousGate())
         {
             cGate *gate = arrivalGate->getPreviousGate();
             CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawMessageOnGate ",
                             canvas, " ",
                             ptrToStr(gate), " ",
                             msgptr,
                             NULL));
         }
         else
         {
             CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawMessageOnModule ",
                             canvas, " ",
                             ptrToStr(arrivalmod), " ",
                             msgptr,
                             NULL));
         }
      }
   }
   CHK(Tcl_VarEval(interp, canvas, " raise bubble",NULL));
}

void TGraphicalModWindow::redrawNextEventMarker()
{
   Tcl_Interp *interp = getTkenv()->getInterp();
   cModule *mod = static_cast<cModule *>(object);

   // removing marker from previous event
   CHK(Tcl_VarEval(interp, canvas, " delete nexteventmarker", NULL));

   // this thingy is only needed if animation is going on
   if (!getTkenv()->animating || !getTkenv()->opt_nexteventmarkers)
       return;

   // if any parent of the module containing the next event is on this canvas, draw marker
   cModule *nextmod = simulation.guessNextModule();
   cModule *nextmodparent = nextmod;
   while (nextmodparent && nextmodparent->getParentModule()!=mod)
       nextmodparent = nextmodparent->getParentModule();
   if (nextmodparent)
   {
       CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawNextEventMarker ",
                       canvas, " ",
                       ptrToStr(nextmodparent), " ",
                       (nextmod==nextmodparent ? "2" : "1"),
                       NULL));
   }
}

void TGraphicalModWindow::updateSubmodules()
{
   Tcl_Interp *interp = getTkenv()->getInterp();
   for (cModule::SubmoduleIterator submod(static_cast<cModule *>(object)); !submod.end(); submod++)
   {
       CHK(Tcl_VarEval(interp, "graphicalModuleWindow:updateSubmodule ",
                       canvas, " ",
                       ptrToStr(submod()),
                       NULL));
   }
}


void TGraphicalModWindow::submoduleCreated(cModule *newmodule)
{
   needs_redraw = true;
}

void TGraphicalModWindow::submoduleDeleted(cModule *module)
{
   needs_redraw = true;
}

void TGraphicalModWindow::connectionCreated(cGate *srcgate)
{
   needs_redraw = true;
}

void TGraphicalModWindow::connectionDeleted(cGate *srcgate)
{
   needs_redraw = true;
}

void TGraphicalModWindow::displayStringChanged(cModule *)
{
   needs_redraw = true;
}

void TGraphicalModWindow::displayStringChanged()
{
   needs_redraw = true; //TODO check, probably only non-background tags have changed...
}

void TGraphicalModWindow::displayStringChanged(cGate *)
{
   needs_redraw = true;
}

void TGraphicalModWindow::bubble(cModule *submod, const char *text)
{
    Tcl_Interp *interp = getTkenv()->getInterp();

    // if submod position is not yet known (because e.g. we're in fast mode
    // and it was dynamically created since the last update), refresh layout
    // so that we can get coordinates for it
    if (submodPosMap.find(submod)==submodPosMap.end())
        refreshLayout();

    cModule *parentmodule = static_cast<cModule *>(object);
    std::string buffer;
    const char *rawScaling = parentmodule->hasDisplayString() && parentmodule->parametersFinalized() ? parentmodule->getDisplayString().getTagArg("bgs",0) : "";
    const char *scaling = substituteDisplayStringParamRefs(rawScaling, buffer, parentmodule, true);

    // invoke Tcl code to display bubble
    char coords[64];
    Point& pos = submodPosMap[submod];
    sprintf(coords, " %g %g ", pos.x, pos.y);
    CHK(Tcl_VarEval(interp, "graphicalModuleWindow:bubble ", canvas, coords, " ", TclQuotedString(scaling).get(), " ", TclQuotedString(text).get(), NULL));
}

int TGraphicalModWindow::inspectorCommand(Tcl_Interp *interp, int argc, const char **argv)
{
   if (argc<1) {Tcl_SetResult(interp, TCLCONST("wrong number of args"), TCL_STATIC); return TCL_ERROR;}

   // supported commands:
   //   arrowcoords, relayout, etc...

   if (strcmp(argv[0],"arrowcoords")==0)
   {
      return arrowcoords(interp,argc,argv);
   }
   else if (strcmp(argv[0],"relayout")==0)
   {
      TRY(relayoutAndRedrawAll());
      return TCL_OK;
   }
   else if (strcmp(argv[0],"redraw")==0)
   {
      TRY(redrawAll());
      return TCL_OK;
   }
   else if (strcmp(argv[0],"submodulecount")==0)
   {
      return getSubmoduleCount(interp,argc,argv);
   }
   else if (strcmp(argv[0],"getsubmodq")==0)
   {
      return getSubmodQ(interp,argc,argv);
   }
   else if (strcmp(argv[0],"getsubmodqlen")==0)
   {
      return getSubmodQLen(interp,argc,argv);
   }
   return TCL_ERROR;
}

int TGraphicalModWindow::getSubmoduleCount(Tcl_Interp *interp, int argc, const char **argv)
{
   int count = 0;
   for (cModule::SubmoduleIterator submod(static_cast<cModule *>(object)); !submod.end(); submod++)
       count++;
   char buf[20];
   sprintf(buf, "%d", count);
   Tcl_SetResult(interp, buf, TCL_VOLATILE);
   return TCL_OK;
}

int TGraphicalModWindow::getSubmodQ(Tcl_Interp *interp, int argc, const char **argv)
{
   // args: <module ptr> <qname>
   if (argc!=3) {Tcl_SetResult(interp, TCLCONST("wrong number of args"), TCL_STATIC); return TCL_ERROR;}

   cModule *mod = dynamic_cast<cModule *>(strToPtr( argv[1] ));
   const char *qname = argv[2];
   cQueue *q = dynamic_cast<cQueue *>(mod->findObject(qname));
   char buf[21];
   ptrToStr(q,buf);
   Tcl_SetResult(interp, buf, TCL_VOLATILE);
   return TCL_OK;
}

int TGraphicalModWindow::getSubmodQLen(Tcl_Interp *interp, int argc, const char **argv)
{
   // args: <module ptr> <qname>
   if (argc!=3) {Tcl_SetResult(interp, TCLCONST("wrong number of args"), TCL_STATIC); return TCL_ERROR;}

   cModule *mod = dynamic_cast<cModule *>(strToPtr( argv[1] ));
   const char *qname = argv[2];
   cQueue *q = dynamic_cast<cQueue *>(mod->findObject(qname)); //FIXME THIS MUST BE REFINED! SEARCHES WAY TOO DEEEEEP!!!!
   if (!q) {Tcl_SetResult(interp, TCLCONST(""), TCL_STATIC); return TCL_OK;}

   char buf[20];
   sprintf(buf, "%d", q->length());
   Tcl_SetResult(interp, buf, TCL_VOLATILE);
   return TCL_OK;
}


//=======================================================================

//
// class TCompoundModInspectorFactory : public cInspectorFactory
// {
//   public:
//     TCompoundModInspectorFactory(const char *name) : cInspectorFactory(name) {}
//
//     bool supportsObject(cObject *obj) {return dynamic_cast<cModule *>(obj)!=NULL;}
//     int inspectorType() {return INSP_OBJECT;}
//     double qualityAsDefault(cObject *object) {return 2.9;}
//
//     TInspector *createInspectorFor(cObject *object,int type,const char *geom,void *data) {
//         return new TCompoundModInspector(object, type, geom, data);
//     }
// };
//
// Register_InspectorFactory(TCompoundModInspectorFactory);
//
//
// TCompoundModInspector::TCompoundModInspector(cObject *obj,int typ,const char *geom,void *dat) :
//     TInspector(obj,typ,geom,dat)
// {
// }
//
// void TCompoundModInspector::createWindow()
// {
//    TInspector::createWindow(); // create window name etc.
//
//    // create inspector window by calling the specified proc with
//    // the object's pointer. Window name will be like ".ptr80003a9d-1"
//    Tcl_Interp *interp = getTkenv()->getInterp();
//    CHK(Tcl_VarEval(interp, "createCompoundModuleInspector ", windowname, " \"", geometry, "\"", NULL ));
// }
//
// void TCompoundModInspector::update()
// {
//    TInspector::update();
//
//    cCompoundModule *mod = static_cast<cCompoundModule *>(object);
//
//    //setToolbarInspectButton(".toolbar.parent", mod->getParentModule(),INSP_DEFAULT);
//
//    setEntry(".nb.info.name.e", mod->getName());
//    char id[16]; sprintf(id,"%ld", (long)mod->getId());
//    setLabel(".nb.info.id.e", id);
//    setEntry(".nb.info.dispstr.e", mod->getDisplayString());
//    setEntry(".nb.info.dispstrpt.e", mod->backgroundDisplayString());
//
//    deleteInspectorListbox(".nb.contents");
//    fillInspectorListbox(".nb.contents", mod, false);
// }
//
// void TCompoundModInspector::writeBack()
// {
//    cCompoundModule *mod = static_cast<cCompoundModule *>(object);
//    mod->setName(getEntry(".nb.info.name.e"));
//    mod->getDisplayString().parse(getEntry(".nb.info.dispstr.e"));
//    mod->backgroundDisplayString().parse(getEntry(".nb.info.dispstrpt.e"));
//
//    TInspector::writeBack();    // must be there after all changes
// }
//

//=======================================================================

//
// class TSimpleModInspectorFactory : public cInspectorFactory
// {
//   public:
//     TSimpleModInspectorFactory(const char *name) : cInspectorFactory(name) {}
//
//     bool supportsObject(cObject *obj) {return dynamic_cast<cSimpleModule *>(obj)!=NULL;}
//     int inspectorType() {return INSP_OBJECT;}
//     double qualityAsDefault(cObject *object) {return 4.0;}
//
//     TInspector *createInspectorFor(cObject *object,int type,const char *geom,void *data) {
//         return new TSimpleModInspector(object, type, geom, data);
//     }
// };
//
// Register_InspectorFactory(TSimpleModInspectorFactory);
//
//
// TSimpleModInspector::TSimpleModInspector(cObject *obj,int typ,const char *geom,void *dat) :
//     TInspector(obj,typ,geom,dat)
// {
// }
//
// void TSimpleModInspector::createWindow()
// {
//    TInspector::createWindow(); // create window name etc.
//
//    // create inspector window by calling the specified proc with
//    // the object's pointer. Window name will be like ".ptr80003a9d-1"
//    Tcl_Interp *interp = getTkenv()->getInterp();
//    CHK(Tcl_VarEval(interp, "createSimpleModuleInspector ", windowname, " \"", geometry, "\"", NULL ));
// }
//
// void TSimpleModInspector::update()
// {
//    TInspector::update();
//
//    cSimpleModule *mod = static_cast<cSimpleModule *>(object);
//
//    char buf[40];
//    setEntry(".nb.info.name.e", mod->getName());
//    sprintf(buf,"%ld", (long)mod->getId());
//    setLabel(".nb.info.id.e", buf);
//    setEntry(".nb.info.dispstr.e", mod->getDisplayString());
//    setEntry(".nb.info.dispstrpt.e", mod->backgroundDisplayString());
//    setLabel(".nb.info.state.e",  modstate[ mod->moduleState() ]  );
//    if (mod->usesActivity())
//    {
//       unsigned stk = mod->getStackSize();
//       unsigned extra = ev.getExtraStackForEnvir();
//       unsigned used = mod->getStackUsage();
//       sprintf(buf,"%u + %u = %u bytes", stk-extra, extra, stk);
//       setLabel(".nb.info.stacksize.e", buf );
//       sprintf(buf,"approx. %u bytes", used);
//       setLabel(".nb.info.stackused.e", buf );
//    }
//    else
//    {
//       setLabel(".nb.info.stacksize.e", "n/a" );
//       setLabel(".nb.info.stackused.e", "n/a" );
//    }
//
//    deleteInspectorListbox(".nb.contents");
//    fillInspectorListbox(".nb.contents", mod, false);
// }
//
// void TSimpleModInspector::writeBack()
// {
//    cSimpleModule *mod = static_cast<cSimpleModule *>(object);
//    mod->setName(getEntry(".nb.info.name.e"));
//    mod->getDisplayString().parse(getEntry(".nb.info.dispstr.e"));
//    mod->backgroundDisplayString().parse(getEntry(".nb.info.dispstrpt.e"));
//
//    TInspector::writeBack();    // must be there after all changes
// }
//

//=======================================================================

//
// class TGateInspectorFactory : public cInspectorFactory
// {
//   public:
//     TGateInspectorFactory(const char *name) : cInspectorFactory(name) {}
//
//     bool supportsObject(cObject *obj) {return dynamic_cast<cGate *>(obj)!=NULL;}
//     int inspectorType() {return INSP_OBJECT;}
//     double qualityAsDefault(cObject *object) {return 2.9;}
//
//     TInspector *createInspectorFor(cObject *object,int type,const char *geom,void *data) {
//         return new TGateInspector(object, type, geom, data);
//     }
// };
//
// Register_InspectorFactory(TGateInspectorFactory);
//
// TGateInspector::TGateInspector(cObject *obj,int typ,const char *geom,void *dat) :
//     TInspector(obj,typ,geom,dat)
// {
// }
//
// void TGateInspector::createWindow()
// {
//    TInspector::createWindow(); // create window name etc.
//
//    // create inspector window by calling the specified proc with
//    // the object's pointer. Window name will be like ".ptr80003a9d-1"
//    Tcl_Interp *interp = getTkenv()->getInterp();
//    CHK(Tcl_VarEval(interp, "createGateInspector ", windowname, " \"", geometry, "\"", NULL ));
// }
//
// void TGateInspector::update()
// {
//    TInspector::update();
//
//    cGate *g = static_cast<cGate *>(object);
//
//    setEntry(".nb.info.name.e", g->getName());
//    char buf[64];
//    sprintf(buf,"#%d", g->getId());
//    setLabel(".nb.info.id.e", buf);
//    setEntry(".nb.info.dispstr.e", g->getDisplayString().str());
//    cDatarateChannel *ch = dynamic_cast<cDatarateChannel*>(g->getChannel());
//    if (ch)
//    {
//        setEntry(".nb.info.delay.e", ch->getDelay());
//        setEntry(".nb.info.error.e", ch->getError());
//        setEntry(".nb.info.datarate.e", ch->getDatarate());
//    }
//    else
//    {
//        setEntry(".nb.info.delay.e", 0.0);
//        setEntry(".nb.info.error.e", 0.0);
//        setEntry(".nb.info.datarate.e", 0.0);
//    }
//    setLabel(".nb.info.trfinish.e", g->getTransmissionFinishTime());
//
//    setInspectButton(".nb.info.from", g->getPreviousGate(), true, INSP_DEFAULT);
//    setInspectButton(".nb.info.to", g->getNextGate(), true, INSP_DEFAULT);
// }
//
// void TGateInspector::writeBack()
// {
//    cGate *g = static_cast<cGate *>(object);
//    g->setName(getEntry(".nb.info.name.e"));
//    g->getDisplayString().parse(getEntry(".nb.info.dispstr.e"));
//    cDatarateChannel *ch = dynamic_cast<cDatarateChannel*>(g->getChannel());
//    double delay = atof(getEntry(".nb.info.delay.e"));
//    double error = atof(getEntry(".nb.info.error.e"));
//    double datarate = atof(getEntry(".nb.info.datarate.e"));
//    if (delay!=0 || error!=0 || datarate!=0 || ch!=NULL)
//    {
//        if (!ch)
//        {
//            ch = new cDatarateChannel("channel");
//            g->setChannel(ch);
//        }
//        ch->setDelay(delay<0 ? 0 : delay);
//        ch->setError(error<0 ? 0 : error>1 ? 1 : error);
//        ch->setDatarate(datarate<0 ? 0 : datarate);
//    }
//
//    TInspector::writeBack();    // must be there after all changes
// }
//

//=======================================================================
class TGraphicalGateWindowFactory : public cInspectorFactory
{
  public:
    TGraphicalGateWindowFactory(const char *name) : cInspectorFactory(name) {}

    bool supportsObject(cObject *obj) {return dynamic_cast<cGate *>(obj)!=NULL;}
    int inspectorType() {return INSP_GRAPHICAL;}
    double qualityAsDefault(cObject *object) {return 3.0;}

    TInspector *createInspectorFor(cObject *object,int type,const char *geom,void *data) {
        return new TGraphicalGateWindow(object, type, geom, data);
    }
};

Register_InspectorFactory(TGraphicalGateWindowFactory);


TGraphicalGateWindow::TGraphicalGateWindow(cObject *obj,int typ,const char *geom,void *dat) :
    TInspector(obj,typ,geom,dat)
{
}

void TGraphicalGateWindow::createWindow()
{
   TInspector::createWindow(); // create window name etc.
   strcpy(canvas,windowname); strcat(canvas,".c");

   // create inspector window by calling the specified proc with
   // the object's pointer. Window name will be like ".ptr80003a9d-1"
   Tcl_Interp *interp = getTkenv()->getInterp();
   CHK(Tcl_VarEval(interp, "createGraphicalGateWindow ", windowname, " \"", geometry, "\"", NULL ));
}

int TGraphicalGateWindow::redraw(Tcl_Interp *interp, int, const char **)
{
   cGate *gate = (cGate *)object;

   CHK(Tcl_VarEval(interp, canvas, " delete all",NULL));

   // draw modules
   int k = 0;
   int xsiz = 0;
   char prevdir = ' ';
   cGate *g;
   for (g = gate->getPathStartGate(); g!=NULL; g=g->getNextGate(),k++)
   {
        if (g->getType()==prevdir)
             xsiz += (g->getType()==cGate::OUTPUT) ? 1 : -1;
        else
             prevdir = g->getType();

        char modptr[32], gateptr[32], kstr[16], xstr[16], dir[2];
        ptrToStr(g->getOwnerModule(),modptr);
        ptrToStr(g,gateptr);
        sprintf(kstr,"%d",k);
        sprintf(xstr,"%d",xsiz);
        dir[0] = g->getType(); dir[1]=0;
        CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawModuleGate ",
                      canvas, " ",
                      modptr, " ",
                      gateptr, " ",
                      "{",g->getOwnerModule()->getFullPath().c_str(), "} ",
                      "{",g->getFullName(), "} ",
                      kstr," ",
                      xstr," ",
                      dir, " ",
                      g==gate?"1":"0",
                      NULL ));
   }

   // draw connections
   for (g = gate->getPathStartGate(); g->getNextGate()!=NULL; g=g->getNextGate())
   {
        char srcgateptr[32], destgateptr[32], chanptr[32];
        ptrToStr(g,srcgateptr);
        ptrToStr(g->getNextGate(),destgateptr);
        cChannel *chan = g->getChannel();
        ptrToStr(chan,chanptr);
        const char *dispstr = (chan && chan->hasDisplayString() && chan->parametersFinalized() ) ? chan->getDisplayString().str() : "";
        CHK(Tcl_VarEval(interp, "graphGateWin:drawConnection ",
                      canvas, " ",
                      srcgateptr, " ",
                      destgateptr, " ",
                      chanptr, " ",
                      TclQuotedString(chan?chan->info().c_str():"").get(), " ",
                      TclQuotedString(dispstr).get(), " ",
                      NULL ));
   }

   // loop through all messages in the event queue
   update();

   return TCL_OK;
}

void TGraphicalGateWindow::update()
{
   TInspector::update();

   Tcl_Interp *interp = getTkenv()->getInterp();
   cGate *gate = static_cast<cGate *>(object);

   // redraw modules only on explicit request

   // loop through all messages in the event queue
   CHK(Tcl_VarEval(interp, canvas, " delete msg msgname", NULL));
   cGate *destgate = gate->getPathEndGate();
   for (cMessageHeap::Iterator msg(simulation.msgQueue); !msg.end(); msg++)
   {
      char gateptr[32], msgptr[32];
      ptrToStr(msg(),msgptr);

      if (msg()->getArrivalGate()== destgate)
      {
         cGate *gate = msg()->getArrivalGate();
         if (gate) gate = gate->getPreviousGate();
         if (gate)
         {
             CHK(Tcl_VarEval(interp, "graphicalModuleWindow:drawMessageOnGate ",
                             canvas, " ",
                             ptrToStr(gate,gateptr), " ",
                             msgptr,
                             NULL));
         }
      }
   }
}

int TGraphicalGateWindow::inspectorCommand(Tcl_Interp *interp, int argc, const char **argv)
{
   if (argc<1) {Tcl_SetResult(interp, TCLCONST("wrong number of args"), TCL_STATIC); return TCL_ERROR;}

   // supported commands:
   //   redraw

   if (strcmp(argv[0],"redraw")==0)
   {
      return redraw(interp,argc,argv);
   }

   Tcl_SetResult(interp, TCLCONST("invalid arg: must be 'redraw'"), TCL_STATIC);
   return TCL_ERROR;
}

void TGraphicalGateWindow::displayStringChanged(cGate *gate)
{
   //XXX should defer redraw (via redraw_needed) to avoid "flickering"
}

NAMESPACE_END

