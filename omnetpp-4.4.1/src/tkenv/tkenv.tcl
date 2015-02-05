#=================================================================
#  TKENV.TCL - part of
#
#                     OMNeT++/OMNEST
#            Discrete System Simulation in C++
#
#=================================================================

#----------------------------------------------------------------#
#  Copyright (C) 1992-2008 Andras Varga
#
#  This file is distributed WITHOUT ANY WARRANTY. See the file
#  `license' for details on this and other legal matters.
#----------------------------------------------------------------#

#
# Load library files
#
if [info exist OMNETPP_TKENV_DIR] {

   set dir $OMNETPP_TKENV_DIR

   source [file join $dir combobox.tcl]
   source [file join $dir icons.tcl]
   source [file join $dir widgets.tcl]
   source [file join $dir balloon.tcl]
   source [file join $dir dialogs.tcl]
   source [file join $dir windows.tcl]
   source [file join $dir inspector.tcl]
   source [file join $dir fieldspage.tcl]
   source [file join $dir insplist.tcl]
   source [file join $dir objinsp.tcl]
   source [file join $dir modinsp1.tcl]
   source [file join $dir modinsp2.tcl]
   source [file join $dir animate.tcl]
   source [file join $dir animate2.tcl]
   source [file join $dir gateinsp.tcl]
   source [file join $dir statinsp.tcl]
   source [file join $dir menuproc.tcl]
   source [file join $dir main.tcl]
   source [file join $dir tree.tcl]
   source [file join $dir treemgr.tcl]
   source [file join $dir canvlbl.tcl]
   source [file join $dir timeline.tcl]
   source [file join $dir tkenvrc.tcl]

}

#
# Execute startup code
#
proc startTkenv {} {
  global OMNETPP_IMAGE_PATH
  global OMNETPP_PLUGIN_PATH
  global HAVE_BLT

  wm withdraw .
  checkTclTkVersion
  setupTkOptions
  genericBindings
  initBalloons
  createOmnetppWindow
  puts ""
  loadBitmaps $OMNETPP_IMAGE_PATH
  loadPlugins $OMNETPP_PLUGIN_PATH
  loadTkenvrc "~/.tkenvrc"
  loadTkenvrc ".tkenvrc"
  reflectSettingsInGui
  determineClocksPerSec

  global tcl_platform
  if {$tcl_platform(platform) == "windows"} {
      # without "update", the main window comes up on top of all others, it also
      # obscures any dialog box (e.g. param prompt) that might appear at startup!
      update
  }

}

