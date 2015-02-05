#=================================================================
#  WINDOWS.TCL - part of
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

#------
# Parts of this file were created using Stewart Allen's Visual Tcl (vtcl)
#------

#===================================================================
#    MESSAGE WINDOW - IT IS NOT INSPECTOR!
#===================================================================

proc createMessageWindow {name} {
    set w $name
    if {[winfo exists $w]} {
        wm deiconify $w; return
    }

    global icons fonts

    # creating widgets
    toplevel $w -class Toplevel
    wm focusmodel $w passive
    #wm maxsize $w 1009 738
    wm minsize $w 1 1
    wm overrideredirect $w 0
    wm resizable $w 1 1
    wm title $w "Sent/Delivered Messages"

    wm protocol $w WM_DELETE_WINDOW "opp_setmsgwindowexists 0; destroy $w"

    frame $w.toolbar
    textWindowAddIcons $w

    frame $w.main
    text $w.main.text -yscrollcommand "$w.main.sb set" -width 88 -height 15 -font $fonts(text)
    scrollbar $w.main.sb -command "$w.main.text yview"
    logTextWidget:configureTags $w.main.text

    # setting geometry
    pack $w.toolbar  -anchor center -expand 0 -fill x -side top
    pack $w.main -anchor center -expand 1 -fill both -side top
    pack $w.main.sb -anchor center -expand 0 -fill y -ipadx 0 -ipady 0 -padx 0 -pady 0 -side right
    pack $w.main.text -anchor center -expand 1 -fill both -side left

    logTextWidget:configureTags $w.main.text

    # keyboard bindings
    bindRunCommands $w
    bindCommandsToTextWidget $w.main.text

    # let C++ code know about it
    opp_setmsgwindowexists 1
}

#===================================================================
#    CONSOLE WINDOW
#===================================================================

proc showConsole {} {
    set w .con
    if {[winfo exists .con]} {
        wm deiconify .con; return
    }
    toplevel .con -class vTcl
    wm minsize .con 375 160
    wm title .con "Tcl Console"
    frame .con.fra5 \
        -height 30 -width 30
    pack .con.fra5 \
        -anchor center -expand 1 -fill both -ipadx 0 -ipady 0 -padx 2 -pady 2 \
        -side top
    text .con.fra5.tex7 \
        -highlightthickness 0 -state disabled -width 50 -height 6 \
        -yscrollcommand {.con.fra5.scr8 set}
    .con.fra5.tex7 tag configure command -underline 1
    .con.fra5.tex7 tag configure error -foreground red
    .con.fra5.tex7 tag configure output
    pack .con.fra5.tex7 \
        -anchor center -expand 1 -fill both -ipadx 0 -ipady 0 -padx 0 -pady 0 \
        -side left
    scrollbar .con.fra5.scr8 \
        -command {.con.fra5.tex7 yview} -highlightthickness 0
    pack .con.fra5.scr8 \
        -anchor center -expand 0 -fill y -ipadx 0 -ipady 0 -padx 0 -pady 0 \
        -side right
    frame .con.fra6 \
        -height 30 -width 30
    pack .con.fra6 \
        -anchor center -expand 0 -fill both -ipadx 0 -ipady 0 -padx 0 -pady 0 \
        -side top
    entry .con.fra6.ent10 \
        -highlightthickness 0
    pack .con.fra6.ent10 \
        -anchor center -expand 0 -fill x -ipadx 0 -ipady 0 -padx 2 -pady 2 \
        -side top
    bind .con.fra6.ent10 <Key-Return> {
        .con.fra5.tex7 conf -state normal
        .con.fra5.tex7 insert end \n[.con.fra6.ent10 get] command
        if { [catch [.con.fra6.ent10 get] output] == 1 } {
            .con.fra5.tex7 insert end "\n$output" error
        } else {
            .con.fra5.tex7 insert end "\n$output" output
        }
        .con.fra5.tex7 conf -state disabled
        .con.fra5.tex7 yview end
        .con.fra6.ent10 delete 0 end
    }
    focus .con.fra6.ent10
}

#===================================================================
#    FILE VIEWER/EDITOR PROCEDURES
#===================================================================

proc loadFile {win filename} {
    if [catch {open $filename r} f] {
       messagebox {Error} "Error: $f" info ok
       return
    }
    set contents [read $f]
    close $f

    set t $win.main.text
    set curpos [$t index insert]
    $win.main.text delete 1.0 end
    $win.main.text insert end $contents
    catch {$t mark set insert $curpos}
    $t see insert
}

proc saveFile {win {filename ""}} {
    global config

    if {$filename == ""} {
        set filename $config(log-save-filename)
        set filename [tk_getSaveFile -title {Save Log Window Contents} -parent $win \
                      -defaultextension "out" -initialfile $filename \
                      -filetypes {{{Log files} {*.out}} {{All files} {*}}}]
        if {$filename == ""} return
        set config(log-save-filename) $filename
    }

    if [catch {
       set f [open $filename w]
       set txt $win.main.text
       if {$txt == "..main.text"} {set txt .main.text}
       puts -nonewline $f [$txt get 1.0 end]
       close $f
    } err] {
       messagebox {Error} "Error: $err" info ok
    }
}

#
# Open file viewer window
#
proc createFileViewer {filename} {
    global icons fonts help_tips

    if {$filename == ""} return

    # create a widget name from filename
    set w ".win[clock seconds]"
    if {[winfo exists $w]} {destroy $w}

    # creating widgets
    toplevel $w -class Toplevel
    wm focusmodel $w passive
    #wm maxsize $w 1009 738
    wm minsize $w 1 1
    wm overrideredirect $w 0
    wm resizable $w 1 1
    wm title $w $filename

    frame $w.toolbar

    packIconButton $w.toolbar.copy   -image $icons(copy) -command "editCopy $w.main.text"
    packIconButton $w.toolbar.find   -image $icons(find) -command "findDialog $w.main.text"
    packIconButton $w.toolbar.sep20  -separator
    packIconButton $w.toolbar.save   -image $icons(save) -command "saveFile $w $filename"
    packIconButton $w.toolbar.sep21  -separator

    set help_tips($w.toolbar.copy)   {Copy selected text to clipboard (Ctrl+C)}
    set help_tips($w.toolbar.find)   {Find string in window (Ctrl+F}
    set help_tips($w.toolbar.save)   {Save window contents to file}

    pack $w.toolbar  -anchor center -expand 0 -fill x -side top

    frame $w.main  -borderwidth 1 -relief sunken
    pack $w.main  -anchor center -expand 1 -fill both -ipadx 0 -ipady 0 -padx 0 -pady 0 -side top
    scrollbar $w.main.sb -command "$w.main.text yview"
    pack $w.main.sb -anchor center -expand 0 -fill y -ipadx 0 -ipady 0 -padx 0 -pady 0 -side right
    text $w.main.text -yscrollcommand "$w.main.sb set" -width 88 -height 30 -font $fonts(text)
    pack $w.main.text -anchor center -expand 1 -fill both -side left

    logTextWidget:configureTags $w.main.text
    bindCommandsToTextWidget $w.main.text

    # Read file
    loadFile $w $filename
}


#
# Create a context menu for a text widget
#
proc textwidget:contextMenu {txt wintype X Y} {
    global tmp config

    set tmp(wrap) [$txt cget -wrap]

    catch {destroy .popup}
    menu .popup -tearoff 0

    .popup add command -command editCopy -label {Copy} -accel {Ctrl+C} -underline 0
    .popup add separator
    .popup add command -command "editFind $txt" -label {Find...} -accel {Ctrl+F} -underline 0
    .popup add command -command "editFindNext $txt" -label {Find next} -accel {Ctrl+N,F3} -underline 5
    .popup add separator
    if {$wintype=="modulewindow"} {
        set w [winfo toplevel $txt]
        .popup add command -command "moduleWindow:openFilterDialog $w" -label {Filter window contents...} -accel {Ctrl+H} -underline 0
        .popup add separator

    }
    if {$wintype=="mainwindow"} {
        .popup add command -command "mainlogWindow:openFilterDialog" -label {Filter window contents...} -accel {Ctrl+H} -underline 0
        .popup add separator
    }
    .popup add checkbutton -command "textwidget:toggleWrap $txt" -variable tmp(wrap) -onvalue "char" -offvalue "none" -label {Wrap lines} -underline 0
    .popup add separator
    .popup add command -command "$txt tag add sel 1.0 end" -label {Select all} -accel {Ctrl+A} -underline 0

    tk_popup .popup $X $Y
}

proc textwidget:toggleWrap {txt} {
    global tmp config

    $txt config -wrap $tmp(wrap)

    # set default for further windows
    set config(editor-wrap) $tmp(wrap)
}
