#=================================================================
#  FIELDSPAGE.TCL - part of
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
# Creates a Fields page for a tabbed inspector window.
#
proc inspector:createFields2Page {w} {
    global treeroots
    global B2 B3
    set nb $w.nb
    notebook:addPage $nb fields2 {Fields}

    # create treeview with scrollbars
    scrollbar $nb.fields2.vsb -command "$nb.fields2.tree yview"
    scrollbar $nb.fields2.hsb -command "$nb.fields2.tree xview" -orient horiz
    canvas $nb.fields2.tree -bg white -relief sunken -bd 2
    $nb.fields2.tree config -yscrollcommand "$nb.fields2.vsb set" -xscrollcommand "$nb.fields2.hsb set"

    grid $nb.fields2.tree $nb.fields2.vsb -sticky news
    grid $nb.fields2.hsb  x               -sticky news
    grid rowconfig $nb.fields2 0 -weight 1
    grid columnconfig $nb.fields2 0 -weight 1

    set tree $nb.fields2.tree
    if {![regexp {\.(ptr.*)-([0-9]+)} $w match object type]} {
        error "window name $w doesn't look like an inspector window"
    }
    set treeroots($tree) $object

    Tree:init $tree fields2Page:getNodeInfo

    bind $tree <Double-1> {
        focus %W
        set key [Tree:nodeat %W %x %y]
        fields2Page:getNodeInfo:inspect %W $key
    }

    bind $tree <Return> {
        set key [Tree:getselection %W]
        fields2Page:getNodeInfo:inspect %W $key
    }

    bind $tree <Button-$B3> {
        focus %W
        set key [Tree:nodeat %W %x %y]
        if {$key!=""} {
            Tree:setselection %W $key
            fields2Page:getNodeInfo:popup %W $key %X %Y
        }
    }

    Tree:open $tree "0-obj-$object"
}

proc fields2Page:refresh {w} {
    set tree $w.nb.fields2.tree
    if ![winfo exist $tree] {return}

    Tree:build $tree
}

#
# Content provider for the inspector treeview.
#
# Possible keys:
#   <depth>-obj-<ptr>
#   <depth>-struct-<ptr>-<descptr>
#   <depth>-group-<ptr>-<descptr>-<groupid>
#   <depth>-field-<ptr>-<descptr>-<fieldid>
#   <depth>-findex-<ptr>-<descptr>-<fieldid>-<index>
#
# groupid is id of a field (ie a fieldid) which has the right group.
# We use that because the group name itself may contain space, hyphen
# and other unwanted characters.
#
# depth is the depth in the displayed tree (0 for root, 1 for its children, etc).
# It is needed to make multiple appearances of the same object unique. Without it,
# if a node appears under itself in the tree, that causes infinite recursion.
# (crash scenario without depth: open the object's "owner" field, then the object
# itself within its owner --> bang!).
#
proc fields2Page:getNodeInfo {w op {key ""}} {
    global icons treeroots

    set keyargs [split $key "-"]
    set depth [lindex $keyargs 0]
    set keytype [lindex $keyargs 1]
    set obj [lindex $keyargs 2]
    set sd [lindex $keyargs 3]

    switch $op {

        text {
            if [opp_isnull $obj] {return "<object is NULL>"}
            if {$sd!="" && [opp_isnull $sd]} {return "<no descriptor for object>"}

            switch $keytype {
                obj {
                    set name [opp_getobjectfullname $obj]
                    set classname [opp_getobjectshorttypename $obj]
                    return "$name ($classname)"
                }
                struct {
                    set name [opp_classdescriptor name $obj $sd]
                    return $name
                }

                group {
                    set groupid [lindex $keyargs 4]
                    set groupname [opp_classdescriptor $obj $sd fieldproperty $groupid "group"]
                    return "\b$groupname\b"
                }

                field -
                findex {
                    set fieldid [lindex $keyargs 4]
                    set index [lindex $keyargs 5]
                    return [fields2Page:getNodeInfo:getFieldText $obj $sd $fieldid $index]
                }

                default {
                    error "bad keytype '$keytype'"
                }
            }
        }

        needcheckbox {
            return 0
        }

        options {
            return ""
        }

        icon {
            switch $keytype {
                obj {
                    return [inspector:getIconForObject $obj]
                }
                struct -
                group {
                    return ""
                }
                field -
                findex {
                    set fieldptr [fields2Page:getNodeInfo:resolveObject $keyargs]
                    if [opp_isnull $fieldptr] {return ""}
                    return [inspector:getIconForObject $fieldptr]
                }
            }
        }

        tooltip {
            switch $keytype {
                obj -
                struct -
                group {
                    return ""
                }
                field -
                findex {
                    ## These tooltips are actually not very useful
                    # set fieldid [lindex $keyargs 4]
                    # set tooltip [opp_classdescriptor $obj $sd fieldproperty $fieldid "hint"]
                    # return $tooltip
                    return ""
                }
            }
        }

        haschildren {
            switch $keytype {
                group -
                root {
                    return 1  ;# sure has children
                }
                obj -
                struct -
                field -
                findex {
                    # XXX this can be improved performance-wise (this actually collects all children!)
                    set children [fields2Page:getNodeInfo $w children $key]
                    return [expr [llength $children]!=0]
                }
            }
        }

        children {
            incr depth
            switch $keytype {
                obj -
                struct {
                    if {$obj==[opp_null]} {return ""}
                    if {$keytype=="obj"} {set sd [opp_getclassdescriptorfor $obj]}
                    if {$sd==[opp_null]} {return ""}

                    set children1 [fields2Page:getNodeInfo:getFieldsInGroup $depth $obj $sd ""]
                    set children2 [fields2Page:getNodeInfo:getGroupKeys $depth $obj $sd]
                    return [concat $children1 $children2]
                }

                group {
                    # return fields in the given group
                    set groupname [lindex $keyargs 4]
                    return [fields2Page:getNodeInfo:getFieldsInGroup $depth $obj $sd $groupname]
                }

                field {
                    set fieldid [lindex $keyargs 4]
                    set isarray [opp_classdescriptor $obj $sd fieldisarray $fieldid]
                    if {$isarray} {
                        # expand array: enumerate all indices
                        return [fields2Page:getNodeInfo:getElementsInArray $depth $obj $sd $fieldid]
                    }
                    set iscompound [opp_classdescriptor $obj $sd fieldiscompound $fieldid]
                    if {$iscompound} {
                        # return children on this class/struct
                        set ispoly [opp_classdescriptor $obj $sd fieldiscobject $fieldid]
                        set fieldptr [opp_classdescriptor $obj $sd fieldstructpointer $fieldid]
                        if [opp_isnull $fieldptr] {return ""}
                        if {$ispoly} {
                            return [fields2Page:getNodeInfo $w children "$depth-obj-$fieldptr"]
                        } else {
                            set fielddesc [opp_classdescriptor $obj $sd fieldstructdesc $fieldid]
                            if {$fielddesc==""} {return ""}  ;# nothing known about it
                            set tmpkey "$depth-struct-$fieldptr-$fielddesc"
                            return [fields2Page:getNodeInfo $w children $tmpkey]
                        }
                    }
                    return ""
                }

                findex {
                    set fieldid [lindex $keyargs 4]
                    set index [lindex $keyargs 5]
                    set iscompound [opp_classdescriptor $obj $sd fieldiscompound $fieldid]
                    if {$iscompound} {
                        # return children on this class/struct
                        set ispoly [opp_classdescriptor $obj $sd fieldiscobject $fieldid]
                        set fieldptr [opp_classdescriptor $obj $sd fieldstructpointer $fieldid $index]
                        if [opp_isnull $fieldptr] {return ""}
                        if {$ispoly} {
                            return [fields2Page:getNodeInfo $w children "$depth-obj-$fieldptr"]
                        } else {
                            set fielddesc [opp_classdescriptor $obj $sd fieldstructdesc $fieldid]
                            if {$fielddesc==""} {return ""}  ;# nothing known about it
                            set tmpkey "$depth-struct-$fieldptr-$fielddesc"
                            return [fields2Page:getNodeInfo $w children $tmpkey]
                        }
                    }
                    return ""
                }

                root {
                    return "0-obj-$obj"
                }

                default {
                    error "bad keytype '$keytype'"
                }
            }
        }

        root {
            # add an extra level to make the root object appear in the tree as well
            #return "0-obj-$treeroots($w)"
            return "0-root-$treeroots($w)"
        }
    }
}

#
# Helper proc for fields2Page:getNodeInfo.
# Collects field groups, and converts them to keys.
#
proc fields2Page:getNodeInfo:getGroupKeys {depth obj sd} {
    # collect list of groups
    set numfields [opp_classdescriptor $obj $sd fieldcount]
    for {set i 0} {$i<$numfields} {incr i} {
        set fieldgroup [opp_classdescriptor $obj $sd fieldproperty $i "group"]
        set groups($fieldgroup) $i
    }

    # convert them to keys
    set children {}
    foreach groupname [lsort [array names groups]] {
        if {$groupname!=""} {
            lappend children "$depth-group-$obj-$sd-$groups($groupname)"
        }
    }
    return $children
}


#
# Helper proc for fields2Page:getNodeInfo.
# Return fields in the given group; groupname may be "" (meaning no group).
#
proc fields2Page:getNodeInfo:getFieldsInGroup {depth obj sd groupid} {
    set children {}
    if {$groupid!=""} {
        set groupname [opp_classdescriptor $obj $sd fieldproperty $groupid "group"]
    } else {
        set groupname ""
    }
    set numfields [opp_classdescriptor $obj $sd fieldcount]
    for {set i 0} {$i<$numfields} {incr i} {
        if {$groupname==[opp_classdescriptor $obj $sd fieldproperty $i "group"]} {
            lappend children "$depth-field-$obj-$sd-$i"
        }
    }
    return $children
}

#
# Helper proc for fields2Page:getNodeInfo.
# Expands array by enumerating all indices, and returns the list of corresponding keys.
#
proc fields2Page:getNodeInfo:getElementsInArray {depth obj sd fieldid} {
    set children {}
    set n [opp_classdescriptor $obj $sd fieldarraysize $fieldid]
    for {set i 0} {$i<$n} {incr i} {
        lappend children "$depth-findex-$obj-$sd-$fieldid-$i"
    }
    return $children
}

#
# Helper proc for fields2Page:getNodeInfo. Produces text for a field.
#
proc fields2Page:getNodeInfo:getFieldText {obj sd fieldid index} {

    set typename [opp_classdescriptor $obj $sd fieldtypename $fieldid]
    set isarray [opp_classdescriptor $obj $sd fieldisarray $fieldid]
    set iscompound [opp_classdescriptor $obj $sd fieldiscompound $fieldid]
    set ispoly [opp_classdescriptor $obj $sd fieldiscobject $fieldid]
    set isobject [opp_classdescriptor $obj $sd fieldiscownedobject $fieldid]
    set iseditable [opp_classdescriptor $obj $sd fieldiseditable $fieldid]

    # field name can be overridden with @label property
    set name [opp_classdescriptor $obj $sd fieldproperty $fieldid "label"]
    if {$name==""} {set name [opp_classdescriptor $obj $sd fieldname $fieldid]}

    # if it's an unexpanded array, return "name[size]" immediately
    if {$isarray && $index==""} {
        set size [opp_classdescriptor $obj $sd fieldarraysize $fieldid]
        return "$name\[$size\] ($typename)"
    }

    # when showing array elements, omit name and just show "[index]" instead
    if {$index!=""} {
        set name "\[$index\]"
    }

    # we'll want to print the field type, except for expanded array elements
    # (no need to repeat it, as it's printed in the "name[size]" node already)
    if {$index==""} {
        set typenametext " ($typename)"
    } else {
        set typenametext ""
    }

    # "editable" flag
    if {$iseditable} {
        set typenametext " \[...\] $typenametext"
    }

    if {$iscompound} {
        # if it's an object, try to say something about it...
        if {$ispoly} {
            set fieldobj [opp_classdescriptor $obj $sd fieldstructpointer $fieldid $index]
            if [opp_isnull $fieldobj] {return "$name = \bNULL\b$typenametext"}
            if {!$isobject || [opp_getobjectowner $fieldobj]==$obj} {
                set fieldobjname [opp_getobjectfullname $fieldobj]
            } else {
                set fieldobjname [opp_getobjectfullpath $fieldobj]
            }
            set fieldobjname [opp_getobjectfullname $fieldobj]
            set fieldobjclassname [opp_getobjectshorttypename $fieldobj]
            set fieldobjinfo [opp_getobjectinfostring $fieldobj]
            if {$fieldobjinfo!=""} {
                set fieldobjinfotext ": $fieldobjinfo"
            } else {
                set fieldobjinfotext ""
            }
            return "$name = \b($fieldobjclassname) $fieldobjname$fieldobjinfotext\b$typenametext"
        } else {
            # a value can be generated via operator<<
            if [catch {set value [opp_classdescriptor $obj $sd fieldvalue $fieldid $index]} err] {set value "<!> Error: $err"}
            if {$value==""} {
                return "$name$typenametext"
            } else {
                return "$name = \b$value\b$typenametext"
            }
        }
    } else {
        # plain field, return "name = value" text
        if [catch {set value [opp_classdescriptor $obj $sd fieldvalue $fieldid $index]} err] {set value "<!> Error: $err"}
        set enumname [opp_classdescriptor $obj $sd fieldproperty $fieldid "enum"]
        if {$enumname!=""} {
            append typename " - enum $enumname"
            set symbolicname [opp_getnameforenum $enumname $value]
            set value "$symbolicname ($value)"
        }
        #if {$typename=="string"} {set value "\"$value\""}
        if {$typename=="string"} {set value "'$value'"}
        if {$value==""} {
            return "$name$typenametext"
        } else {
            return "$name = \b$value\b$typenametext"
        }
    }
}

#
# If the given key (in split form) identifies an object (cClassDescriptor
# isCObject), returns its pointer. Otherwise returns [opp_null].
#
#
proc fields2Page:getNodeInfo:resolveObject {keyargs} {
    set depth [lindex $keyargs 0]
    set keytype [lindex $keyargs 1]

    if {$keytype=="field" || $keytype=="findex"} {
        set obj [lindex $keyargs 2]
        set sd [lindex $keyargs 3]
        set fieldid [lindex $keyargs 4]
        set index [lindex $keyargs 5]
        set isobject [opp_classdescriptor $obj $sd fieldiscownedobject $fieldid]
        set isarray [opp_classdescriptor $obj $sd fieldisarray $fieldid]

        if {$isobject && (!$isarray || $index!="")} {
            return [opp_classdescriptor $obj $sd fieldstructpointer $fieldid $index]
        }
    }
    return [opp_null]
}


#
# Invoked on hitting Enter or double-clicking in the tree, it opens
# an inspector for the given object. Or starts editing. Or at least
# opens/closes that tree branch.
#
proc fields2Page:getNodeInfo:inspect {w key} {
    set keyargs [split $key "-"]
    set ptr [fields2Page:getNodeInfo:resolveObject $keyargs]
    if [opp_isnotnull $ptr] {
        opp_inspect $ptr "(default)"
    } elseif [fields2Page:getNodeInfo:isEditable $w $key] {
        fields2Page:getNodeInfo:edit $w $key
    } else {
        Tree:toggle $w $key
    }
}

#
# Returns true if a tree node is editable (it's a field and its descriptor says editable)
#
proc fields2Page:getNodeInfo:isEditable {w key} {
    set keyargs [split $key "-"]
    set keytype [lindex $keyargs 1]
    if {$keytype=="field" || $keytype=="findex"} {
        set obj [lindex $keyargs 2]
        set sd [lindex $keyargs 3]
        set fieldid [lindex $keyargs 4]
        return [opp_classdescriptor $obj $sd fieldiseditable $fieldid]
    }
    return 0
}

#
# Initiates in-place editing of a tree field
#
proc fields2Page:getNodeInfo:edit {w key} {
    set id [lindex [$w find withtag "text-$key"] 1]
    if {$id!=""} {
        editCanvasLabel $w $id [list fields2Page:getNodeInfo:setValue $w $key]
    }
}

#
# Sets the value of a tree field after in-place editing
#
proc fields2Page:getNodeInfo:setValue {w key value} {
    set keyargs [split $key "-"]
    set keytype [lindex $keyargs 1]
    if {$keytype=="field" || $keytype=="findex"} {
        set obj [lindex $keyargs 2]
        set sd [lindex $keyargs 3]
        set fieldid [lindex $keyargs 4]
        set index [lindex $keyargs 5]
        #regexp {^"(.*)"$} $value match value  ;# strip quotes
        if [catch {opp_classdescriptor $obj $sd fieldsetvalue $fieldid $index $value} e] {
            Tree:build $w
            tk_messageBox -parent [winfo toplevel $w] -title "Tkenv" -icon warning -type ok -message "Cannot set field value -- syntax error?"
            return
        }
    }
    Tree:build $w
}

proc fields2Page:getNodeInfo:popup {w key x y} {
    catch {destroy .popup}
    menu .popup -tearoff 0

    .popup add command -label "Copy" -command [list fields2Page:getNodeInfo:copy $w $key]

    .popup add separator

    set keyargs [split $key "-"]
    set ptr [fields2Page:getNodeInfo:resolveObject $keyargs]
    if [opp_isnotnull $ptr] {
        foreach i [opp_supported_insp_types $ptr] {
           .popup add command -label "Inspect $i" -command "opp_inspect $ptr \"$i\""
        }
    } else {
        if [fields2Page:getNodeInfo:isEditable $w $key] {
            .popup add command -label "Edit..." -command [list fields2Page:getNodeInfo:edit $w $key]
        } else {
            .popup add command -label "Edit..." -state disabled
        }
    }

    tk_popup .popup $x $y
}

proc fields2Page:getNodeInfo:copy {w key} {
    set txt [fields2Page:getNodeInfo $w text $key]
    regsub -all "\b" $txt "" txt

    # note: next two lines are from tk_textCopy Tk proc.
    clipboard clear -displayof $w
    clipboard append -displayof $w $txt
}


##
## This alternative version of the Fields table uses a BLT treeview widget.
##
#proc inspector:createFields2PageX {w} {
#    global treeroots
#    set nb $w.nb
#    notebook:addPage $nb fields2 {Fields}
#
#    multicolumnlistbox $nb.fields2.list {
#       {value  Value   200}
#       {type   Type    80}
#       {on     Declared-on}
#    } -width 400 -yscrollcommand "$nb.fields2.vsb set" -xscrollcommand "$nb.fields2.hsb set"
#    scrollbar $nb.fields2.hsb  -command "$nb.fields2.list xview" -orient horiz
#    scrollbar $nb.fields2.vsb  -command "$nb.fields2.list yview"
#    grid $nb.fields2.list $nb.fields2.vsb -sticky news
#    grid $nb.fields2.hsb  x           -sticky news
#    grid rowconfig    $nb.fields2 0 -weight 1 -minsize 0
#    grid columnconfig $nb.fields2 0 -weight 1 -minsize 0
#
#    set tree $nb.fields2.list
#
#    $tree config -flat no -hideroot yes
#    $tree column configure treeView -text "Name" -hide no -width 160 -state disabled
#
#    #set root [$tree insert end ROOT -data {value 42 type struct}]
#    #set one  [$tree insert end {ROOT ONE} -data {value 42 type short}]
#    #set two  [$tree insert end {ROOT TWO} -data {value 42 type double}]
#
#    if {![regexp {\.(ptr.*)-([0-9]+)} $w match object type]} {
#        error "window name $w doesn't look like an inspector window"
#    }
#
#    fillTreeView $tree $object   ;# should be called from C++ on updates
#}
#
#proc fillTreeView {tree obj} {
#    set desc [opp_getclassdescriptor $obj]
#    set key "fld-$obj-$desc---"
#
#    _doFillTreeView $tree {} $key
#}
#
#proc _doFillTreeView {tree path key} {
#    global icons
#
#    set data [_getFieldDataFor $key]
#    set name [lindex $data 1]
#    set newpath [concat $path [list $name]]
#    #set icon $icons(1pixtransp)
#    #set icon $icons(cogwheel_vs)
#    set icon $icons(node_xs)
#    $tree insert end $newpath -data $data -icons [list $icon $icon] -activeicons [list $icon $icon]
#
#    foreach child [fields2Page:getNodeInfo $tree children $key] {
#        _doFillTreeView $tree $newpath $child
#    }
#}
#
#proc _getFieldDataFor {key} {
#    # key: objectptr-descriptorptr-fieldid-index
#    set obj ""
#    set sd ""
#    set fieldid ""
#    set index ""
#    regexp {^fld-(ptr.*)-(ptr.*)-([^-]*)-([^-]*)$} $key dummy obj sd fieldid index
#    #puts "DBG --> $obj -- $sd -- field=$fieldid -- index=$index"
#
#    if {$obj==[opp_null]} {return "<object is NULL>"}
#    if {$sd==[opp_null]} {return "<no descriptor for object>"}
#
#    set declname [opp_classdescriptor $obj $sd name]
#
#    if {$fieldid==""} {
#        return [list treeView $declname value "" type "" on ""]
#    }
#
#    set typename [opp_classdescriptor $obj $sd fieldtypename $fieldid]
#    set isarray [opp_classdescriptor $obj $sd fieldisarray $fieldid]
#    set name [opp_classdescriptor $obj $sd fieldname $fieldid]
#    if {$index!=""} {
#        append name "\[$index\]"
#    } elseif {$isarray} {
#        set size [opp_classdescriptor $obj $sd fieldarraysize $fieldid]
#        append name "\[$size\]"
#    }
#    set value [opp_classdescriptor $obj $sd fieldvalue $fieldid $index]
#    regsub -all "\n" $value "; " value
#    regsub -all "   +" $value "  " value
#    if {$typename=="string"} {set value "\"$value\""}
#
#    return [list treeView $name value $value type $typename on $declname]
#}

