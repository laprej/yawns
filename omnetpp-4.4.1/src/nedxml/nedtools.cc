//==========================================================================
// nedtools.cc -
//
//                     OMNeT++/OMNEST
//            Discrete System Simulation in C++
//
//==========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 2002-2008 Andras Varga
  Copyright (C) 2006-2008 OpenSim Ltd.

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
*--------------------------------------------------------------*/

#include "nedtools.h"
#include "nederror.h"
#include "neddtdvalidator.h"
#include "fileutil.h"

NAMESPACE_BEGIN


void NEDTools::repairNEDElementTree(NEDElement *tree)
{
    while (true)
    {
        // try DTD validation, and find first problem
        NEDErrorStore errors;
        NEDDTDValidator dtdvalidator(&errors);
        dtdvalidator.validate(tree);
        if (errors.empty())
            break; // we're done
        NEDElement *errnode = errors.errorContext(0);
        if (!errnode)
            break; // this shouldn't happen, but if it does we can't go on
        if (!errnode->getParent())
            break; // we can't help if root node is wrong

        // throw out problem node, and try again
        //printf("DBG: repairNEDElementTree: discarding <%s>\n", errnode->getTagName());
        errnode->getParent()->removeChild(errnode);
    }
}

void NEDTools::splitToFiles(FilesElement *tree)
{
    FilesElement *tmpTree = new FilesElement();
    for (NEDElement *child=tree->getFirstChild(); child; child = child->getNextSibling())
    {
        // ignore msg files
        if (child->getTagCode()!=NED_NED_FILE)
            continue;

        NedFileElement *fileNode = (NedFileElement *)child;

        // we'll generate new files into the directory of the original file
        std::string directory;
        std::string filename;
        splitFileName(fileNode->getFilename(), directory, filename);

        // go through NED components in the file, and create new NED files for them
        for (NEDElement *child=fileNode->getFirstChild(); child; )
        {
            int type = child->getTagCode();
            if (type!=NED_SIMPLE_MODULE && type!=NED_COMPOUND_MODULE &&
                type!=NED_CHANNEL && type!=NED_MODULE_INTERFACE &&
                type!=NED_CHANNEL_INTERFACE)
            {
                child = child->getNextSibling();
                continue;
            }

            // process NED component
            NEDElement *componentNode = child;
            const char *componentName = componentNode->getAttribute("name");

            // create new file for it
            NedFileElement *newFileNode = fileNode->dup();
            std::string newFileName = directory + "/" + componentName + ".ned";
            newFileNode->setFilename(newFileName.c_str());
            tmpTree->appendChild(newFileNode);

            // copy comments and imports from old file
            for (NEDElement *child2=fileNode->getFirstChild(); child2; child2 = child2->getNextSibling())
                if (child2->getTagCode()==NED_COMMENT || child2->getTagCode()==NED_IMPORT)
                    newFileNode->appendChild(child2->dupTree());

            // move NED component into new file
            child = child->getNextSibling(); // adjust iterator
            fileNode->removeChild(componentNode);
            newFileNode->appendChild(componentNode);
        }

        // rename original file
        fileNode->setFilename((std::string(fileNode->getFilename())+"-STRIPPED").c_str());
    }

    while (tmpTree->getFirstChild())
        tree->appendChild(tmpTree->removeChild(tmpTree->getFirstChild()));
}

NAMESPACE_END

