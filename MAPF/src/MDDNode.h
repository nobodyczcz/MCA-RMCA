//
// Created by Zhe Chen on 24/3/20.
//
#pragma once
#ifndef CBS_K_MDDNODE_H
#define CBS_K_MDDNODE_H
#endif //CBS_K_MDDNODE_H
#include <iostream>
#include <list>
#include <vector>



class MDDNode
{
public:
    MDDNode(int currloc, MDDNode* parent);
    int location;
    int row;
    int col;
    int level;
    int heading;
    int label;

    bool operator == (const MDDNode & node) const
    {
        return (this->location == node.location) && (this->level == node.level) && (this->heading == node.heading);
    }


    std::list<MDDNode*> children;
    std::list<MDDNode*> parents;
    MDDNode* parent;
};


typedef std::vector<std::list<MDDNode*>> MDDLevels;
