#pragma once

#include "BVHNode.h"
#include "BVHLeafNode.h"
#include "CollisionPlane.h"
#include "Collision.h"
#include <list>

class World
{
public:
    World();
    ~World();

    static World* getWorld() {return World::theWorld;};

    void checkCollisions();

    BVHLeafNode* insert(Collision* toInsert);
    void insert(BVHLeafNode* toInsert);
    std::list<BVHLeafNode*>::iterator insertPlane(CollisionPlane* toInsert);
    void removePlane(std::list<BVHLeafNode*>::iterator toRemove);
    void setRoot(BVHNode* newRoot);
    void nullRoot();

private:
    static World* theWorld;
    BVHNode* treeRoot;
    std::list<BVHLeafNode*> planes;
};