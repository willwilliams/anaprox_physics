#include "World.h"
#include <cmath>
#include <ctime>

World* World::theWorld = NULL;

World::World()
    : treeRoot(NULL)
{
    World::theWorld = this;
    std::srand(std::time(0));
}

World::~World()
{
    World::theWorld = NULL;
}

void World::checkCollisions()
{
    std::list<BVHLeafNode*>::iterator iter;
    if(treeRoot != NULL)
    {
        for(iter = planes.begin(); iter != planes.end(); iter++)
            treeRoot->getCollision(*iter);
        treeRoot->getCollision();
    }
}

BVHLeafNode* World::insert(Collision* toInsert)
{
    BVHLeafNode* newNode = new BVHLeafNode(toInsert, true, NULL);
    if(treeRoot != NULL)
    {
        treeRoot->insert(newNode);
        if(treeRoot->isLeaf)
            treeRoot = (BVHNode*)newNode->parentNode;
    }
    else
        treeRoot = newNode;
    return newNode;
}

void World::insert(BVHLeafNode* toInsert)
{
    if(treeRoot != NULL)
    {
        treeRoot->insert(toInsert);
        if(treeRoot->isLeaf)
            treeRoot = (BVHNode*)toInsert->parentNode;
    }
    else
        treeRoot = toInsert;
}

std::list<BVHLeafNode*>::iterator World::insertPlane(CollisionPlane *toInsert)
{
    BVHLeafNode* newNode = new BVHLeafNode(toInsert, true, NULL);
    planes.push_front(newNode);
    return planes.begin();
}

void World::removePlane(std::list<BVHLeafNode*>::iterator toRemove)
{
    delete *toRemove;
    planes.erase(toRemove);
}

void World::setRoot(BVHNode *newRoot)
{
    //delete treeRoot;
    treeRoot = newRoot;
}

void World::nullRoot()
{
    treeRoot = NULL;
}