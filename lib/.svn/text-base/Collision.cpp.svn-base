#include "Collision.h"
#include <string>

Collision::Collision(CollisionType type, RigidBody* obj)
     : object(obj), mType(type)
{}

Collision::~Collision()
{}

std::list<Contact*> Collision::getContacts(Collision *check)
{
    switch(check->mType)
    {
    case SPHERE:
        return getContacts((CollisionSphere*)check);
    case BOX:
        return getContacts((CollisionBox*)check);
    case PLANE:
        return getContacts((CollisionPlane*)check);
    };
}

void Collision::resolveContacts(std::list<Contact*> conts)
{
    precision restitution = 0.4;
    precision coeffFriction = 0.005;

    std::list<Contact*>::iterator iter;
    for(iter = conts.begin(); iter != conts.end(); iter++)
    {
        precision velPerImpulse = 0;
        Vector velPerImpulseTemp;
        Vector relativePosition2;
        Vector contactTangent2;

        Vector relativePosition1 = (*iter)->position - object->getPosition();

        //Calculate closing velocity
        Vector contactVel = object->getAngVel().vectorProduct(relativePosition1);
        contactVel += object->getVelocity();

        if((*iter)->object2->mType != Collision::PLANE)
        {
            
            //Calculate closing velocity
            contactVel = (*iter)->object2->object->getAngVel().vectorProduct(relativePosition2) * -1;
            contactVel -= (*iter)->object2->object->getVelocity();

            //Early out if seperating
            if(contactVel.magnitude() <= 0)
                return;

            if((*iter)->object2->object->getInvMass() == 0 && object->getInvMass() == 0 && (*iter)->object2->object->getInvTensor().isZero() && object->getInvTensor().isZero())
                return;

            //Calculate a vector for the change in velocity per unit of impulse in the direction of the constact normal
            relativePosition2 = (*iter)->position - (*iter)->object2->object->getPosition();
            velPerImpulseTemp = relativePosition2.vectorProduct((*iter)->contactNormal);
            contactTangent2 = velPerImpulseTemp;
            contactTangent2.normalise();
            velPerImpulseTemp = (*iter)->object2->object->getInvTensor() * velPerImpulseTemp;
            velPerImpulseTemp = velPerImpulseTemp.vectorProduct(relativePosition2);

            //Angular
            velPerImpulse += velPerImpulseTemp.scalarProduct((*iter)->contactNormal);

            //Linear
            velPerImpulse += (*iter)->object2->object->getInvMass();
        }
        else if(contactVel.magnitude() <= 0 || (object->getInvMass() == 0 &&object->getInvTensor().isZero()))
            return;

        std::cout << (*iter)->contactNormal;

        //Calculate a vector for the change in velocity per unit of impulse in the direction of the constact normal
        velPerImpulseTemp = relativePosition1.vectorProduct((*iter)->contactNormal);
        Vector contactTangent1 = velPerImpulseTemp;
        contactTangent1.normalise();
        velPerImpulseTemp = object->getInvTensor() * velPerImpulseTemp;
        velPerImpulseTemp = velPerImpulseTemp.vectorProduct(relativePosition1);

        //Angular part
        velPerImpulse += velPerImpulseTemp.scalarProduct((*iter)->contactNormal);

        //Linear part
        velPerImpulse += object->getInvMass();

        //Calculate impulse to be added
        precision normalVel = contactVel.scalarProduct((*iter)->contactNormal);
        precision velChange = -(normalVel*(1+restitution));
        Vector impulse = (*iter)->contactNormal * (velChange / velPerImpulse);

        //Add impulse to objects (opposite directions)
        object->addImpulse(impulse * object->getInvMass());
        Vector impulseTorque = impulse.vectorProduct(relativePosition1);
        object->addAngVel(object->getInvTensor() * impulseTorque);

        if((*iter)->object2->mType != Collision::PLANE)
        {
            impulse = impulse * -1;

            (*iter)->object2->object->addImpulse(impulse * (*iter)->object2->object->getInvMass());
            Vector impulseTorque = impulse.vectorProduct(relativePosition2);
            (*iter)->object2->object->addAngVel((*iter)->object2->object->getInvTensor() * impulseTorque);
        }

        //Penetration correction
        if(iter == conts.begin())
            object->setPosition(((*iter)->contactNormal * -(*iter)->penetrationDepth) + object->getPosition());

        //Calculate frictional impulse
        impulse = ((contactVel-(*iter)->contactNormal * normalVel))*(coeffFriction);
        impulse = impulse * velPerImpulse;

        //Add impulse to objects (opposite directions)
        object->addImpulse(impulse * object->getInvMass());
        impulseTorque = impulse.vectorProduct(relativePosition1);
        object->addAngVel(object->getInvTensor() * impulseTorque);

        if((*iter)->object2->mType != Collision::PLANE)
        {
            (*iter)->object2->object->addImpulse(impulse * (*iter)->object2->object->getInvMass());
            Vector impulseTorque = impulse.vectorProduct(relativePosition2);
            (*iter)->object2->object->addAngVel((*iter)->object2->object->getInvTensor() * impulseTorque);
        }
    }
}
