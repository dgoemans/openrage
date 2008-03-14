#ifndef H_BULLETCORE_H
#define H_BULLETCORE_H

#include "btBulletDynamicsCommon.h"
#include "Ogre.h"



class coreBulletNode : public btDefaultMotionState
{
public:

    Ogre::SceneNode *node;
    float mass;
    Ogre::String name;
    btRigidBody *body;


public:

    // build a new object integrating Ogre3D Scene Node and Bullet collision shape
    // load the scene node, starting transform, and CoM transform (default values provided)
    // Call various functions to init the physics system transforms (inhereted code)
    coreBulletNode( Ogre::SceneNode *n, const btTransform startTrans = btTransform::getIdentity(),
        const btTransform centerOfMassOffset = btTransform::getIdentity() )
            : btDefaultMotionState( startTrans, centerOfMassOffset )
        { node = n; };

    // translate Bullet btTransform into Ogre3D Scene Node orientation and position
    void hammerNode();

    // switch the Ogre3D scene node this Bullet transform is associated with
    void setNode( Ogre::SceneNode *n ) { node = n; };

    // return the Ogre3D scene node this Bullet transform is associated with
    Ogre::SceneNode *getNode() { return node; };
    const Ogre::String getName() { return name; };
};



// Top level interface between Ogre and Bullet
// Starts, stops, pauses Bullet world, init objects, processes results
class coreBullet
{
protected:

    // Array of all Bullet/Ogre crossbread nodes in use DYNAMIC OBJECTS
    std::vector<coreBulletNode *> bullet_nodes;

    // Array of all STATIC Bullet collision objects
    std::vector<coreBulletNode *> static_nodes;

    // Map of defined Bullet shape instances to be re-used by coreBulletNode's
    // The Bullet shapes are associated with string names, like, "Hex_Block_01", etc.
    // These names directly reflec the xxxx.mesh name found in .scene files
    // So, .mesh's are directly linked to the Bullet shapes that represent them in the physics sim.
    std::map<Ogre::String, btCollisionShape *> shapes;

    // Ogre3D specifics
    Ogre::SceneManager *scene;

    // specifics of Bullet physics solving system
	bool idle;
    float collision_margin;
    float time_scale;


public:

    // Bullet physics world, including collisions
    btDynamicsWorld *bullet_world;
    btDefaultCollisionConfiguration* collision_config;

    // start a new Bullet world
    coreBullet( Ogre::SceneManager *s );

    // reset all Bullet objects to start transforms, v = 0, ang_v = 0;
    void resetWorld();

    // hammer Bullet orientation/position transform results into Ogre3D Scene Nodes
    int processNodeList( Ogre::Real time );

    bool removeNode( Ogre::SceneNode *n );
    btRigidBody *addNode( btCollisionShape *shape, Ogre::Real mass, Ogre::SceneNode *n );
    btCollisionShape *addSphere( Ogre::String shapename, Ogre::Real radius );
    btCollisionShape *addBox( Ogre::String shapename, Ogre::Vector3 size );
    btCollisionShape *addStaticMesh( Ogre::String shapename, Ogre::Mesh *mesh );

    // returns a pointer to a collision shape if an instance of it already exists, 0 if not
    btCollisionShape *getShape( Ogre::String meshname );
    unsigned long int getShapeMapSize() { return shapes.size(); };

    // start/stop/check physics simulation run flag
    void toggleIdle() { if( idle ) idle = false; else idle = true; };
    bool isIdle() { return idle; };

    // set/get the time step scale factor
    void setTimeStep( Ogre::Real step ) { time_scale = step; };
    Ogre::Real getTimeStep() { return time_scale; };

    // dump debug data to file
    void dump( const char *filename );
};

#endif
