#include "bullet_core.h"
#include "Ogre_getmeshdata.h"
#include <iostream>
#include <fstream>

using namespace Ogre;
using namespace std;



// dump contents of coreBullet to debug file
void coreBullet::dump( const char *filename )
{
    ofstream elog( filename, ios::out );

    elog << "STATE OF BULLET PHYSICS AND COLLISIONS WORLD" << "\n\n";
    elog << "Number of collision objects: " << bullet_world->getNumCollisionObjects() << "\n\n";

    elog << endl << "SHAPE INSTANCES: " << shapes.size() << endl;
    /*map<String, btCollisionShape *>::iterator it;
    for( it = shapes.begin(); it != shapes.end(); it++ ) {
        elog << (*it)->getName() << endl;
    }*/

    elog << endl << "BULLET DYNAMIC NODES: " << bullet_nodes.size() << endl;
    vector<coreBulletNode *>::iterator it2;
    for( it2 = bullet_nodes.begin(); it2 != bullet_nodes.end(); it2++ ) {
        elog << "\n" << (*it2)->getName() << ", " << "(real name) " << (*it2)->node->getName() << ", ";
        if( (*it2)->node->isInSceneGraph() ) elog << "IN SCENE GRAPH\n";
        else elog << "NOT IN SCENE GRAPH (not attached to root node!\n";
        elog << "parent = " << (*it2)->node->getParent()->getName() << "\n";

        SceneNode::ObjectIterator it = (*it2)->node->getAttachedObjectIterator();
        while( it.hasMoreElements() ) {

            MovableObject *gobj = it.getNext();

            elog << "\t [attached] TYPE: " << gobj->getMovableType() << ", ";
            if( gobj->getMovableType() == "Entity" ) elog << "MESH: " <<
                ((Entity *)gobj)->getMesh()->getName() << "\n";

            btVector3 origin = (*it2)->m_graphicsWorldTrans.getOrigin();
            elog << "btTransform origin: ( " << origin.getX() << ", " <<
                origin.getY() << ", " << origin.getZ() << " )\n";

            Vector3 nodepos = (*it2)->node->getWorldPosition();
            elog << "Ogre node origin (relative to world): ( " << nodepos[ 0 ] << ", " <<
                nodepos[ 1 ] << ", " << nodepos[ 2 ] << " )\n";

            nodepos = (*it2)->node->getPosition();
            elog << "Ogre node origin (relative to parent): ( " << nodepos[ 0 ] << ", " <<
                nodepos[ 1 ] << ", " << nodepos[ 2 ] << " )\n";
        }
    }

    elog << endl << "BULLET STATIC NODES: " << static_nodes.size() << endl;
    for( it2 = static_nodes.begin(); it2 != static_nodes.end(); it2++ ) {
        elog << (*it2)->getName() << endl;
    }

    elog << "\n\n" << "_LIST OF ALL COLLISION OBJECTS IN WORLD_" << endl;
    elog << "-----------------------------------------" << endl;
    for( int i = 0; i < bullet_world->getNumCollisionObjects(); i++ ) {
        btCollisionObject *obj = bullet_world->getCollisionObjectArray()[ i ];

        if( obj->isStaticObject() ) elog << " STATIC ";
        if( obj->isKinematicObject() ) elog << " KINEMATIC ";
        if( obj->hasContactResponse() ) elog << " RESPONDS TO CONTACTS ";
        elog << " flags = " << obj->getCollisionFlags();
        elog << " state = " << obj->getActivationState();
        elog << " active? "; if( obj->isActive() ) elog << "YES "; else elog << "NO ";
        elog << " restitution = " << obj->getRestitution();
        elog << " friction = " << obj->getFriction();
        elog << " island tag = " << obj->getIslandTag();
        elog << " hit friction = " << obj->getHitFraction();

        btCollisionShape *shape = obj->getCollisionShape();
        elog << endl << "\tSHAPE: " << shape->getName();

        const btTransform &trans = obj->getWorldTransform();
        elog << ", ORIGIN: ( " << trans.getOrigin().getX() << ", " <<
            trans.getOrigin().getY() << ", " << trans.getOrigin().getZ() << " )\n";

    }

    elog.close();
}



// Transfer all active Bullet transforms into their respective Ogre3D SceneNode's
int coreBullet::processNodeList( Real time )
{
    // If a Bullet world exists, continue.
	if( bullet_world ) {

		int processed_nodes = 0;

        // get total number of collision objects in the Bullet world
		int num_objects = bullet_world->getNumCollisionObjects();

		// USE VECTOR LIST of bullet_object's AND ITERATOR TO STEP THRU ALL
        vector<coreBulletNode *>::iterator it;
        for( it = bullet_nodes.begin(); it != bullet_nodes.end(); it++ ) {
            (*it)->hammerNode();
            processed_nodes++;
        }

        // bounding boxes for collision detection
        bullet_world->updateAabbs();

        // step the Bullet simulation world forward in time
		bullet_world->stepSimulation( time * time_scale, 1 );

        // return the number of Ogre3D nodes processed - number of Bullet collision objects
        return num_objects - processed_nodes;
	}

    // not sure what a good value to return here is.... -1 is a bad result tho...
	return -1;
}



// Initialize a Bullet physics and collision world with default parameters
coreBullet::coreBullet( SceneManager *s )
{
    scene = s;

    // collisions...
    collision_config = new btDefaultCollisionConfiguration();
    btCollisionDispatcher *dispatcher = new	btCollisionDispatcher( collision_config );
    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    const int maxProxies = 32766; // ????
    btBroadphaseInterface *broadphase = new btAxisSweep3( worldAabbMin, worldAabbMax, maxProxies );

    //default constraint solver
	btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;

    // create physics world
    bullet_world = new btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collision_config );
    bullet_world->setGravity( btVector3( 0, 0, -9.81f ) );

    // default for new objects
    collision_margin = 0.05f;

    // default time scale to apply to physics simulation: 1.0f = "real time"
    time_scale = 0.1f;

    idle = true;
}



// reset (initialize) all physics objects to start transforms
void coreBullet::resetWorld()
{
	if( bullet_world ) bullet_world->stepSimulation( 1.0f / 60.0f, 0 );

	int numObjects = bullet_world->getNumCollisionObjects();

    // loop through all objects in the Bullet world
	for( int i = 0; i < numObjects; i++ ) {
		btCollisionObject *colObj = bullet_world->getCollisionObjectArray()[ i ];
		btRigidBody *body = btRigidBody::upcast( colObj );

        // if the btCollisionBody exists and it has a motion state...
        // get it's btTransform, set it to active, remove cached contact points
		if( body && body->getMotionState() ) {
			coreBulletNode *m_bulletnode = (coreBulletNode*) body->getMotionState();
			m_bulletnode->m_graphicsWorldTrans = m_bulletnode->m_startWorldTrans;

			colObj->setWorldTransform( m_bulletnode->m_graphicsWorldTrans );
			colObj->setInterpolationWorldTransform( m_bulletnode->m_startWorldTrans );
			colObj->activate();

			//removed cached contact points
			//bullet_world->getBroadphase()->destroyProxy( colObj->getBroadphaseHandle() );

			btRigidBody *body = btRigidBody::upcast( colObj );

            // if it's not a static object set it's velocities to (0,0,0)
			if( body && !body->isStaticObject() ) {
				btRigidBody::upcast(colObj)->setLinearVelocity( btVector3( 0, 0, 0 ) );
				btRigidBody::upcast(colObj)->setAngularVelocity( btVector3( 0, 0, 0 ) );
			}
		}
	}
}



// Remove a bullet node from the list and delete its physics/collision data
bool coreBullet::removeNode( SceneNode *n )
{
    // search thru bullet nodes and delete the first node with matching SceneNode
    vector<coreBulletNode *>::iterator it = bullet_nodes.begin();

    while( it != bullet_nodes.end() ) {
        if( (*it)->getNode() == n ) {
            bullet_nodes.erase( it );
            delete *it;
            return true;
        }
        it++;
    }

    // no node found that matches
    return false;
}



// add a node
// if it's dynamic, add it to the node list.  If it's not, don't.
btRigidBody *coreBullet::addNode( btCollisionShape *shape, Real m, SceneNode *n )
{
    btVector3 localInertia( 0, 0, 0 );

    // check if it's a dynamic rigid body or a static collision object
    // if dynamic calculate it's local inertia
    if( m > 0.0f ) shape->calculateLocalInertia( m, localInertia );

    // get Ogre3D world position of SceneNode
    Vector3 pos = n->getWorldPosition();

    // get the Ogre3D world orientation of SceneNode
    Quaternion q = n->getWorldOrientation();

    // setting up transform for new physics object from Ogre3D SceneNode coord.
    btTransform trans;
    trans.setIdentity();
    trans.setRotation( btQuaternion( q.x, q.y, q.z, q.w ) );
    trans.setOrigin( btVector3( pos.x, pos.y, pos.z ) );

    // using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    // btDefaultMotionState is inhereted by coreBulletNode class
    coreBulletNode *myNewNode = new coreBulletNode( n, trans );
    btRigidBody *body = new btRigidBody( m, myNewNode, shape, localInertia );

    // add new rigid body to the Bullet physics world
    bullet_world->addRigidBody( body );

    // Only do CCD if  motion in one timestep exceeds size of object
    body->setCcdSquareMotionThreshold( 10.0f );

    // Experimental: better estimation of CCD Time of Impact
    body->setCcdSweptSphereRadius( 2.0f );

    //?? messing around?? body->setRestitution( 0.1 );

    myNewNode->mass = m;
    myNewNode->name = n->getName();
    myNewNode->body = body;

    // If it's a dynamic node, store pointer to coreBulletNode in the manager's list of bullet nodes
    if( m > 0.0f ) bullet_nodes.push_back( myNewNode ); else static_nodes.push_back( myNewNode );

    return body;
}



// Finds a btcollisionshape in the list of already-built collision shapes
// If none is found return 0
btCollisionShape *coreBullet::getShape( String meshname )
{
    map<String, btCollisionShape *>::iterator it = shapes.find( meshname );
    if( it != shapes.end() ) return shapes[ meshname ];
    else return 0;
}



// Adds a collision shape to the map of all define shape instances
btCollisionShape *coreBullet::addSphere( String shapename, Real radius )
{
    // create a new Bullet collision shape and store it in a string-pointer map
    btCollisionShape *newshape = new btSphereShape( radius );
    shapes[ shapename ] = newshape;

    // set the collision gap
    newshape->setMargin( collision_margin );

    return newshape;
}



// Adds a collisions shape to the map of all defined shape instances
btCollisionShape *coreBullet::addBox( String shapename, Vector3 size )
{
    // adjust size by collision margin factor
    size -= Vector3::UNIT_SCALE * 2 * collision_margin;

    // create the box shape
    btCollisionShape *newshape = new btBoxShape( btVector3( size.x, size.y, size.z ) );

    // store a pointer to the shape in the shape map (lookup by shape name)
    shapes[ shapename ] = newshape;

    // set the collision gap
    newshape->setMargin( collision_margin );

    return newshape;
}



// Load mesh data from Ogre3D mesh (and it's submeshes) into a Bullet btBvhTriangleMeshShape (collisions)
btCollisionShape *coreBullet::addStaticMesh( String shapename, Mesh *mesh )
{
    size_t vertex_count, index_count;
    Vector3 *vertices;
    btVector3 *bullet_vertices;
    unsigned long int *indices;
    int *bullet_indices;

    // Ogre_getmeshdata.cpp: grabs all mesh data from an Ogre3D mesh and stores it in supplied memory areas
    // We must clean up after it (delete memory it allocates using our empty pointers we pass to it).
    getMeshInformation( mesh, vertex_count, vertices, index_count, indices );

    LogManager::getSingleton().logMessage(LML_NORMAL, "Vertices in mesh: " +
        StringConverter::toString( vertex_count ) );
    LogManager::getSingleton().logMessage(LML_NORMAL, "Triangles in mesh: " +
        StringConverter::toString( index_count / 3 ) );

    // Build Bullet verticies from Ogre3D verticies - straight-forward copy of vectors
    bullet_vertices = new btVector3[ vertex_count ];
    for( unsigned long int i = 0; i < vertex_count; i++ ) {
        bullet_vertices[ i ].setX( vertices[ i ].x );
        bullet_vertices[ i ].setY( vertices[ i ].y );
        bullet_vertices[ i ].setZ( vertices[ i ].z );
    }

    // Build Bullet indices from Ogre3D indicies - WARNING unsigned long int -> int conversion!!!!
    // Bullet shape may be missing some of the data if the Ogre3D mesh contains more than 2^15 points
    bullet_indices = new int[ index_count ];
    for( int i = 0; i < (int) index_count; i++ ) {
        bullet_indices[ i ] = (int) indices[ i ];
    }

    // create a Bullet triangle vertex array for building the Bullet triangle mesh collisions shape
    btTriangleIndexVertexArray *index_vert_arrays = new btTriangleIndexVertexArray(
        (int) index_count / 3,      // WARNING: Ogre3D return size_t for this value
        bullet_indices,             // WARNING: Ogre3D uses unsigned long int here - converted to int for Bullet
        (int) 3 * sizeof( int ),
        (int) vertex_count,         // WARNING: Ogre3D returns size_t for this value
        (float *) &bullet_vertices[ 0 ].x(),
        (int) sizeof( btVector3 ) );

    // create the collision shape

    btCollisionShape *newshape = new btBvhTriangleMeshShape();

    // save pointer to the shape in the collision shapes map, referenced by String name of shape
    // name is usually derived from the .mesh file used to create it
    shapes[ shapename ] = newshape;

    // set the collision gap
    newshape->setMargin( collision_margin );

    // return a pointer to the shape
    return newshape;

    // cleanup after getMeshInformation
    delete[] vertices;
    delete[] indices;
    delete[] bullet_vertices;
    delete[] bullet_indices;
}



// transfers Bullet's m_graphicsWorldTrans transform into Ogre3D's SceneNode
void coreBulletNode::hammerNode()
{
    // m_graphicsWorldTrans is a member of DefaultMotionState
    // it holds a brTransform which contains position and orientation data of the physics object
    // since we inheret btMotionState, coreBulletNode contains the m_graphicsWorldTrans transform
    btVector3 origin = m_graphicsWorldTrans.getOrigin();
    btQuaternion rot = m_graphicsWorldTrans.getRotation();

    // hammer btVector3 and btQuanternion into SceneNode
    node->setPosition( origin.x(), origin.y(), origin.z() );
    node->setOrientation( rot.getW(), rot.x(), rot.y(), rot.z() );
}
