/***************************************************************************

This source file is part of OGREBULLET
(Object-oriented Graphics Rendering Engine Bullet Wrapper)
For the latest info, see http://www.ogre3d.org/phpBB2addons/viewforum.php?f=10

Copyright (c) 2007 tuan.kuranes@gmail.com



This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
-----------------------------------------------------------------------------
*/
#ifndef _GMeshConverter_H_
#define _GMeshConverter_H_

#include "Ogre.h"
#include "btBulletCollisionCommon.h"

using namespace Ogre;

typedef std::vector<Ogre::Vector3> Vector3Vector;
typedef std::map<unsigned char, Vector3Vector* > BoneIndex;
typedef std::pair<unsigned short, Vector3Vector* > BoneKeyIndex;

class GMeshConverter
{
public:
    GMeshConverter(Ogre::Entity *entity,const Ogre::Matrix4 &transform = Ogre::Matrix4::IDENTITY);
    GMeshConverter();
    ~GMeshConverter();

    void addEntity(Ogre::Entity *entity,const Ogre::Matrix4 &transform = Ogre::Matrix4::IDENTITY);
    // Cannot be animated.
    void addMesh(const Ogre::MeshPtr &mesh, const Ogre::Matrix4 &transform = Ogre::Matrix4::IDENTITY);

    Ogre::Real      getRadius();
    Ogre::Vector3   getSize();


    btSphereShape*           createSphere();
    btBoxShape*              createBox();
    btTriangleMeshShape*     createTrimesh();
    btCylinderShape*         createCylinder();
    btConvexHullShape*       createConvex();


    const Ogre::Vector3*    getVertices();
    unsigned int            getVertexCount();
    const unsigned int*     getIndices();
    unsigned int            getIndexCount();

    btVector3               getBulletVector( Ogre::Vector3& vector ) const;
    Ogre::Vector3           getOgreVector( btVector3& vector ) const;

protected:

    Ogre::Entity*		mEntity;
    Ogre::SceneNode*	mNode;
    Ogre::Matrix4		mTransform;

    Ogre::Vector3*	    mVertexBuffer;
    unsigned int*       mIndexBuffer;
    unsigned int        mVertexCount;
    unsigned int        mIndexCount;

    BoneIndex *mBoneIndex;


    void addVertexData(const Ogre::VertexData *vertex_data,
                        const Ogre::VertexData *blended_data = 0,
                        const Ogre::Mesh::IndexMap *indexMap = 0);

    void addIndexData(Ogre::IndexData *data, const unsigned int offset = 0);

    bool addBoneVertices(unsigned char bone,unsigned int &vertex_count,
                        Ogre::Vector3* &vertices);

private:
    Ogre::Vector3		mBounds;
    Ogre::Real		    mBoundRadius;

};


#endif