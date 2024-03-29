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

#include "GMeshConverter.h"

using namespace Ogre;

//------------------------------------------------------------------------------------------------
void GMeshConverter::addVertexData(const VertexData *vertex_data,
                                   const VertexData *blended_data,
                                   const Mesh::IndexMap *indexMap)
{
	if (!vertex_data)
        return;

	const VertexData *data = (blended_data) ? blended_data: vertex_data;

	const unsigned int prev_size = mVertexCount;
    mVertexCount += (unsigned int)data->vertexCount;

    Ogre::Vector3* tmp_vert = new Ogre::Vector3[mVertexCount];
	if (mVertexBuffer)
	{
	    std::cout<<"Empty Vertex Buffer\n";
		memcpy(tmp_vert,mVertexBuffer,sizeof(Vector3) * prev_size);
		delete[] mVertexBuffer;
	}
	mVertexBuffer = tmp_vert;

	// Get the positional buffer element
    {
        const Ogre::VertexElement* posElem = data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
        Ogre::HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
        const unsigned int vSize = (unsigned int)vbuf->getVertexSize();

	    unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
	    float* pReal;
        Ogre::Vector3 * curVertices = &mVertexBuffer[prev_size];
        const unsigned int vertexCount = (unsigned int)data->vertexCount;

        std::cout<<"Fill Vertex Buffer\n";
	    for(unsigned int j = 0; j < vertexCount; ++j)
	    {
		    posElem->baseVertexPointerToElement(vertex, &pReal);
            vertex += vSize;

		    curVertices->x = (*pReal++);
		    curVertices->y = (*pReal++);
		    curVertices->z = (*pReal++);

		    *curVertices = mTransform * (*curVertices);

            curVertices++;
        }
	    vbuf->unlock();
    }

	// Get the bone index element
	if (mEntity->hasSkeleton())
	{

        Ogre::MeshPtr mesh = mEntity->getMesh ();

		const Ogre::VertexElement* bneElem = vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_BLEND_INDICES);
		assert (bneElem);
		{
            Ogre::HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(bneElem->getSource());
            const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
			unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

			unsigned char* pBone;

            if (!mBoneIndex)
                mBoneIndex = new BoneIndex();
            BoneIndex::iterator i;

            Ogre::Vector3 * curVertices = &mVertexBuffer[prev_size];

            const unsigned int vertexCount = (unsigned int)vertex_data->vertexCount;
			for(unsigned int j = 0; j < vertexCount; ++j)
			{
				bneElem->baseVertexPointerToElement(vertex, &pBone);
                vertex += vSize;

                const unsigned char currBone = (indexMap) ? (*indexMap)[*pBone] : *pBone;
				i = mBoneIndex->find (currBone);
				Vector3Vector* l = 0;
				if (i == mBoneIndex->end())
				{
					l = new Vector3Vector;
					mBoneIndex->insert(BoneKeyIndex(currBone, l));
				}
				else
                {
                    l = i->second;
                }

				l->push_back(*curVertices);

                curVertices++;
			}
			vbuf->unlock();
		}
	}
}


//------------------------------------------------------------------------------------------------
void GMeshConverter::addIndexData(IndexData *data, const unsigned int offset)
{
    const unsigned int prev_size = mIndexCount;
    mIndexCount += (unsigned int)data->indexCount;

	unsigned int* tmp_ind = new unsigned int[mIndexCount];
	if (mIndexBuffer)
	{
		memcpy (tmp_ind, mIndexBuffer, sizeof(unsigned int) * prev_size);
		delete[] mIndexBuffer;
	}
	mIndexBuffer = tmp_ind;

	const unsigned int numTris = (unsigned int) data->indexCount / 3;
	HardwareIndexBufferSharedPtr ibuf = data->indexBuffer;
	const bool use32bitindexes = (ibuf->getType() == HardwareIndexBuffer::IT_32BIT);
    unsigned int index_offset = prev_size;

	if (use32bitindexes)
    {
        const unsigned int* pInt = static_cast<unsigned int*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
        for(unsigned int k = 0; k < numTris; ++k)
        {
            mIndexBuffer[index_offset ++] = offset + *pInt++;
            mIndexBuffer[index_offset ++] = offset + *pInt++;
            mIndexBuffer[index_offset ++] = offset + *pInt++;
        }
        ibuf->unlock();
    }
	else
    {
        const unsigned short* pShort = static_cast<unsigned short*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
		for(unsigned int k = 0; k < numTris; ++k)
        {
            mIndexBuffer[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
            mIndexBuffer[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
            mIndexBuffer[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
        }
        ibuf->unlock();
    }

}
//------------------------------------------------------------------------------------------------
void GMeshConverter::addEntity(Entity *entity,const Matrix4 &transform)
{
	// Each entity added need to reset size and radius
	// next time getRadius and getSize are asked, they're computed.
	mBounds  = Ogre::Vector3(-1,-1,-1);
	mBoundRadius = -1;

    std::cout<<"Add Entity\n";

	mEntity = entity;
	mNode = (SceneNode*)(mEntity->getParentNode());
	mTransform = transform;

	const bool isSkeletonAnimated = mEntity->hasSkeleton();
	//const bool isHWanimated = isSkeletonAnimated && entity->isHardwareAnimationEnabled();
	if (isSkeletonAnimated)
	{
		mEntity->addSoftwareAnimationRequest(false);
		mEntity->_updateAnimation();
	}


	if (mEntity->getMesh()->sharedVertexData)
	{
		if (!isSkeletonAnimated)
			addVertexData (mEntity->getMesh()->sharedVertexData);
		else
			addVertexData (mEntity->getMesh()->sharedVertexData,
			mEntity->_getSkelAnimVertexData(),
			&mEntity->getMesh()->sharedBlendIndexToBoneIndexMap);
	}

	for(unsigned int i = 0;i < mEntity->getNumSubEntities();++i)
	{
		SubMesh *sub_mesh = mEntity->getSubEntity(i)->getSubMesh();
        std::cout<<"Add Submesh\n";

		if (!sub_mesh->useSharedVertices)
		{
		    std::cout<<"Add Indices\n";
			addIndexData(sub_mesh->indexData,mVertexCount);

			if (!isSkeletonAnimated)
			{
			    std::cout<<"Add Vertices\n";
				addVertexData (sub_mesh->vertexData);
			}
			else
			{
			    std::cout<<"Add Vertices\n";
				addVertexData (sub_mesh->vertexData,
				mEntity->getSubEntity(i)->_getSkelAnimVertexData(),
				&sub_mesh->blendIndexToBoneIndexMap);
			}
		}
		else
		{
            std::cout<<"Add Indices\n";
			addIndexData (sub_mesh->indexData);
		}

	}

	if (isSkeletonAnimated)
		mEntity->removeSoftwareAnimationRequest(false);
}
//------------------------------------------------------------------------------------------------
void GMeshConverter::addMesh(const MeshPtr &mesh, const Matrix4 &transform)
{
	// Each entity added need to reset size and radius
	// next time getRadius and getSize are asked, they're computed.
	mBounds  = Ogre::Vector3(-1,-1,-1);
	mBoundRadius = -1;

	//_entity = entity;
	//_node = (SceneNode*)(_entity->getParentNode());
	mTransform = transform;

	if (mesh->hasSkeleton ())
		Ogre::LogManager::getSingleton().logMessage("GMeshConverter::addMesh : Mesh " + mesh->getName () + " as skeleton but added to trimesh non animated");
// 		OGRE_EXCEPT(1, "Mesh " + mesh->getName () + " as skeleton but added to trimesh non animated",
// 					"GMeshConverter::addMesh");

	if (mesh->sharedVertexData)
	{
		addVertexData (mesh->sharedVertexData);
	}

	for(unsigned int i = 0;i < mesh->getNumSubMeshes();++i)
	{
		SubMesh *sub_mesh = mesh->getSubMesh(i);

		if (!sub_mesh->useSharedVertices)
		{
			addIndexData(sub_mesh->indexData, mVertexCount);
			addVertexData (sub_mesh->vertexData);
		}
		else
		{
			addIndexData (sub_mesh->indexData);
		}

	}
}
//------------------------------------------------------------------------------------------------
GMeshConverter::GMeshConverter(Entity *entity,const Matrix4 &transform) :
	mVertexBuffer (0),
	mIndexBuffer (0),
	mVertexCount (0),
	mIndexCount (0),
    mBoneIndex (0),
	mBounds (Vector3(-1,-1,-1)),
	mBoundRadius (-1)
{
	addEntity(entity, transform);
}
//------------------------------------------------------------------------------------------------
GMeshConverter::GMeshConverter() :
		mVertexBuffer (0),
		mIndexBuffer (0),
		mVertexCount (0),
		mIndexCount (0),
        mBoneIndex (0),
		mBounds (Vector3(-1,-1,-1)),
		mBoundRadius (-1)

{

}
//------------------------------------------------------------------------------------------------
Real GMeshConverter::getRadius()
{
	if (mBoundRadius == (-1))
	{
		getSize();
		mBoundRadius = (std::max(mBounds.x,std::max(mBounds.y,mBounds.z)) * 0.5);
	}
	return mBoundRadius;
}
//------------------------------------------------------------------------------------------------
Vector3 GMeshConverter::getSize()
{
    const unsigned int vCount = getVertexCount();
	if (mBounds == Ogre::Vector3(-1,-1,-1) && vCount > 0)
	{

		const Ogre::Vector3 * const v = getVertices();

        Ogre::Vector3 vmin(v[0]);
        Ogre::Vector3 vmax(v[0]);

		for(unsigned int j = 1; j < vCount; j++)
		{
			vmin.x = std::min(vmin.x, v[j].x);
			vmin.y = std::min(vmin.y, v[j].y);
			vmin.z = std::min(vmin.z, v[j].z);

			vmax.x = std::max(vmax.x, v[j].x);
			vmax.y = std::max(vmax.y, v[j].y);
			vmax.z = std::max(vmax.z, v[j].z);
		}

		mBounds.x = vmax.x - vmin.x;
		mBounds.y = vmax.y - vmin.y;
		mBounds.z = vmax.z - vmin.z;
	}

	return mBounds;
}
//------------------------------------------------------------------------------------------------
const Ogre::Vector3* GMeshConverter::getVertices()
{
	return mVertexBuffer;
}
//------------------------------------------------------------------------------------------------
unsigned int GMeshConverter::getVertexCount()
{
	return mVertexCount;
}
//------------------------------------------------------------------------------------------------
const unsigned int* GMeshConverter::getIndices()
{
	return mIndexBuffer;
}
//------------------------------------------------------------------------------------------------
unsigned int GMeshConverter::getIndexCount()
{
	return mIndexCount;
}

//------------------------------------------------------------------------------------------------
btSphereShape* GMeshConverter::createSphere()
{
	const Ogre::Real rad = getRadius();
	assert((rad > 0.0) &&
        ("Sphere radius must be greater than zero"));
    btSphereShape* shape = new btSphereShape(rad);

    return shape;
}
//------------------------------------------------------------------------------------------------
btBoxShape* GMeshConverter::createBox()
{
	Ogre::Vector3 sz = getSize();

	assert((sz.x > 0.0) && (sz.y > 0.0) && (sz.y > 0.0) &&
        ("Size of box must be greater than zero on all axes"));

    btVector3 vector = getBulletVector( sz );
	btBoxShape* shape = new btBoxShape( vector );
	return shape;
}
//------------------------------------------------------------------------------------------------
btCylinderShape* GMeshConverter::createCylinder()
{
    Ogre::Vector3 sz = getSize();

    assert((sz.x > 0.0) && (sz.y > 0.0) && (sz.y > 0.0) &&
        ("Size of Cylinder must be greater than zero on all axes"));

    btVector3 vector = getBulletVector( sz );
    btCylinderShape* shape = new btCylinderShape( vector );
    return shape;
}
//------------------------------------------------------------------------------------------------
btConvexHullShape* GMeshConverter::createConvex()
{
    assert(mVertexCount && (mIndexCount >= 6) &&
        ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

    return 0;//new ConvexHullCollisionShape (_vertices, _vertex_count, 3*sizeof(float));
}
//------------------------------------------------------------------------------------------------
btTriangleMeshShape* GMeshConverter::createTrimesh()
{
	assert(mVertexCount && (mIndexCount >= 6) &&
        ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

    std::cout<<"New Triangle Mesh\n";
    btTriangleMesh* triangleMesh = new btTriangleMesh();

    unsigned int numFaces = mIndexCount / 3;

    btVector3 vertexPos[3];
    for (size_t n = 0; n < numFaces; ++n)
    {
        std::cout<<"... Add to buffer\n...... Face: "<<n<<"\n";
        for (unsigned int i = 0; i < 3; ++i)
        {
            std::cout<<"..... Vertex: "<<i<<"\n";
            std::cout<<"..... Buffer: "<<mVertexBuffer<<"\n";
            const Vector3 &vec = mVertexBuffer[*mIndexBuffer];
            vertexPos[i][0] = vec.x;
            vertexPos[i][1] = vec.y;
            vertexPos[i][2] = vec.z;
            *mIndexBuffer++;
        }
        std::cout<<"... New Triangle\n";
        triangleMesh->addTriangle(vertexPos[0], vertexPos[1], vertexPos[2]);
    }

    const bool useQuantizedAABB = true;

    std::cout<<"Return the Shape\n";
    btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape( triangleMesh, useQuantizedAABB);

	return shape;
}
//------------------------------------------------------------------------------------------------
bool GMeshConverter::addBoneVertices(unsigned char bone, unsigned int &vertex_count, Ogre::Vector3* &vertices)
{
	BoneIndex::iterator i = mBoneIndex->find(bone);

    if (i == mBoneIndex->end())
        return false;

	if (i->second->empty())
        return false;

    vertex_count = (unsigned int) i->second->size() + 1;

	vertices = new Ogre::Vector3[vertex_count];
	vertices[0] = mEntity->_getParentNodeFullTransform() *
            mEntity->getSkeleton()->getBone(bone)->_getDerivedPosition();

	unsigned int o = 1;
	for(Vector3Vector::iterator j = i->second->begin();
        j != i->second->end(); ++j,++o)
    {
        vertices[o] = (*j);
    }
	return true;
}
//------------------------------------------------------------------------------------------------
GMeshConverter::~GMeshConverter()
{
	delete[] mVertexBuffer;
	delete[] mIndexBuffer;

	if (mBoneIndex)
	{
		for(BoneIndex::iterator i = mBoneIndex->begin();
            i != mBoneIndex->end();
            ++i)
		{
			delete i->second;
		}
		delete mBoneIndex;
	}
}
//------------------------------------------------------------------------------------------------

btVector3 GMeshConverter::getBulletVector( Ogre::Vector3& vector ) const
{
    return btVector3( vector.x, vector.y, vector.z );
}

//------------------------------------------------------------------------------------------------

Ogre::Vector3 GMeshConverter::getOgreVector( btVector3& vector ) const
{
    return Ogre::Vector3( Ogre::Real( vector.x() ), Ogre::Real( vector.y() ), Ogre::Real( vector.z() ) );
}

//------------------------------------------------------------------------------------------------
