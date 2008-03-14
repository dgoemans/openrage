#include "Ogre.h"
#include "Ogre_getmeshdata.h"

using namespace Ogre;



// given an Ogre3D mesh, calculate the tightest bounding box aligned in the z-axis
// return the data in v0 and v1
void calc_tightboundbox( const Mesh *const mesh, Vector3 &v0, Vector3 &v1 )
{
    size_t vertex_count, index_count;
    Vector3 *vertices;
    unsigned long int *indices;
    Real minx, miny, minz, maxx, maxy, maxz;

    // Ogre_getmeshdata.cpp: grabs all mesh data from an Ogre3D mesh and stores it in supplied memory areas
    // We must clean up after it (delete memory it allocates using our empty pointers we pass to it).
    getMeshInformation( mesh, vertex_count, vertices, index_count, indices );

    LogManager::getSingleton().logMessage( "[bounding box] creating tight bounding box" );

    // pre-load compare values
    minx = vertices[ 0 ].x; maxx = vertices[ 0 ].x;
    miny = vertices[ 0 ].y; maxy = vertices[ 0 ].y;
    minz = vertices[ 0 ].z; maxz = vertices[ 0 ].z;

    // loop through all vertex information
    for( unsigned long int i = 0; i < vertex_count; i++ ) {

        // compare vertex components to minx,miny,minz,maxx,maxy,maxz
        if( vertices[ i ].x > maxx ) maxx = vertices[ i ].x;
        if( vertices[ i ].y > maxy ) maxy = vertices[ i ].y;
        if( vertices[ i ].z > maxz ) maxz = vertices[ i ].z;
        if( vertices[ i ].x > minx ) minx = vertices[ i ].x;
        if( vertices[ i ].y > miny ) miny = vertices[ i ].y;
        if( vertices[ i ].z > minz ) minz = vertices[ i ].z;
    }

    v0.x = minx; v0.y = miny; v0.z = minz;
    v1.x = maxx; v1.y = maxy; v0.z = maxz;

    LogManager::getSingleton().logMessage( "( " + StringConverter::toString( minx ) +
        ", " + StringConverter::toString( miny ) + ", " + StringConverter::toString( minz ) + " )" );
    LogManager::getSingleton().logMessage( "( " + StringConverter::toString( maxx ) + ", " +
        StringConverter::toString( maxy ) + ", " + StringConverter::toString( maxz ) + " )" );

    // clean up after getMeshInformation() call
    delete[] vertices;
    delete[] indices;
}



// given an Ogre3D mesh, calculate the tightest bounding box aligned in the z-axis
// return the half-extends size of the box in v0
// Bullet Physics specific: Bullet likes half-extends
void calc_bulletbox( const Mesh *const mesh, Vector3 &s )
{
    size_t vertex_count, index_count;
    Vector3 *vertices;
    unsigned long int *indices;
    Real minx, miny, minz, maxx, maxy, maxz;
    Vector3 v0, v1;

    // Ogre_getmeshdata.cpp: grabs all mesh data from an Ogre3D mesh and stores it in supplied memory areas
    // We must clean up after it (delete memory it allocates using our empty pointers we pass to it).
    getMeshInformation( mesh, vertex_count, vertices, index_count, indices );

    // pre-load compare values
    minx = vertices[ 0 ].x; maxx = vertices[ 0 ].x;
    miny = vertices[ 0 ].y; maxy = vertices[ 0 ].y;
    minz = vertices[ 0 ].z; maxz = vertices[ 0 ].z;

    // loop through all vertex information
    for( unsigned long int i = 0; i < vertex_count; i++ ) {

        // compare vertex components to minx,miny,minz,maxx,maxy,maxz
        if( vertices[ i ].x > maxx ) maxx = vertices[ i ].x;
        if( vertices[ i ].y > maxy ) maxy = vertices[ i ].y;
        if( vertices[ i ].z > maxz ) maxz = vertices[ i ].z;
        if( vertices[ i ].x > minx ) minx = vertices[ i ].x;
        if( vertices[ i ].y > miny ) miny = vertices[ i ].y;
        if( vertices[ i ].z > minz ) minz = vertices[ i ].z;
    }

    v0.x = minx; v0.y = miny; v0.z = minz;
    v1.x = maxx; v1.y = maxy; v0.z = maxz;

    if( fabs( v0.x ) > fabs( v1.x ) ) s.x = fabs( v0.x ); else s.x = fabs( v1.x );
    if( fabs( v0.y ) > fabs( v1.y ) ) s.y = fabs( v0.y ); else s.y = fabs( v1.y );
    if( fabs( v0.z ) > fabs( v1.z ) ) s.z = fabs( v0.z ); else s.z = fabs( v1.z );

    // clean up after getMeshInformation() call
    delete[] vertices;
    delete[] indices;
}



// Extract verticies and indicies from an Ogre3D Mesh into arrays
void getMeshInformation( const Ogre::Mesh* const mesh, size_t &vertex_count,
                                 Ogre::Vector3* &vertices,
                                 size_t &index_count, unsigned long* &indices,
                                 const Ogre::Vector3 &position,
                                 const Ogre::Quaternion &orient,
                                 const Ogre::Vector3 &scale )
{
 bool added_shared = false;
 size_t current_offset = 0;
 size_t shared_offset = 0;
 size_t next_offset = 0;
 size_t index_offset = 0;


  vertex_count = index_count = 0;

    // Calculate how many vertices and indices we're going to need
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
      Ogre::SubMesh* submesh = mesh->getSubMesh( i );

        // We only need to add the shared vertices once
      if(submesh->useSharedVertices)
      {
          if( !added_shared )
          {
            vertex_count += mesh->sharedVertexData->vertexCount;
            added_shared = true;
          }
      }
      else
      {
          vertex_count += submesh->vertexData->vertexCount;
      }

      // Add the indices
      index_count += submesh->indexData->indexCount;
  }


  // Allocate space for the vertices and indices
  vertices = new Ogre::Vector3[vertex_count];
  indices = new unsigned long[index_count];

  added_shared = false;

  // Run through the submeshes again, adding the data into the arrays
  for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = mesh->getSubMesh(i);

    Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

    if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
    {
      if(submesh->useSharedVertices)
      {
        added_shared = true;
        shared_offset = current_offset;
      }

      const Ogre::VertexElement* posElem =
        vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

      Ogre::HardwareVertexBufferSharedPtr vbuf =
        vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char* vertex =
        static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

// There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
//  as second argument. So make it float, to avoid trouble when Ogre::Real will
//  be comiled/typedefed as double:
//      Ogre::Real* pReal;
      float* pReal;

      for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
      {
        posElem->baseVertexPointerToElement(vertex, &pReal);

        Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

        vertices[current_offset + j] = (orient * (pt * scale)) + position;
      }

      vbuf->unlock();
      next_offset += vertex_data->vertexCount;
    }


    Ogre::IndexData* index_data = submesh->indexData;
    size_t numTris = index_data->indexCount / 3;
    Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

    bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

    unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
    unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


    size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

    if ( use32bitindexes )
    {
      for ( size_t k = 0; k < numTris*3; ++k)
      {
        indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
      }
    }
    else
    {
      for ( size_t k = 0; k < numTris*3; ++k)
      {
        indices[index_offset++] = static_cast<unsigned long>(pShort[k]) +
                                  static_cast<unsigned long>(offset);
      }
    }

    ibuf->unlock();
    current_offset = next_offset;
  }
}
