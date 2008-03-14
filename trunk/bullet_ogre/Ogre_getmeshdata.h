#ifndef H_MESHDATA_H
#define H_MESHDATA_H

void calc_tightboundbox( const Ogre::Mesh *const mesh, Ogre::Vector3 &v0, Ogre::Vector3 &v1 );

void calc_bulletbox( const Ogre::Mesh *const mesh, Ogre::Vector3 &s );

void getMeshInformation( const Ogre::Mesh *const mesh,
                            size_t &vertex_count,
                            Ogre::Vector3 *&vertices,
                            size_t &index_count,
                            unsigned long *&indices,
                            const Ogre::Vector3 &position = Ogre::Vector3::ZERO,
                            const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY,
                            const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE );

#endif
