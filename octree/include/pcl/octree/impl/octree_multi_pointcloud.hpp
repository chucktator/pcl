
#ifndef PCL_OCTREE_MULTI_POINTCLOUD_HPP
#define PCL_OCTREE_MULTI_POINTCLOUD_HPP

/*
 * OctreePointCloudVoxelcontroid is not precompiled, since it's used in other
 * parts of PCL with custom LeafContainers. So if PCL_NO_PRECOMPILE is NOT
 * used, octree_pointcloud_voxelcentroid.h includes this file but octree_pointcloud.h
 * would not include the implementation because it's precompiled. So we need to
 * include it here "manually".
 */
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/octree_multi_pointcloud.h>




#define PCL_INSTANTIATE_OctreeMultiPointCloud(T) template class PCL_EXPORTS pcl::octree::OctreeMultiPointCloud<T>;

#endif

