//
// Created by alex on 19.03.19.
//

#pragma once

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_HPP
#define PCL_OCTREE_MULTI_POINTCLOUD_HPP


#include <pcl/octree/octree_pointcloud.h>

// Octree MultiPointCloud
#include <pcl/octree/octree_voxel_list.h>
#include <pcl/octree/octree_multi_pointcloud_container.h>
#include <pcl/octree/octree_multi_pointcloud_wrapper.h>
#include <pcl/octree/octree_multi_pointcloud_device.h>



#include <pcl/common/io.h>
#include <set>
#include <utility>
#include <chrono>

#include <cassert>      // assert
#include <cstddef>      // ptrdiff_t
#include <iterator>     // iterator
#include <type_traits>  // remove_cv
#include <utility>      // swap

namespace pcl {
	namespace octree {




		/** \brief @b Octree pointcloud voxel centroid class
		  * \note This class generate an octrees from a point cloud (zero-copy). It provides a vector of centroids for all occupied voxels.
		  * \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
		  * \note
		  * \note typename: PointT: type of point used in pointcloud
		  *
		  * \ingroup octree
		  * \author Julius Kammerl (julius@kammerl.de)
		  */
		template<typename PointT,
				typename LeafContainerT = OctreeMultiPointCloudContainer<PointT> ,
				typename BranchContainerT = OctreeContainerEmpty >
		class OctreeMultiPointCloud : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT> {
			public:
				typedef boost::shared_ptr<OctreeMultiPointCloud<PointT, LeafContainerT> > Ptr;
				typedef boost::shared_ptr<const OctreeMultiPointCloud<PointT, LeafContainerT> > ConstPtr;

				typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT> OctreeT;
				typedef typename OctreeT::LeafNode LeafNode;
				typedef typename OctreeT::BranchNode BranchNode;

				/** \brief OctreeMultiPointClouds class constructor.
				  * \param[in] resolution_arg octree resolution at lowest octree level
				  */
				OctreeMultiPointCloud (const double resolution_arg) :
						OctreePointCloud<PointT, LeafContainerT, BranchContainerT> (resolution_arg) {
				}

				/** \brief Empty class deconstructor. */

				~OctreeMultiPointCloud () {
					std::cout << "~OctreeMultiPointCloud DEstructor called!" << std::endl;
					for (VoxelList<LeafNode>* list : device_voxel_map_) {
						if (list)
							delete list;
					}
				}

				/** \brief Add new point to voxel.
				  * \param[in] new_point the new point to add
				  */
				void
				addPointCloud (SCDevice *device, PointCloud<PointT> *cloud) {
					pcl::PointCloud<PointT> *cloud_copy = new pcl::PointCloud<PointT>();

					pcl::copyPointCloud(*cloud, *cloud_copy);

					// Don't allow the addition of new devices after insertion of points has begun
					this->running = true;

					std::cout << "Adding PointCloud for device '" << device->identifier << "' of type '" << device->type << "'" << std::endl;

					auto start = std::chrono::steady_clock::now();
					// Remove all old points for the given device first
					//std::set<LeafContainerT*> *occupied_voxels = this->device_voxel_map_.find(device->device_id)->second;
					//VoxelList<LeafContainerT> *occupied_voxels = this->device_voxel_map_.find(device->device_id)->second;
					VoxelList<LeafNode> *occupied_voxels;
					try {
						 occupied_voxels = this->device_voxel_map_.at(device->device_id);
					}
					catch (const std::out_of_range& oor) {
						std::cerr << "Out of Range error: " << oor.what() << '\n';
						return;
					}
					//pcl::octree::OctreeMultiPointCloudContainer<pcl::PointXYZ>* temp;
					//temp->clearPointsForDevice();
					//for (typename std::set<LeafContainerT*>::iterator it = occupied_voxels->begin(), end = occupied_voxels->end(); it != end; it++) {
					for (auto item : *occupied_voxels) {
						OctreeMultiPointCloudContainer<PointT> multi_container = item->getContainer();
						if (multi_container.isVirgin ())
							multi_container.registerDevices(&registered_devices_);
						else
							multi_container.clearPointsForDevice(device);
					}
					//delete occupied_voxels;
					occupied_voxels->clear();
					auto end = std::chrono::steady_clock::now();

					std::cout << "Deleting old points took "
						 << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
						 << " ms" << std::endl;

					start = std::chrono::steady_clock::now();
					// Then insert the point cloud for the given device
					//std::vector< PointT, Eigen::aligned_allocator< PointT > > *points = &cloud_copy->points;
					auto *points = &cloud_copy->points;
					for (int i=0; i<points->size(); i++) {
						auto new_point = new OctreeMultiPointCloudPointWrapper<PointT>(&(points->at(i)), device, cloud);
						this->addPoint(new_point);
						//return;
					}
					end = std::chrono::steady_clock::now();
					std::cout << "Inserting new points took "
							  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
							  << " ms" << std::endl;

					PointCloud<PointT> *temp = current_point_clouds_[device->device_id];
					if (temp != nullptr) {
						delete temp;
					}
					current_point_clouds_[device->device_id] = cloud_copy;
				}

				/** \brief Add DataT object to leaf node at octree key.
				  * \param pointIdx_arg
				  */
				void
				addPointIdx (const int) override {
					return;
				}

				bool
				registerDevice(SCDevice* device) {
					if (!this->running) {
						bool ret = registered_devices_.insert(device).second;
						if (ret) {
							//device_voxel_map_.insert(std::pair<int, std::set<LeafContainerT*>*>(device->device_id, new std::set<LeafContainerT*>()));
							device_voxel_map_.insert(device_voxel_map_.end(), new VoxelList<LeafNode>());
							current_point_clouds_.insert(current_point_clouds_.end(), nullptr);
						}
					}

					/*for (typename OctreeMultiPointCloud<PointT>::LeafNodeBreadthFirstIterator it = this->leaf_breadth_begin(),
								end=this->leaf_breadth_end(); it!= end; ++it) {
						it.getLeafContainer().registerDevices();
					}*/
					return false;
				}

				void
				addPoint (OctreeMultiPointCloudPointWrapper<PointT>* const& new_point) {
					this->running = true;
					OctreeKey key;

					// make sure bounding box is big enough
					this->adoptBoundingBoxToPoint (*(new_point->getPoint()));

					// generate key
					this->genOctreeKeyforPoint (*(new_point->getPoint()), key);

					LeafNode* leaf_node;
					BranchNode* parent_branch_of_leaf_node;
					unsigned int depth_mask = this->createLeafRecursive(key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);

					if (this->dynamic_depth_enabled_ && depth_mask)
					{
						// get amount of objects in leaf container
						size_t leaf_obj_count = (*leaf_node)->getSize ();

						while (leaf_obj_count>=this->max_objs_per_leaf_ && depth_mask)
						{
							// index to branch child
							unsigned char child_idx = key.getChildIdxWithDepthMask(depth_mask*2);

							this->expandLeafNode (leaf_node,
												  parent_branch_of_leaf_node,
												  child_idx,
												  depth_mask);

							depth_mask = this->createLeafRecursive(key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);
							leaf_obj_count = (*leaf_node)->getSize();
						}

					}


					if ((*leaf_node)->isVirgin())
						(*leaf_node)->registerDevices(&registered_devices_);
					(*leaf_node)->addPoint(new_point);

					auto temp = device_voxel_map_.at(new_point->getDevice()->device_id);
					//temp->insert(reinterpret_cast<LeafContainerT*>(&(*leaf_node)));
					temp->insert(leaf_node);
					//temp->second->insert(reinterpret_cast<LeafContainerT*>(&(*leaf_node)));
					//temp->second->insert(new OctreeMultiPointCloudContainer<PointXYZ>);
				}

			private:

				std::set<SCDevice*> registered_devices_;  // Saves list of all currently registered devices
				std::vector<PointCloud<PointT>*> current_point_clouds_;
				//std::map<int, std::set<LeafContainerT*>*> device_voxel_map_;  // Saves combination of device id and all occupied voxels
				//std::vector<VoxelList<LeafContainerT>*> device_voxel_map_;  // Saves combination of device id and all occupied voxels
				std::vector<VoxelList<LeafNode>*> device_voxel_map_;  // Saves combination of device id and all occupied voxels
				//VoxelList* device_voxel_map = new VoxelList;
				bool running = false;


		};
	}
}

// TODO Evaluate Notice: Note: Don't precompile this octree type to speed up compilation. It's probably rarely used.
//#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>

#endif  // PCL_OCTREE_MULTI_POINTCLOUD_HPP