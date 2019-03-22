//
// Created by alex on 19.03.19.
//

#pragma once

#include <pcl/octree/octree_pointcloud.h>
#include <set>

namespace pcl
{
	namespace octree
	{

		struct SCDevice
		{
			std::string type;
			std::string identifier;
			int point_count;
		};


		class VoxelNode
		{
			friend class VoxelList;

			public:
				VoxelNode(int voxel_key) {
					this->voxel_key_ = voxel_key;
				}

			private:
				int voxel_key_;
				VoxelNode *next_;
		};

		class VoxelList
		{

			public:

				VoxelList() {
					this->head_ = NULL;
					this->tail_ = NULL;
				}

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  */
				uint8_t
				insert(VoxelNode * const& node) {
					//int head_key, tail_key, curr_key;

					if (this->head_ == NULL && this->tail_ == NULL) {
						this->insertStart(node);
						return 2;
					}

					if (this->head_->voxel_key_ == node->voxel_key_ || this->tail_->voxel_key_ == node->voxel_key_) {
						return 1;
					}

					if (this->head_->voxel_key_ > node->voxel_key_) {
						insertStart(node);
						return 0;
					}

					if (this->tail_->voxel_key_ < node->voxel_key_) {
						insertEnd(node);
						return 0;
					}

					VoxelNode *curr = this->head_;

					// TODO Divide and Conquer approach
					// TODO Possibly Doubly Linked List
					while (curr != NULL) {
						if (curr->voxel_key_ < node->voxel_key_) {
							insertAfter(node, curr);
							return 0;
						}
						curr = curr->next_;
					}
				}

			private:

				VoxelNode *head_, *tail_;

				void
				insertStart(VoxelNode * const& node) {
					VoxelNode *temp = this->head_;
					this->head_ = node;
					if (temp != NULL)
						node->next_ = temp;
				}

				void
				insertEnd(VoxelNode * const& node) {
					if (this->tail_ == NULL)
					{
						this->insertStart(node);
						return;
					}
					this->tail_->next_ = node;
					this->tail_ = node;
				}

				void
				insertBefore(VoxelNode * const& node, VoxelNode * const& before, VoxelNode * const& beforeThat) {
					beforeThat->next_ = node;
					node->next_ = before;
				}

				void
				insertAfter(VoxelNode * const& node, VoxelNode * const& after) {
					node->next_ = after->next_;
					after->next_ = node;
				}
		};





		/** \brief @b Octree multi-pointcloud point wrapper
		  * \note This class implements a wrapper that stores a point from a pointcloud together with its's corresponding device.
		  * \author Alexander Poeppel (poeppel@isse.de)
		  */
		template<typename PointT>
		class OctreeMultiPointCloudPointWrapper
		{
		public:
			/** \brief Class initialization. */
			OctreeMultiPointCloudPointWrapper (PointT *point) {
				this(point, NULL, NULL);
			}

			OctreeMultiPointCloudPointWrapper (PointT *point, SCDevice *device) {
				this(point, device, NULL);
			}

			OctreeMultiPointCloudPointWrapper (PointT *point, SCDevice *device, PointCloud<PointT> *cloud) {
				this->point_ = point;
				this->device_ = device;
				this->cloud_ = cloud;
			}

			/** \brief Empty class deconstructor. */
			~OctreeMultiPointCloudPointWrapper () = default;


			SCDevice*
			getDevice() {
				return this->device_;
			}

			PointT*
			getPoint() {
				return this->point_;
			}

			PointCloud<PointT>*
			getPointCloud() {
				return this->cloud_;
			}




		private:
			SCDevice *device_;
			PointT *point_;
			PointCloud<PointT> *cloud_;
		};


		/** \brief @b Octree multi-pointcloud leaf node class
		  * \note This class implements a leaf node that calculates the mean centroid of all points from multiple input point clouds added to this octree container.
		  * \author Alexander Poeppel (poeppel@isse.de)
		  */
		template<typename PointT>
		class OctreeMultiPointCloudContainer : public OctreeContainerBase
		{
			public:
				/** \brief Class initialization. */
				OctreeMultiPointCloudContainer () {
					this->reset();
				}

				/** \brief Empty class deconstructor. */
				~OctreeMultiPointCloudContainer () {
				}

				void
				registerDevices(std::set<SCDevice*> devices) {
					for (std::set<SCDevice*>::iterator it = devices.begin(), end = devices.end(); it != end; it++) {
						this->point_map_.insert(it, std::vector<PointT>());

					}
				}

				/** \brief deep copy function */
				virtual OctreeMultiPointCloudContainer *
				deepCopy () const {
					return (new OctreeMultiPointCloudContainer (*this));
				}

				/** \brief Equal comparison operator - set to false
				 */
				// param[in] OctreeMultiPointCloudContainer to compare with
				bool operator==(const OctreeContainerBase&) const override {
					return ( false );
				}

				/** \brief Add new point to voxel.
				  * \param[in] new_point the new point to add
				  */
				void
				addPoint (PointT *new_point) {
					this->addPoint(new OctreeMultiPointCloudPointWrapper<PointT>(new_point));
				}

				/** \brief Add new point to voxel.
				  * \param[in] new_point the new point to add
				  */
				void
				addPoint (PointT *new_point, PointCloud<PointT> *point_cloud) {
					this->addPoint(new OctreeMultiPointCloudPointWrapper<PointT>(new_point, point_cloud));
				}

				/** \brief Add new point to voxel.
				  * \param[in] new_point the new point to add
				  */
				void
				addPoint (OctreeMultiPointCloudPointWrapper<PointT> *new_point) {
					this->virgin_ = false;
					using namespace pcl::common;

					++point_counter_;

					//point_sum_ += *(new_point->getPoint());


					this->point_map_.insert(new_point->getDevice(), new_point);
				}

				void
				clearPointsForDevice(SCDevice* device) {
					this->point_map_.find(device)->second.clear();
				}

				/** \brief Calculate centroid of voxel.
				  * \param[out] centroid_arg the resultant centroid of the voxel
				  */
				void
				getCentroid (PointT& centroid_arg) const {
					/*using namespace pcl::common;

					if (point_counter_)
					{
						centroid_arg = point_sum_;
						centroid_arg /= static_cast<float> (point_counter_);
					}
					else
					{
						centroid_arg *= 0.0f;
					}*/
				}

				/** \brief Calculate weighted centroid of voxel.
				  * \param[out] centroid_arg the resultant centroid of the voxel
				  */
				void
				getWeightedCentroid (PointT& centroid_arg) const {
					using namespace pcl::common;

					// TODO calculate centroid from all contained points

					/*if (point_counter_)
					{
						centroid_arg = point_sum_;
						centroid_arg /= static_cast<float> (point_counter_);
					}
					else
					{
						centroid_arg *= 0.0f;
					}*/
				}

				/** \brief Reset leaf container. */
				void
				reset () override {
					using namespace pcl::common;

					point_counter_ = 0;
					point_sum_ *= 0.0f;
				}

				bool
				isVirgin() {
					return virgin_;
				}

			private:
				unsigned int point_counter_;
				PointT point_sum_;
				std::map<SCDevice*, std::vector<PointT>> point_map_;
				bool virgin_ = true;


		};

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
				}

				/** \brief Add new point to voxel.
				  * \param[in] new_point the new point to add
				  */
				void
				addPointCloud (SCDevice *device, PointCloud<PointT> *cloud) {
					// Don't allow the addition of new devices after insertion of points has begun
					this->running = true;

					// Remove all old points for the given device first
					std::set<LeafContainerT*> *occupied_voxels = &(this->device_voxel_map_.find(device)->second);
					for (auto it = occupied_voxels->begin(), end = occupied_voxels->end(); it != end; it++) {
						it->clearPointsForDevice(device);
					}

					// Then insert the point cloud for the sudo given device
					std::vector<PointT> *points;
					points = cloud->points;
					for (int i=0; i<points->size(); i++) {
						this->addPoint(new OctreeMultiPointCloudPointWrapper<PointT>(&(points[i]), cloud, device));
						//return;
					}
				}

				/** \brief Add DataT object to leaf node at octree key.
				  * \param pointIdx_arg
				  */
				void
				addPointIdx (const int point_idx_arg) override {
					return;
				}

				bool
				registerDevice(SCDevice* device) {
					if (!this->running) {
						bool ret = registered_devices_.insert(device).second;
						if (ret)
							device_voxel_map_.insert(std::pair<SCDevice*, std::set<uint32_t >>(device, std::set<uint32_t >()));
					}

					/*for (typename OctreeMultiPointCloud<PointT>::LeafNodeBreadthFirstIterator it = this->leaf_breadth_begin(),
								end=this->leaf_breadth_end(); it!= end; ++it) {
						it->
					}*/
					return false;
				}

				void
				addPoint (OctreeMultiPointCloudPointWrapper<PointT>* const& new_point) {
					this->running = true;
					OctreeKey key;

					// make sure bounding box is big enough
					adoptBoundingBoxToPoint (new_point);

					// generate key
					genOctreeKeyforPoint (new_point, key);

					LeafNode* leaf_node;
					BranchNode* parent_branch_of_leaf_node;
					unsigned int depth_mask = this->createLeafRecursive (key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);

					if (this->dynamic_depth_enabled_ && depth_mask)
					{
						// get amount of objects in leaf container
						size_t leaf_obj_count = (*leaf_node)->getSize ();

						while  (leaf_obj_count>=this->max_objs_per_leaf_ && depth_mask)
						{
							// index to branch child
							unsigned char child_idx = key.getChildIdxWithDepthMask (depth_mask*2);

							expandLeafNode (leaf_node,
											parent_branch_of_leaf_node,
											child_idx,
											depth_mask);

							depth_mask = this->createLeafRecursive (key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);
							leaf_obj_count = (*leaf_node)->getSize ();
						}

					}


					if ((*leaf_node)->isVirgin ())
						(*leaf_node)->registerDevices(registered_devices_);
					(*leaf_node)->addPoint (new_point);

					device_voxel_map_.find(new_point->getDevice())->second.insert((*leaf_node));
				}

			private:

				std::set<SCDevice*> registered_devices_;
				std::map<SCDevice*, std::set<LeafContainerT*>> device_voxel_map_;
				bool running = false;


		};
	}
}

// Note: Don't precompile this octree type to speed up compilation. It's probably rarely used.
//#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>
