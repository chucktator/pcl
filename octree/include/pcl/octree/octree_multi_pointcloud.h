//
// Created by alex on 19.03.19.
//

#pragma once

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/common/io.h>
#include <set>
#include <utility>
#include <chrono>

#include <cassert>      // assert
#include <cstddef>      // ptrdiff_t
#include <iterator>     // iterator
#include <type_traits>  // remove_cv
#include <utility>      // swap

namespace pcl
{
	namespace octree
	{

		struct SCDevice
		{
			public:

				int device_id = SCDevice::id++;
				std::string type;
				std::string identifier;
				int point_count;

			private:
				static int id;

		};

		int SCDevice::id = 0;

		template <typename ListT> class VoxelList;

		template <typename ContentT>
		class VoxelNode {
			friend class VoxelList<ContentT>;

			public:
				VoxelNode(u_int voxel_key) {
					this->voxel_key_ = voxel_key;
				}

				VoxelNode(u_int voxel_key, ContentT *content) {
					this->voxel_key_ = voxel_key;
					this->content_ = content;
				}

				~VoxelNode() {
					delete content;
				}

			private:
				u_int voxel_key_ = 0;
				ContentT* content_ = nullptr;
				VoxelNode *next_ = nullptr;
		};

		template <typename ListT>
		class VoxelList
		{
			template <class Type, class UnqualifiedType = std::remove_cv_t<Type>>
			class ForwardIterator : public std::iterator<std::forward_iterator_tag,
					UnqualifiedType,
					std::ptrdiff_t,
					Type*,
					Type&>
			{
				friend class VoxelList;
				VoxelNode<UnqualifiedType>* itr;

				explicit ForwardIterator(VoxelNode<UnqualifiedType>* nd)
						: itr(nd)
				{
				}

			public:

				ForwardIterator()   // Default construct gives end.
						: itr(nullptr)
				{
				}

				void swap(ForwardIterator& other) noexcept
				{
					using std::swap;
					swap(itr, other.iter);
				}

				ForwardIterator& operator++ () // Pre-increment
				{
					assert(itr != nullptr && "Out-of-bounds iterator increment!");
					itr = itr->next_;
					return *this;
				}

				ForwardIterator operator++ (int) // Post-increment
				{
					assert(itr != nullptr && "Out-of-bounds iterator increment!");
					ForwardIterator tmp(*this);
					itr = itr->next_;
					return tmp;
				}

				// two-way comparison: v.begin() == v.cbegin() and vice versa
				template<class OtherType>
				bool operator == (const ForwardIterator<OtherType>& rhs) const
				{
					return itr == rhs.itr;
				}

				template<class OtherType>
				bool operator != (const ForwardIterator<OtherType>& rhs) const
				{
					return itr != rhs.itr;
				}

				Type* operator* () const
				{
					assert(itr != nullptr && "Invalid iterator dereference!");
					return itr->content_;
				}

				Type& operator-> () const
				{
					assert(itr != nullptr && "Invalid iterator dereference!");
					return *(*itr->content_);
				}

				// One way conversion: iterator -> const_iterator
				operator ForwardIterator<const Type>() const
				{
					return ForwardIterator<const Type>(itr);
				}
			};

			// `iterator` and `const_iterator` used by your class:
			typedef ForwardIterator<ListT> iterator;
			typedef ForwardIterator<const ListT> const_iterator;

			public:

				VoxelList() {
					this->head_ = nullptr;
					this->tail_ = nullptr;
				}
			
				~VoxelList() {
					this->clear();
				}

				uint8_t
				insert(ListT* const& content) {
					return this->insert(new VoxelNode<ListT>(reinterpret_cast<std::uintptr_t>(content), content));
				}

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				uint8_t
				insert(VoxelNode<ListT> * const& node) {
					//int head_key, tail_key, curr_key;

					if (this->head_ == nullptr && this->tail_ == nullptr) {
						this->head_ = node;
						this->tail_ = node;
						this->size++;
						return 2;
					}

					if (this->head_->voxel_key_ == node->voxel_key_ || this->tail_->voxel_key_ == node->voxel_key_) {
						return 1;
					}

					if (this->head_->voxel_key_ > node->voxel_key_) {
						insertStart(node);
						this->size++;
						return 0;
					}

					if (this->tail_->voxel_key_ < node->voxel_key_) {
						this->size++;
						insertEnd(node);
						return 0;
					}

					VoxelNode<ListT> *curr = this->head_;

					// TODO Divide and Conquer approach
					// TODO Possibly Doubly Linked List
					while (curr != nullptr) {
						if (curr->voxel_key_ == node->voxel_key_)
							return 1;
						if (curr->voxel_key_ < node->voxel_key_) {
							insertAfter(node, curr);
							this->size++;
							return 0;
						}
						curr = curr->next_;
					}

					return 3;
				}

				void
				clear() {
					VoxelNode<ListT> *curr = this->head_;
					this->head_ = nullptr;
					this->tail_ = nullptr;

					while (curr != nullptr) {
						VoxelNode<ListT> *temp = curr;
						curr = curr->next_;
						delete temp;
					}
				}

				ForwardIterator<ListT>
				begin() {
					return ForwardIterator<ListT>(this->head_);
				}

				ForwardIterator<ListT>
				end() {
					return ForwardIterator<ListT>(this->tail_);
				}



			private:

				VoxelNode<ListT> *head_, *tail_;
				uint64_t size = 0;

				void
				insertStart(VoxelNode<ListT> * const& node) {
					VoxelNode<ListT> *temp = this->head_;
					this->head_ = node;
					if (temp != NULL)
						node->next_ = temp;
				}

				void
				insertEnd(VoxelNode<ListT> * const& node) {
					if (this->tail_ == NULL)
					{
						this->insertStart(node);
						return;
					}
					this->tail_->next_ = node;
					this->tail_ = node;
				}

				void
				insertBefore(VoxelNode<ListT> * const& node, VoxelNode<ListT> * const& before, VoxelNode<ListT> * const& beforeThat) {
					beforeThat->next_ = node;
					node->next_ = before;
				}

				void
				insertAfter(VoxelNode<ListT> * const& node, VoxelNode<ListT> * const& after) {
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
		class OctreeMultiPointCloudContainer : public OctreeContainerBase {
			
			public:
				/** \brief Class initialization. */
				OctreeMultiPointCloudContainer () {
					this->reset();
					//virgin_ = true;
					//this->point_map_ = new std::map<int, std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>;
					this->point_map_ = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>();
				}

				/** \brief Class deconstructor. */
				~OctreeMultiPointCloudContainer () {
					for (auto dev = this->point_map_->begin(), end = this->point_map_->end(); dev != end; ++dev) {
						for (auto points = (*dev)->begin(), end = (*dev)->end(); points != end; ++points) {
							delete (*points);
						}
						delete (*dev);
					}
					delete this->point_map_;
				}

				void
				registerDevices(std::set<SCDevice*> *devices) {
					if (!virgin_)
						return;

					//if (!this->point_map_)
						//this->point_map_ = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>;
						//this->point_map_ = new std::map<int, std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>;
					// std::map<SCDevice*, std::vector<PointT*>> this->point_map_;
					for (auto it = devices->begin(), end = devices->end(); it != end; it++) {
						auto point_vector = new std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>;
						point_vector->reserve((*it)->point_count / 1000);
						this->point_map_->insert(this->point_map_->end(), point_vector);
						//this->point_map_->insert({ (*it)->device_id, new std::vector<OctreeMultiPointCloudPointWrapper<PointT>*> });
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
					virgin_ = false;
					using namespace pcl::common;

					++point_counter_;

					//point_sum_ += *(new_point->getPoint());

					//if (!this->point_map_->empty()) {
					//	this->point_map_->find(new_point->getDevice()->device_id)->second->push_back(new_point);
					//}
					try {
						auto device_vector = this->point_map_->at(new_point->getDevice()->device_id);
						device_vector->insert(device_vector->end(), new_point);
					}
					catch (const std::out_of_range& oor) {
						std::cerr << "Out of Range error: " << oor.what() << '\n';
					}
				}

				void
				clearPointsForDevice(SCDevice* device) {
					//if (this->point_map_ && !this->point_map_->empty()) {
						//typename std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT> *> *>::iterator ret = this->point_map_->find(
						//		device->device_id);
						//if (ret != this->point_map_->end())
							//ret->second->clear();
					//}
					try {
						if (this->point_map_ && !virgin_) {
							auto *vec = this->point_map_->at(device->device_id);
							for (auto it = vec->begin(), end = vec->end(); it != end; ++it) {
								if ((*it))
									delete (*it);
							}
							vec->clear();
						}
					}
					catch (const std::out_of_range& oor) {
						std::cerr << "Out of Range error: " << oor.what() << '\n';
					}
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
				std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>* point_map_ ;  // Saves a combination of device id and all points for that device belonging to this leaf node (voxel)
				// = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>()
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
					for (VoxelList<LeafNode>* list : device_voxel_map_) {
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
						this->addPoint(new OctreeMultiPointCloudPointWrapper<PointT>(&points->at(i), device, cloud));
						//return;
					}
					end = std::chrono::steady_clock::now();
					std::cout << "Inserting new points took "
							  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
							  << " ms" << std::endl;
					delete cloud_copy;
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
