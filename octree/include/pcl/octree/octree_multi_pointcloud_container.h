//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H
#define PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H

#include <pcl/octree/octree_multi_pointcloud_wrapper.h>
#include <pcl/octree/octree_point_list.h>
//#include <eigen3/Eigen/Core>

namespace pcl {
	namespace octree {

		/** \brief @b Octree multi-pointcloud leaf node class
		  * \note This class implements a leaf node that calculates the mean centroid of all points from multiple input point clouds added to this octree container.
		  * \author Alexander Poeppel (poeppel@isse.de)
		  */
		template<typename PointT>
		class OctreeMultiPointCloudContainer : public OctreeContainerBase {

		public:
			static uint64_t constructed, destructed, reused;
			/** \brief Class initialization. */
			OctreeMultiPointCloudContainer () {
				++OctreeMultiPointCloudContainer::constructed;
				//std::cout << "OctreeMultiPointCloudContainer CONstructor called!" << std::endl;
				this->reset();
				//this->point_map_ = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>();
				if (this->point_map_ == nullptr)
					this->point_map_ = new std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>();

			}

			/** \brief Class deconstructor. */
			~OctreeMultiPointCloudContainer () {
				++OctreeMultiPointCloudContainer::destructed;
				//std::cout << "~OctreeMultiPointCloudContainer DEstructor called!" << std::endl;

				/*
				 * // std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>* point_map_
				for (auto dev : this->point_map_) {
					// OctreeMultiPointCloudPointWrapper<PointT> point
					for (auto point : dev) {
						delete (&point);
					}
					dev->clear();
					if (dev != nullptr)
						delete dev;
				 * */

				// std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>* point_map_
				for (auto dev : *this->point_map_) {
					if (dev != nullptr) {
						// PointList<OctreeMultiPointCloudPointWrapper<PointT>>* dev
						// OctreeMultiPointCloudPointWrapper<PointT> point
						/*for (OctreeMultiPointCloudPointWrapper<PointT>* point : *dev) {
						//for (auto point = (*dev)->begin(), end2 = (*dev)->end(); point != end2; ++point) {
							if (point != nullptr) {
								//std::cout << "Deleting point" << point->getPoint() << std::endl;
								delete point;
							}
						}*/
						#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
							dev->clear([](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { delete wrapper; });
						#else
							dev->clear([](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { OctreeMultiPointCloudPointWrapper<PointT>::returnUsedObject(wrapper); });
						#endif
						//delete dev;
					}
				}
				/*if (this->point_map_ != nullptr) {
					delete this->point_map_;
					this->point_map_ = nullptr;
				}*/
			}

			void
			registerDevices(std::set<SCDevice*> *devices, int segments) {
				if (!virgin_)
					return;

				//if (!this->point_map_)
				//this->point_map_ = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>;
				//this->point_map_ = new std::map<int, std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>;
				// std::map<SCDevice*, std::vector<PointT*>> this->point_map_;
				for (auto it = devices->begin(), end = devices->end(); it != end; ++it) {
					auto point_list = new PointList<OctreeMultiPointCloudPointWrapper<PointT>>(segments);
					//point_vector->reserve((*it)->point_count / 1000);
					this->point_map_->insert(this->point_map_->end(), point_list);
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
			  * \param[in] point_cloud the point cloud of origin for the inserted point
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

				//point_sum_ += *(new_point->getPoint());

				//if (!this->point_map_->empty()) {
				//	this->point_map_->find(new_point->getDevice()->device_id)->second->push_back(new_point);
				//}
				try {
					auto device_vector = this->point_map_->at(new_point->getDevice()->device_id);
					//device_vector->insert(device_vector->end(), new_point);
					device_vector->insert(new_point);
					++point_counter_;
				}
				catch (const std::out_of_range& oor) {
					std::cerr << "Out of Range error: " << oor.what() << '\n';
				}
			}

			/** \brief Add new point to voxel.
			  * \param[in] new_point the new point to add
			  */
			void
			addPoint (OctreeMultiPointCloudPointWrapper<PointT> *new_point, OctreeListNodePool<OctreeMultiPointCloudPointWrapper<PointT>> &olnp, int poolSegment) {
				virgin_ = false;
				using namespace pcl::common;

				//point_sum_ += *(new_point->getPoint());

				//if (!this->point_map_->empty()) {
				//	this->point_map_->find(new_point->getDevice()->device_id)->second->push_back(new_point);
				//}
				try {
					auto device_vector = this->point_map_->at(new_point->getDevice()->device_id);
					//device_vector->insert(device_vector->end(), new_point);
					#ifndef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
						device_vector->insert(new_point);
					#else
						device_vector->insert(new_point, olnp, poolSegment);
					#endif
					++point_counter_;
				}
				catch (const std::out_of_range& oor) {
					std::cerr << "Out of Range error: " << oor.what() << '\n';
				}
			}

			void
			clearPointsForDevice(SCDevice* device) {
				PointList<OctreeMultiPointCloudPointWrapper<PointT>> *list = this->point_map_->at(device->device_id);
				//uint64_t  size = list->size();
				#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
					list->clear([](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { delete wrapper; });
				#else
					list->clear([](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { OctreeMultiPointCloudPointWrapper<PointT>::returnUsedObject(wrapper); });
				#endif
				//list->clear();
				//this->point_counter_ -= this->addedSize();
				//if (this->point_map_ && !this->point_map_->empty()) {
				//typename std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT> *> *>::iterator ret = this->point_map_->find(
				//		device->device_id);
				//if (ret != this->point_map_->end())
				//ret->second->clear();
				//}
				/*try {
					if (this->point_map_ && !this->virgin_) {
						auto *vec = this->point_map_->at(device->device_id);
						if (vec != nullptr) {
							for (auto it = vec->begin(), end = vec->end(); it != end; ++it) {
								if ((*it) != nullptr)
									delete (*it);
							}
							vec->clear();
						}
					}
				}
				catch (const std::out_of_range& oor) {
					std::cerr << "Out of Range error: " << oor.what() << '\n';
				}*/
			}

			void
			clearPointsForDevice(SCDevice* device, OctreeListNodePool<OctreeMultiPointCloudPointWrapper<PointT>> &olnp) {
				PointList<OctreeMultiPointCloudPointWrapper<PointT>> *list = this->point_map_->at(device->device_id);
				//uint64_t  size = list->size();
				#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
					list->clear([](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { delete wrapper; });
				#else
					list->clear(olnp);
				#endif
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
				PointT point_sum(0,0,0);
				float weight_sum = 0.0;
				// std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>* point_map_  = nullptr;
				for (auto devices = this->point_map_->begin(), devices_end = this->point_map_->end(); devices != devices_end; ++devices) {
					for (auto points = (*devices)->begin(), end = (*devices)->end(); points != end; ++points) {
						OctreeMultiPointCloudPointWrapper<PointT>* point = (*points);
						float acc = point->getDevice()->accuracy;
						point_sum.x += point->getPoint()->x * acc;
						point_sum.y += point->getPoint()->y * acc;
						point_sum.z += point->getPoint()->z * acc;
						weight_sum += acc;
					}
				}

				centroid_arg.x = point_sum.x / weight_sum;
				centroid_arg.y = point_sum.y / weight_sum;
				centroid_arg.z = point_sum.z / weight_sum;

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
				++OctreeMultiPointCloudContainer::reused;

				point_counter_ = 0;
				//point_sum_ *= 0.0f;

				/*if (this->point_map_ != nullptr) {
					for (auto dev = this->point_map_->begin(), end = this->point_map_->end(); dev != end; ++dev) {
						for (auto points = (*dev)->begin(), end = (*dev)->end(); points != end; ++points) {
							if ((*points) != nullptr)
								delete (*points);
						}
					}
					//delete this->point_map_;
				}*/

			}

			unsigned int
			size() {
				return point_counter_;
			}

			unsigned int
			addedSize() {
				unsigned int ret = 0;
				for (auto devices = this->point_map_->begin(), devices_end = this->point_map_->end(); devices != devices_end; ++devices) {
					ret += (*devices)->size();
				}
				return ret;
			}

			void
			setKey(OctreeKey const& key) {
				this->key_ = key;
			}

			OctreeKey const&
			getKey() {
				return this->key_;
			}

			bool
			isVirgin() {
				return virgin_;
			}

		private:
			unsigned int point_counter_ = 0;
			//PointT point_sum_;
			//std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>* point_map_  = nullptr;  // Saves a combination of device id and all points for that device belonging to this leaf node (voxel)
			std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>* point_map_  = nullptr;  // Saves a combination of device id and all points for that device belonging to this leaf node (voxel)
			// = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>()
			bool virgin_ = true;
			OctreeKey key_;



		};

		template<typename PointT>
		uint64_t OctreeMultiPointCloudContainer<PointT>::constructed = 0;

		template<typename PointT>
		uint64_t OctreeMultiPointCloudContainer<PointT>::destructed = 0;

		template<typename PointT>
		uint64_t OctreeMultiPointCloudContainer<PointT>::reused = 0;

	}
}

#endif //PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H
