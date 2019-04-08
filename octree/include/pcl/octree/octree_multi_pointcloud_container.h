//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H
#define PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H

#include <pcl/octree/octree_multi_pointcloud_wrapper.h>
#include <pcl/octree/octree_point_list.h>

namespace pcl {
	namespace octree {

		/** \brief @b Octree multi-pointcloud leaf node class
		  * \note This class implements a leaf node that calculates the mean centroid of all points from multiple input point clouds added to this octree container.
		  * \author Alexander Poeppel (poeppel@isse.de)
		  */
		template<typename PointT>
		class OctreeMultiPointCloudContainer : public OctreeContainerBase {

		public:
			/** \brief Class initialization. */
			OctreeMultiPointCloudContainer () {
				//std::cout << "OctreeMultiPointCloudContainer CONstructor called!" << std::endl;
				this->reset();
				//this->point_map_ = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>();
				this->point_map_ = new std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>();

			}

			/** \brief Class deconstructor. */
			~OctreeMultiPointCloudContainer () {
				//std::cout << "~OctreeMultiPointCloudContainer DEstructor called!" << std::endl;
				for (auto dev = this->point_map_->begin(), end = this->point_map_->end(); dev != end; ++dev) {
					(*dev)->clear();
					/*for (auto points = (*dev)->begin(), end = (*dev)->end(); points != end; ++points) {
						if ((*points) != nullptr)
							delete (*points);
					}
					if ((*dev) != nullptr)
						delete (*dev);*/
				}
				/*if (this->point_map_ != nullptr) {
					delete this->point_map_;
					this->point_map_ = nullptr;
				}*/
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
					auto point_list = new PointList<OctreeMultiPointCloudPointWrapper<PointT>>;
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
					//device_vector->insert(device_vector->end(), new_point);
					device_vector->insert(new_point);
				}
				catch (const std::out_of_range& oor) {
					std::cerr << "Out of Range error: " << oor.what() << '\n';
				}
			}

			void
			clearPointsForDevice(SCDevice* device) {
				this->point_map_->at(device->device_id)->clear();
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

				/*if (this->point_map_ != nullptr) {
					for (auto dev = this->point_map_->begin(), end = this->point_map_->end(); dev != end; ++dev) {
						for (auto points = (*dev)->begin(), end = (*dev)->end(); points != end; ++points) {
							if ((*points) != nullptr)
								delete (*points);
						}
					}
					delete this->point_map_;
				}*/

			}

			bool
			isVirgin() {
				return virgin_;
			}

		private:
			unsigned int point_counter_;
			PointT point_sum_;
			//std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>* point_map_  = nullptr;  // Saves a combination of device id and all points for that device belonging to this leaf node (voxel)
			std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>* point_map_  = nullptr;  // Saves a combination of device id and all points for that device belonging to this leaf node (voxel)
			// = new std::vector<std::vector<OctreeMultiPointCloudPointWrapper<PointT>*>*>()
			bool virgin_ = true;



		};

	}
}

#endif //PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H
