//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H
#define PCL_OCTREE_MULTI_POINTCLOUD_CONTAINER_H

#include <pcl/octree/octree_multi_pointcloud_wrapper.h>
#include <pcl/octree/octree_point_list.h>
#include <pcl/octree/octree_thread_pool.h>
#include <pcl/octree/octree_multithreaded_list.h>

#include <atomic>

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
				OctreeMultiPointCloudContainer::constructed++;
				this->reset();
			}

			/** \brief Class deconstructor. */
			~OctreeMultiPointCloudContainer () {
				OctreeMultiPointCloudContainer::destructed++;
				for (auto dev : point_map_) {
					//for (auto seg : dev) {
					#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
						dev->clear([](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { delete wrapper; });
					#else
						dev->clear([](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { OctreeMultiPointCloudPointWrapper<PointT>::returnUsedObject(wrapper); });
					#endif
					//}
				}
			}

			void
			registerDevices(std::set<SCDevice*> *devices, int segments = 1) {
				if (!this->virgin_) {
					return;
				}
				if (wrapper_manager_ == nullptr) {
					throw "No memory manager provided!";
				}
				for (auto it = devices->begin(), end = devices->end(); it != end; it++) {
					/*auto vec = new std::vector<PointList<OctreeMultiPointCloudPointWrapper<PointT>>*>();
					for (int i=0; i<segments; i++) {
						auto point_list = new PointList<OctreeMultiPointCloudPointWrapper<PointT>>(wrapper_manager_);
						vec->push_back(point_list);
					}
					point_map_.push_back(*vec);*/
					point_map_.push_back(new OctreeMultiThreadedList<OctreeMultiPointCloudPointWrapper<PointT>>(wrapper_manager_, segments));
				}
				segments_ = segments;
			}

			void setListNodeManager(OctreeListNodeManager<OctreeMultiPointCloudPointWrapper<PointT>> *wrapper_manager) {
				if (!virgin_) {
					return;
				}
				wrapper_manager_ = wrapper_manager;
			};

			/** \brief deep copy function */
			virtual OctreeMultiPointCloudContainer *
			deepCopy () const {
				return (new OctreeMultiPointCloudContainer (*this));
			}

			/** \brief Equal comparison operator - set to false
			 *  \param[in] OctreeMultiPointCloudContainer to compare with
			 */
			bool operator==(const OctreeContainerBase&) const override {
				return ( false );
			}

			/** \brief Add new point to voxel.
			  * \param[in] new_point the new point to add
			  */
			void
			addPoint (PointT *new_point) {
				// TODO  Check for object pooling
				addPoint(new OctreeMultiPointCloudPointWrapper<PointT>(new_point));
			}

			/** \brief Add new point to voxel.
			  * \param[in] new_point the new point to add
			  * \param[in] point_cloud the point cloud of origin for the inserted point
			  */
			void
			addPoint (PointT *new_point, PointCloud<PointT> *point_cloud) {
				// TODO  Check for object pooling
				addPoint(new OctreeMultiPointCloudPointWrapper<PointT>(new_point, point_cloud));
			}

			/** \brief Add new point to voxel.
			  * \param[in] new_point the new point to add
			  */
			void
			addPoint (OctreeMultiPointCloudPointWrapper<PointT> *new_point, int poolSegment = 0) {
				virgin_ = false;
				using namespace pcl::common;
				try {
					auto device_vector = point_map_.at(new_point->getDevice()->device_id);
					device_vector->insert(new_point, poolSegment);
					point_counter_++;
				}
				catch (const std::out_of_range& oor) {
					std::cerr << "Out of Range error: " << oor.what() << '\n';
				}
			}

			bool
			deviceNeedsClearing(SCDevice* device, int segment = -1) {
				return !point_map_.at(device->device_id)->isClear(segment);
			}

			void
			clearPointsForDevice(SCDevice* device, int segment = -1) {
				auto list = point_map_.at(device->device_id);

				std::function<void(OctreeMultiPointCloudPointWrapper<PointT>*)> contentFunction;

				#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
					contentFunction = [](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { delete wrapper; };
				#else
					contentFunction = [](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) { OctreeMultiPointCloudPointWrapper<PointT>::returnUsedObject(wrapper); };
				#endif

				if (segment >= 0) {
					list->clearSegment(segment, contentFunction);
				} else {
					list->clear(contentFunction);
				}
			}

			/*void
			clearPointsForDevice(SCDevice* device, OctreeThreadPool *otp) {
				auto list = point_map_.at(device->device_id);

				std::atomic_int finished_containers(0), total_containers(0);

				list->foreach([&finished_containers, &otp, this] (OctreeMultiPointCloudPointWrapper<PointT> *wrapper) {
					otp->enqueue([&finished_containers, &list2, this]() {
						try {
							#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
								list2->clear([this](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) {
									delete wrapper;
								});
							#else
								list2->clear([this](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) {
									OctreeMultiPointCloudPointWrapper<PointT>::returnUsedObject(wrapper);
								});
							#endif
						}
						catch (const std::out_of_range& oor) {
							std::cerr << "Out of Range error: " << oor.what() << '\n';
							return;
						}
						++finished_threads;
					});
				});

				while (finished_threads < segments_) {
					usleep(20);
				}
			}*/

			// TODO  Multi-Threaded delete
			/*void
			clearPointsForDevice(SCDevice* device, OctreeThreadPool *otp) {
				auto list = point_map_.at(device->device_id);

				std::atomic_int finished_threads(0);

				list->foreachList([&finished_threads, &otp, this] (OctreeList<OctreeMultiPointCloudPointWrapper<PointT>> *list2) {
					otp->enqueue([&finished_threads, &list2, this]() {
						try {
							#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
								list2->clear([this](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) {
									delete wrapper;
								});
							#else
								list2->clear([this](OctreeMultiPointCloudPointWrapper<PointT> *wrapper) {
									OctreeMultiPointCloudPointWrapper<PointT>::returnUsedObject(wrapper);
								});
							#endif
						}
						catch (const std::out_of_range& oor) {
							std::cerr << "Out of Range error: " << oor.what() << '\n';
							return;
						}
						++finished_threads;
					});
				});

				while (finished_threads < segments_) {
					usleep(20);
				}
			}*/


			/** \brief Calculate centroid of voxel.
			  * \param[out] centroid_arg the resultant centroid of the voxel
			  */
			void
			getCentroid (PointT& centroid_arg) const {}

			/** \brief Calculate weighted centroid of voxel.
			  * \param[out] centroid_arg the resultant centroid of the voxel
			  */
			void
			getWeightedCentroid (PointT& centroid_arg) const {
				using namespace pcl::common;
				PointT point_sum(0,0,0);
				float weight_sum = 0.0;
				for (auto devices = point_map_.begin(), devices_end = point_map_.end(); devices != devices_end; ++devices) {
					(*devices)->foreach([&point_sum, &weight_sum] (OctreeMultiPointCloudPointWrapper<PointT> *point) {
						float acc = point->getDevice()->accuracy;
						point_sum.x += point->getPoint()->x * acc;
						point_sum.y += point->getPoint()->y * acc;
						point_sum.z += point->getPoint()->z * acc;
						weight_sum += acc;
					});
					/*for (auto segment = (*devices).begin(), segment_end = (*devices).end(); segment != segment_end; ++segment) {
						for (auto points = (*segment)->begin(), end = (*segment)->end(); points != end; ++points) {
							OctreeMultiPointCloudPointWrapper<PointT> *point = (*points);
							float acc = point->getDevice()->accuracy;
							point_sum.x += point->getPoint()->x * acc;
							point_sum.y += point->getPoint()->y * acc;
							point_sum.z += point->getPoint()->z * acc;
							weight_sum += acc;
						}
					}*/
				}

				centroid_arg.x = point_sum.x / weight_sum;
				centroid_arg.y = point_sum.y / weight_sum;
				centroid_arg.z = point_sum.z / weight_sum;
			}

			/** \brief Reset leaf container. */
			void
			reset () override {
				using namespace pcl::common;
				OctreeMultiPointCloudContainer::reused++;

				point_counter_ = 0;

			}

			unsigned int
			size() {
				return point_counter_;
			}

			unsigned int
			addedSize() {
				unsigned int ret = 0;
				for (auto devices = point_map_.begin(), devices_end = point_map_.end(); devices != devices_end; devices++) {
					for (auto segment = (*devices).begin(), segment_end = (*devices).end(); segment != segment_end; ++segment) {
						ret += (*segment)->size();
					}
				}
				return ret;
			}

			void
			setKey(OctreeKey const& key) {
				key_ = key;
			}

			OctreeKey const&
			getKey() {
				return key_;
			}

			bool
			isVirgin() {
				return virgin_;
			}

		private:
			unsigned int point_counter_ = 0;
			std::vector<OctreeMultiThreadedList<OctreeMultiPointCloudPointWrapper<PointT>>*> point_map_;  // Saves a combination of device id and all points for that device belonging to this leaf node (voxel)
			bool virgin_ = true;
			OctreeKey key_;

			OctreeListNodeManager<OctreeMultiPointCloudPointWrapper<PointT>> *wrapper_manager_;
			int segments_ = 0;

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
