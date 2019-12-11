//
// Created by alex on 19.03.19.
//

#pragma once

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_HPP
#define PCL_OCTREE_MULTI_POINTCLOUD_HPP

#include <pcl/octree/octree_pointcloud.h>
// Octree MultiPointCloud
#include <pcl/octree/octree_multi_pointcloud_container.h>
#include <pcl/octree/octree_multi_pointcloud_wrapper.h>
#include <pcl/octree/octree_multi_pointcloud_device.h>
#include <pcl/octree/octree_multi_pointcloud_branch_container.h>
#include <pcl/octree/octree_thread_pool.h>
#include <pcl/octree/octree_list_node_manager.h>
#include <pcl/octree/octree_multithreaded_list.h>


#include <pcl/common/io.h>
#include <set>
#include <utility>
#include <chrono>
#include <cmath>

#include <cassert>      // assert
#include <cstddef>      // ptrdiff_t
#include <iterator>     // iterator
#include <type_traits>  // remove_cv
#include <utility>      // swap

//#include </usr/local/cuda-10.2/include/cuda.h>



namespace pcl {
	namespace octree {


		struct voxelCalc {
			float x;
			float y;
			float z;

			float weight;
			int count;
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
				typename LeafContainerT = OctreeMultiPointCloudContainer<PointT>,
				//typename BranchContainerT = OctreeContainerEmpty >
				typename BranchContainerT = OctreeMultiPointCloudBranchContainer>
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
			OctreeMultiPointCloud (const double resolution_arg, const double extent, int threads, OCTREE_MEMORY_STRATEGY strategy = STD_ALLOCATOR) :
					OctreePointCloud<PointT, LeafContainerT, BranchContainerT> (resolution_arg), wrapper_manager_(threads, strategy), container_manager_(threads, strategy), otp(threads) {
				num_hw_threads = threads;
				memory_strategy_ = strategy;
				extent_ = extent;
				resolution_ = resolution_arg;

				max_idx_ = std::ceil(extent / resolution_arg);
				center_idx_ = max_idx_ / 2;
				voxel_count_ = max_idx_ * max_idx_ * max_idx_;

				octree = static_cast<voxelCalc**>( std::malloc(voxel_count_ * sizeof(voxelCalc*)) );
				for (int i=0; i<voxel_count_; ++i) {
					octree[i] = new voxelCalc{.0f, .0f, .0f, .0f, 0};
				}
			}

			/** \brief Empty class deconstructor. */

			~OctreeMultiPointCloud () {
				std::cout << "~OctreeMultiPointCloud DEstructor called!" << "\n";

			}

			/** \brief Calculate the resulting merged point cloud
			  * \param[out] result_out the object in which to save the calculated point cloud
			  * \return The size of the resulting point cloud.
			  */
			size_t
			getResultPointCloud(PointCloud<PointT>& result_out) {

				//std::cout << "Generating result PointCloud" << "\n";
				//auto start = std::chrono::steady_clock::now();

				/*
				 *
				 * std::atomic_int finished_delete_threads(0);
						//int finished_threads = 0;
						for (int i=0; i < num_hw_threads; ++i) {
							otp.enqueue([i, &occupied_voxels, &device, &cloud_copy, &finished_delete_threads, this]() {
								occupied_voxels->foreach([i, &device, this](LeafContainerT *container) {
									//if (container->deviceNeedsClearing(device, i)) {
										container->clearPointsForDevice(device, i);
									//}
								});
								++finished_delete_threads;
							});
						}
						while (finished_delete_threads < num_hw_threads) {
							usleep(20);
						}
				*/

				//auto start2 = std::chrono::steady_clock::now();
				result_out.clear();
				result_out.points.resize(max_possible_voxels_ * 0.7);
				/*std::cout << "Memory operations took "
						  << ((float)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start2).count()/1000000)
						  << " ms" << "\n";*/

				/*for (SCDevice *device : registered_devices_) {
					PointCloud<PointXYZL> *pc = current_point_clouds_[device->device_id];
					if (!pc)
						continue;
					for (PointXYZL  &point : pc->points) {
						if (point.label >= voxel_count_ || std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
							continue;
						auto label = point.label;
						float acc = device->accuracy;
						octree[label]->x += point.x * acc;
						octree[label]->y += point.y * acc;
						octree[label]->z += point.z * acc;
						octree[label]->weight += acc;
						++octree[label]->count;
					}
				}*/


				for (SCDevice *device : registered_devices_) {
					std::atomic_int finished_calcpoints_threads(0);


					//std::cout << "Working with " << num_hw_threads << " processing threads..." << "\n";

					for (int i=0; i < num_hw_threads; ++i) {
						PointCloud<PointXYZL> *pc = current_point_clouds_[device->device_id];
						if (!pc) {
							//std::cout << "getResultPointCloud(): Skipping empty PointCloud!!" << "\n";
							++finished_calcpoints_threads;
							continue;
						}

						size_t segment_size = pc->points.size() / num_hw_threads;

						size_t thread_start = i * segment_size;
						size_t thread_end;

						if (i == num_hw_threads - 1) {
							thread_end = pc->points.size();
						} else {
							thread_end = (i + 1) * segment_size;
						}

						//std::cout << "Pushing work for segment #" << i << "\n";

						otp.enqueue([i, &pc, &device, thread_start, thread_end, &finished_calcpoints_threads, this]() {
							//auto start2 = std::chrono::steady_clock::now();

							//std::cout << "Thread #" << i << " starting... Working on indices " << thread_start << " to " << thread_end << "\n";

							for (int n=thread_start; n<thread_end; n++) {
								auto point = pc->points[n];
								if (point.label >= voxel_count_ || std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
									continue;
								auto label = point.label;
								float acc = device->accuracy;
								this->octree[label]->x += point.x * acc;
								this->octree[label]->y += point.y * acc;
								this->octree[label]->z += point.z * acc;
								this->octree[label]->weight += acc;
								++this->octree[label]->count;
							}

							++finished_calcpoints_threads;

							/*std::cout << "Thread # " << i << " took "
									  << ((float)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start2).count()/1000)
									  << " ms" << "\n";*/
							//std::cout << "Thread #" << i << " finished!" << "\n";
						});
					}
					while (finished_calcpoints_threads < num_hw_threads) {
						usleep(20);
					}
					//std::cout << "All threads finished!" << "\n";
				}

				///////

				PointT temp;
				size_t count = 0;

				for (int i=0; i<voxel_count_; ++i) {
					if (octree[i]->count == 0)
						continue;

					++count;
					temp.x = octree[i]->x / octree[i]->weight;
					temp.y = octree[i]->y / octree[i]->weight;
					temp.z = octree[i]->z / octree[i]->weight;

					octree[i]->x = .0f;
					octree[i]->y = .0f;
					octree[i]->z = .0f;
					octree[i]->weight = .0f;
					octree[i]->count = 0;

					result_out.push_back(temp);
				}

				/*std::cout << "Generating PointCloud took "
						  << ((float)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count()/1000000)
						  << " ms" << "\n";*/

				return count;
			}

			/*__global__
			void test () {

			}*/

			//std::mutex octree_lock;

			/** \brief Add new point cloud to voxel.
			  * \param[in] device the device for which the new point cloud is added
			  * \param[in] cloud the new point cloud
			  */
			void
			addPointCloud (SCDevice *device, PointCloud<PointT> *cloud) {
				if (!cloud) {
					//std::cout << "addPointCloud(): Skipping empty PointCloud!!" << "\n";
					return;
				}

				//std::cout << "Adding PointCloud for device '" << device->identifier << "' of type '" << device->type << "'" << "\n";
				//auto start = std::chrono::steady_clock::now();

				// TODO  Handle this differently for multithreading
				if (current_point_clouds_[device->device_id])
					delete current_point_clouds_[device->device_id];

				auto *temp = new PointCloud<PointXYZL>();
				// TODO  investigate why this doesn't work
				//copyPointCloud(cloud, temp);

				temp->points.resize(cloud->points.size());
				/*for (size_t i = 0; i < cloud->points.size(); i++) {
					auto p = &(temp->points[i]);

					if (p->x != p->x || p->y != p->y || p->z != p->z) {
						continue;
					}

					p->x = cloud->points[i].x;
					p->y = cloud->points[i].y;
					p->z = cloud->points[i].z;

					int x_idx = center_idx_ + (sgn(p->x) * std::abs(p->x / resolution_));
					int y_idx = center_idx_ + (sgn(p->y) * std::abs(p->y / resolution_));
					int z_idx = center_idx_ + (sgn(p->z) * std::abs(p->z / resolution_));

					p->label = arrayOffset(x_idx, y_idx, z_idx);
				}*/

				std::atomic_int finished_process_cloud_threads(0);


				//std::cout << "Working with " << num_hw_threads << " processing threads..." << "\n";

				for (int i=0; i < num_hw_threads; ++i) {

					size_t segment_size = cloud->size() / num_hw_threads;

					size_t thread_start = i * segment_size;
					size_t thread_end;

					if (i == num_hw_threads - 1) {
						thread_end = cloud->size();
					} else {
						thread_end = (i + 1) * segment_size;
					}

					//std::cout << "Pushing work for segment #" << i << "\n";

					otp.enqueue([i, &temp, &cloud, thread_start, thread_end, &finished_process_cloud_threads, this]() {
						//auto start2 = std::chrono::steady_clock::now();

						for (size_t i = thread_start; i < thread_end; i++) {
							auto p = &(temp->points[i]);

							if (p->x != p->x || p->y != p->y || p->z != p->z) {
								continue;
							}

							p->x = cloud->points[i].x;
							p->y = cloud->points[i].y;
							p->z = cloud->points[i].z;

							int x_idx = center_idx_ + (sgn(p->x) * std::abs(p->x / resolution_));
							int y_idx = center_idx_ + (sgn(p->y) * std::abs(p->y / resolution_));
							int z_idx = center_idx_ + (sgn(p->z) * std::abs(p->z / resolution_));

							p->label = arrayOffset(x_idx, y_idx, z_idx);
						}

						++finished_process_cloud_threads;

						/*std::cout << "Thread # " << i << " took "
								  << ((float)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start2).count()/1000)
								  << " ms" << "\n";*/
					});
				}
				while (finished_process_cloud_threads < num_hw_threads) {
					usleep(20);
				}

				current_point_clouds_[device->device_id] = temp;

				/*std::cout << "Processing new points took "
						  << ((float)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count()/1000000)
						  << " ms" << "\n";*/
			}


			/** \brief Add DataT object to leaf node at octree key.
			  * \param pointIdx_arg
			  */
			void
			addPointIdx (const int) override {
				return;
			}

			/** \brief Register a new device with the multi point cloud octree. Cannot be called after point cloud streaming has started.
			  * \param[in] device the device to register
			  * \return Success of registration.
			  */
			bool
			registerDevice(SCDevice* device) {
				if (!running) {
					bool ret = registered_devices_.insert(device).second;
					if (ret) {
						current_point_clouds_.push_back(nullptr);

					}
					max_possible_voxels_ += device->point_count;
				}
				return false;
			}

			int added_points;

			int
			arrayOffset(int x, int y, int z) {
				return (z * max_idx_ * max_idx_) + (y * max_idx_) + x;
			}

			template <typename T> int sgn(T val) {
				return (T(0) < val) - (val < T(0));
			}


		private:

			std::set<SCDevice*> registered_devices_;  // Saves list of all currently registered devices
			std::vector<PointCloud<PointXYZL>*> current_point_clouds_;
			//std::vector<std::vector<PointList<LeafContainerT>*>> device_voxel_map_;   // Saves combination of device id and all occupied voxels per thread
			std::vector<OctreeMultiThreadedList<LeafContainerT>*> device_voxel_map_;   // Saves combination of device id and all occupied voxels per thread
			bool running = false;

			OctreeThreadPool otp;
			OctreeListNodeManager<OctreeMultiPointCloudPointWrapper<PointT>> wrapper_manager_;
			OctreeListNodeManager<LeafContainerT> container_manager_;

			OCTREE_MEMORY_STRATEGY memory_strategy_ = STD_ALLOCATOR;
			int num_hw_threads;
			double extent_, resolution_;
			int max_idx_, center_idx_, voxel_count_, max_possible_voxels_;

			std::mutex parallel_section_lock;

			voxelCalc** octree;

		};
	}
}

// TODO Evaluate Notice: Note: Don't precompile this octree type to speed up compilation. It's probably rarely used.
//#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>

#endif  // PCL_OCTREE_MULTI_POINTCLOUD_HPP