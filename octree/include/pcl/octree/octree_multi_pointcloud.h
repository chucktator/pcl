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
#include <pcl/octree/octree_multi_pointcloud_branch_container.h>
#include <pcl/octree/octree_thread_pool.h>
#include <pcl/octree/octree_list_node_manager.h>
#include <pcl/octree/octree_multithreaded_list.h>



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
				OctreeMultiPointCloud (const double resolution_arg, int threads, OCTREE_MEMORY_STRATEGY strategy = STD_ALLOCATOR) :
						OctreePointCloud<PointT, LeafContainerT, BranchContainerT> (resolution_arg), wrapper_manager_(threads, strategy), container_manager_(threads, strategy), otp(threads) {
					num_hw_threads = threads;
					memory_strategy_ = strategy;

					#ifdef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
						OctreeMultiPointCloudPointWrapper<PointT>::initPool(memory_strategy_, num_hw_threads);
					#endif
					#ifdef OCTREE_MULTI_POINTCLOUD_THREADING
						std::cout << "Working with " << num_hw_threads << " processing threads." << std::endl;

					#endif
				}

				/** \brief Empty class deconstructor. */

				~OctreeMultiPointCloud () {
					std::cout << "~OctreeMultiPointCloud DEstructor called!" << std::endl;

				}

				/** \brief Calculate the resulting merged point cloud
				  * \param[out] result_out the object in which to save the calculated point cloud
				  * \return The size of the resulting ppoint cloud.
				  */
				size_t
				getResultPointCloud(PointCloud<PointT>& result_out) {
					std::cout << "This PointCloud should contain " << this->getLeafCount() << " points." << std::endl;
					result_out.clear();
					for (auto leaves = this->leaf_breadth_begin(), leaves_end = this->leaf_breadth_end(); leaves != leaves_end; ++leaves) {
						PointT temp;
						auto leaf = ((LeafNode*)(*leaves))->getContainerPtr();
						leaf->getWeightedCentroid(temp);
						if (isnan(temp.x) || isnan(temp.y) || isnan(temp.z)) {
							continue;
						}
						result_out.push_back(temp);
					}
					return this->leaf_count_;
				}

				std::mutex octree_lock;

				/** \brief Add new point cloud to voxel.
				  * \param[in] device the device for which the new point cloud is added
				  * \param[in] cloud the new point cloud
				  */
				void
				addPointCloud (SCDevice *device, PointCloud<PointT> *cloud) {
					parallel_section_lock.lock();

					// Copy PointCloud for concurrency safety
					auto *cloud_copy = new pcl::PointCloud<PointT>();
					pcl::copyPointCloud(*cloud, *cloud_copy);

					// Don't allow the addition of new devices after insertion of points has begun
					running = true;

					std::cout << "Adding PointCloud for device '" << device->identifier << "' of type '" << device->type << "'" << std::endl;

					auto start = std::chrono::steady_clock::now();

					// Remove all old points for the given device first
					OctreeMultiThreadedList<LeafContainerT> *occupied_voxels;
					try {
						 occupied_voxels = device_voxel_map_.at(device->device_id);
					}
					catch (const std::out_of_range& oor) {
						std::cerr << "Out of Range error: " << oor.what() << '\n';
						return;
					}

					// Iterate over voxels and
					// 		A) register all devices, if the voxel is fresh
					//		B) clear it of all points for this device if it isn't
					#ifdef OCTREE_MULTI_POINTCLOUD_POOL_STATISTICS
						std::cout << "###############################################################" << std::endl;
						std::cout << "Current stats on the MPWrapper pool:" << std::endl;
						printWrapperPoolStats();
					#endif

					/*for (auto list : occupied_voxels) {
						for (auto item : *list) {
							OctreeMultiPointCloudContainer<PointT>* multi_container = item;
							if (multi_container->isVirgin ()) {
								multi_container->registerDevices(&registered_devices_, num_hw_threads);
							} else {
								#ifdef OCTREE_MULTI_POINTCLOUD_MULTITHREADED_DELETE
									multi_container->clearPointsForDevice(device, &otp);
								#else
									multi_container->clearPointsForDevice(device);
								#endif
							}
						}

					}*/
//#define OCTREE_MULTI_POINTCLOUD_MULTITHREADED_DELETE
					#ifndef OCTREE_MULTI_POINTCLOUD_MULTITHREADED_DELETE
						occupied_voxels->foreach([this, device](LeafContainerT *container) {
							/*if (container->isVirgin ()) {
								container->registerDevices(&registered_devices_, num_hw_threads);
							} else {*/
								container->clearPointsForDevice(device);
							//}
						});
					#else
						/*std::atomic_int finished_delete_threads(0);
						//int finished_threads = 0;
						occupied_voxels->foreachList([this, &finished_delete_threads, &device](OctreeList<LeafContainerT> *list) {
							otp.enqueue([this, &finished_delete_threads, list, &device]() {
								for (auto container : *list) {
									if (container->deviceNeedsClearing(device)) {
										if (container->isVirgin()) {
											container->registerDevices(&registered_devices_, num_hw_threads);
										} else {
											container->clearPointsForDevice(device);
										}
									}
								}
								++finished_delete_threads;
							});
						});
						while (finished_delete_threads < num_hw_threads) {
							usleep(20);
						}*/
						std::atomic_int finished_delete_threads(0);
						//int finished_threads = 0;
						for (int i=0; i < num_hw_threads; ++i) {
							otp.enqueue([i, &occupied_voxels, &device, &cloud_copy, &finished_delete_threads, this]() {
								occupied_voxels->foreach([i, &device, this](LeafContainerT *container) {
									if (container->deviceNeedsClearing(device, i)) {
										container->clearPointsForDevice(device, i);
									}
								});
								++finished_delete_threads;
							});
						}
						while (finished_delete_threads < num_hw_threads) {
							usleep(20);
						}
						/*std::atomic_int finished_containers(0), total_containers(0);

						occupied_voxels->foreach([this, &finished_containers, &total_containers, &device](LeafContainerT *container) {
							if (container->deviceNeedsClearing(device)) {
								otp.enqueue([this, &finished_containers, &total_containers, container, &device]() {
									if (container->isVirgin()) {
										container->registerDevices(&registered_devices_, num_hw_threads);
									} else {
										container->clearPointsForDevice(device);
									}

									++finished_containers;
									//std::cout << "Thread " << i << " finished in " << finished_threads << " place." << std::endl;
								});
								++total_containers;
							}
						});
						while (finished_containers < total_containers) {
							usleep(20);
						}*/
					#endif

					#ifdef OCTREE_MULTI_POINTCLOUD_POOL_STATISTICS
						std::cout << "----------------------------------------------------------------" << std::endl;
						std::cout << "After clearing the pool:" << std::endl;
						printWrapperPoolStats();

						//delete occupied_voxels
						std::cout << "###############################################################" << std::endl;
						std::cout << "Current stats on the MPContainer pool:" << std::endl;
						printContainerPoolStats();
					#endif

					//octree_lock.lock();
					/*for (auto list : occupied_voxels) {
						list->clear();
					}*/
					occupied_voxels->clear();
					//octree_lock.unlock();

					#ifdef OCTREE_MULTI_POINTCLOUD_POOL_STATISTICS
						std::cout << "----------------------------------------------------------------" << std::endl;
						std::cout << "After clearing the pool:" << std::endl;
						printContainerPoolStats();

						std::cout << "Actual octree contains " << leaf_count_ << " at this point." << std::endl;
						std::cout << "OctreeMultiPointCloudContainer objects:" << std::endl;
						std::cout << OctreeMultiPointCloudContainer<PointXYZ>::constructed << " objects created." <<std::endl;
						std::cout << OctreeMultiPointCloudContainer<PointXYZ>::destructed << " objects destroyed." <<std::endl;
						std::cout << OctreeMultiPointCloudContainer<PointXYZ>::reused << " objects reused." <<std::endl;
					#endif


					auto end = std::chrono::steady_clock::now();

					std::cout << "Deleting old points took "
						 << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
						 << " ms" << std::endl;

					start = std::chrono::steady_clock::now();
					// Then insert the point cloud for the given device
					auto *points = &cloud_copy->points;

					/*#ifndef OCTREE_MULTI_POINTCLOUD_THREADING
						for (int i = 0; i < points->size(); i++) {
							PointT point = points->at(i);
							if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
								continue;
							}
							#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
								auto new_point = new OctreeMultiPointCloudPointWrapper<PointT>();
							#else
								auto new_point = OctreeMultiPointCloudPointWrapper<PointT>::getFreeObject();
							#endif
							new_point->init(&(points->at(i)), device, cloud_copy);
							LeafContainerT *voxel = addPoint(new_point);
						}
					#else*/
					/*std::cout << "###################################################################" << std::endl;
					std::cout << "ENTERING PARALLEL POINT ADDER" << std::endl;
					std::cout << "###################################################################" << std::endl;*/
					int segment = points->size() / num_hw_threads;
					std::atomic_int finished_threads(0);
					//int finished_threads = 0;
					for (int i=0; i < num_hw_threads; ++i) {
						int thread_start = i * segment;
						int thread_end;
						if (i == num_hw_threads - 1) {
							thread_end = points->size();
						} else {
							thread_end = (i + 1) * segment;
						}
						//workers.push_back(std::thread([thread_start, thread_end, &points, device, cloud_copy, this]() {
						otp.enqueue([i, thread_start, thread_end, &points, &device, &cloud_copy, &finished_threads, this]() {

							for (int p = thread_start; p < thread_end; ++p) {
								PointT point = points->at(p);
								if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
									continue;
								}
								#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
									auto new_point = new OctreeMultiPointCloudPointWrapper<PointT>();
								#else
									auto new_point = OctreeMultiPointCloudPointWrapper<PointT>::getFreeObject(i);
								#endif
								new_point->init(&(points->at(p)), device, cloud_copy);
								LeafContainerT *voxel = addPoint(new_point, i);
							}
							++finished_threads;
							//std::cout << "Thread " << i << " finished in " << finished_threads << " place." << std::endl;
						});
					}
					while (finished_threads < num_hw_threads) {
						usleep(20);
					}
					/*std::cout << "###################################################################" << std::endl;
					std::cout << "COMPLETED PARALLEL POINT ADDER" << std::endl;
					std::cout << "###################################################################" << std::endl;*/

					//#endif
					end = std::chrono::steady_clock::now();
					std::cout << "Inserting new points took "
							  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
							  << " ms" << std::endl;

					// Save reference to the origin PointCloud for later safe deletion
					PointCloud<PointT> *temp = current_point_clouds_[device->device_id];
					if (temp != nullptr) {
						//temp->clear();
						delete temp;
					}
					current_point_clouds_[device->device_id] = cloud_copy;
					parallel_section_lock.unlock();
				}

				void
				printContainerPoolStats() {
					std::cout << OctreeListNode<LeafContainerT>::free_size << " objects free" << std::endl;
					std::cout << OctreeListNode<LeafContainerT>::used_size << " objects used" << std::endl;
					std::cout << OctreeListNode<LeafContainerT>::requested_objects << " objects requested thus far" << std::endl;
					std::cout << OctreeListNode<LeafContainerT>::returned_objects << " objects returned thus far" << std::endl;
				}

				void
				printWrapperPoolStats() {
					std::cout << OctreeListNode<OctreeMultiPointCloudPointWrapper<PointT>>::free_size << " objects free" << std::endl;
					std::cout << OctreeListNode<OctreeMultiPointCloudPointWrapper<PointT>>::used_size << " objects used" << std::endl;
					std::cout << OctreeListNode<OctreeMultiPointCloudPointWrapper<PointT>>::requested_objects << " objects requested thus far" << std::endl;
					std::cout << OctreeListNode<OctreeMultiPointCloudPointWrapper<PointT>>::returned_objects << " objects returned thus far" << std::endl;
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
							/*auto vec = new std::vector<PointList<LeafContainerT>*>();
							for (int i=0; i<num_hw_threads; i++) {
								vec->push_back(new PointList<LeafContainerT>(&(container_manager_)));
							}
							device_voxel_map_.push_back(*vec);*/
							device_voxel_map_.push_back(new OctreeMultiThreadedList<LeafContainerT>(&container_manager_, num_hw_threads));
							current_point_clouds_.push_back(nullptr);

							#ifdef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
								//OctreeMultiPointCloudPointWrapper<PointT>::initPool(memory_strategy_, num_hw_threads);
								OctreeMultiPointCloudPointWrapper<PointT>::reserveMemory(device->point_count * 2.1);
								//auto test = new OctreeObjectPool<OctreeMultiPointCloudPointWrapper<PointT>>();
							#endif

							// List Node Managers
							wrapper_manager_.reserveMemory(device->point_count * 2.1);
							container_manager_.reserveMemory(device->point_count * 2.1);
						}
					}
					return false;
				}

				int added_points;

				/** \brief Add new point to octree.
				  * \param[in] new_point the new point to add
				  * \return Pointer to the Voxel into which the point was inserted.
				  */
				LeafContainerT*
				addPoint (OctreeMultiPointCloudPointWrapper<PointT>* const& new_point, int poolSegment = 0) {
					running = true;

					OctreeKey key;

					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_BOUNDING_BOX
						octree_lock.lock();
					#endif
					// make sure bounding box is big enough
					this->adoptBoundingBoxToPoint (*(new_point->getPoint()));
					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_BOUNDING_BOX
						octree_lock.unlock();
					#endif

					// generate key
					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_GEN_OCTREE_KEY
						octree_lock.lock();
					#endif
					this->genOctreeKeyforPoint (*(new_point->getPoint()), key);
					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_GEN_OCTREE_KEY
						octree_lock.unlock();
					#endif


					//octree_lock.lock();
					LeafContainerT* container = this->createLeaf(key);

					if (container->isVirgin()) {
						container->setListNodeManager(&(wrapper_manager_));
						container->registerDevices(&registered_devices_, num_hw_threads);
					}

					container->addPoint(new_point, poolSegment);
					container->setKey(key);

					auto temp = device_voxel_map_.at(new_point->getDevice()->device_id);
					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_INSERT
						octree_lock.lock();
					#endif

					temp->insert(container, poolSegment);

					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_INSERT
						octree_lock.unlock();
					#endif
					added_points++;

					return container;
				}

			private:

				std::set<SCDevice*> registered_devices_;  // Saves list of all currently registered devices
				std::vector<PointCloud<PointT>*> current_point_clouds_;
				//std::vector<std::vector<PointList<LeafContainerT>*>> device_voxel_map_;   // Saves combination of device id and all occupied voxels per thread
				std::vector<OctreeMultiThreadedList<LeafContainerT>*> device_voxel_map_;   // Saves combination of device id and all occupied voxels per thread
				bool running = false;

				OctreeThreadPool otp;
				OctreeListNodeManager<OctreeMultiPointCloudPointWrapper<PointT>> wrapper_manager_;
				OctreeListNodeManager<LeafContainerT> container_manager_;

				OCTREE_MEMORY_STRATEGY memory_strategy_ = STD_ALLOCATOR;
				int num_hw_threads;

				std::mutex parallel_section_lock;

		};
	}
}

// TODO Evaluate Notice: Note: Don't precompile this octree type to speed up compilation. It's probably rarely used.
//#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>

#endif  // PCL_OCTREE_MULTI_POINTCLOUD_HPP