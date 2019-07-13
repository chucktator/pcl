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



#include <pcl/common/io.h>
#include <set>
#include <utility>
#include <chrono>

#include <cassert>      // assert
#include <cstddef>      // ptrdiff_t
#include <iterator>     // iterator
#include <type_traits>  // remove_cv
#include <utility>      // swap

#include <thread>

// Thread pool
#include <memory>
#include <boost/asio.hpp>
#include <boost/thread.hpp>


#ifdef OCTREE_MULTI_POINTCLOUD_THREADING
	int num_hw_threads = OCTREE_MULTI_POINTCLOUD_NUM_THREADS;
#else
	int num_hw_threads = 1;
#endif


#define OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING


namespace pcl {
	namespace octree {

		struct OctreeThreadPool {
			typedef std::unique_ptr<boost::asio::io_service::work> asio_worker;

			OctreeThreadPool(int threads) :service(), service_worker(new asio_worker::element_type(service)) {
				for (int i = 0; i < threads; ++i) {
					auto worker = [this] { return service.run(); };
					grp.add_thread(new boost::thread(worker));
				}
			}

			template<class F>
			void enqueue(F f) {
				service.post(f);
			}

			~OctreeThreadPool() {
				service_worker.reset();
				grp.join_all();
				service.stop();
			}

		private:
			boost::asio::io_service service;
			asio_worker service_worker;
			boost::thread_group grp;
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
				OctreeMultiPointCloud (const double resolution_arg) :
						OctreePointCloud<PointT, LeafContainerT, BranchContainerT> (resolution_arg), olnp(num_hw_threads), olnp_container(num_hw_threads) {
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
						//if (leaf.addedSize() != 0) {
							leaf->getWeightedCentroid(temp);
							if (isnan(temp.x) || isnan(temp.y) || isnan(temp.z)) {
								continue;
							}
							//(*leaves)->getContainer().getWeightedCentroid(temp);
							result_out.push_back(temp);
						//}
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

					// Copy PointCloud for concurrency safety
					auto *cloud_copy = new pcl::PointCloud<PointT>();
					pcl::copyPointCloud(*cloud, *cloud_copy);
					//auto *cloud_copy = cloud;

					// Don't allow the addition of new devices after insertion of points has begun
					this->running = true;

					std::cout << "Adding PointCloud for device '" << device->identifier << "' of type '" << device->type << "'" << std::endl;

					auto start = std::chrono::steady_clock::now();

					// Remove all old points for the given device first
					//PointList<LeafNode> *occupied_voxels;
					PointList<LeafContainerT> *occupied_voxels;
					try {
						 occupied_voxels = this->device_voxel_map_.at(device->device_id);
					}
					catch (const std::out_of_range& oor) {
						std::cerr << "Out of Range error: " << oor.what() << '\n';
						return;
					}

					// Iterate over voxels and
					// 		A) register all devices, if the voxel is fresh
					//		B) clear it of all points for this device if it isn't
					//PointList<OctreeMultiPointCloudContainer<PointT>> deletable_voxels;
					#ifdef OCTREE_MULTI_POINTCLOUD_POOL_STATISTICS
						std::cout << "###############################################################" << std::endl;
						std::cout << "Current stats on the MPWrapper pool:" << std::endl;
						printWrapperPoolStats();
					#endif

					for (auto item : *occupied_voxels) {
						//OctreeMultiPointCloudContainer<PointT> multi_container = item->getContainer();
						OctreeMultiPointCloudContainer<PointT>* multi_container = item;
						if (multi_container->isVirgin ()) {
							multi_container->registerDevices(&registered_devices_, num_hw_threads);
						} else {
							multi_container->clearPointsForDevice(device);
							// Delete now empty voxels
							/*if (multi_container->addedSize() == 0) {
								deletable_voxels.insert(multi_container);
							}*/
						}
					}

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
					#ifndef OCTREE_MULTI_POINTCLOUD_THREADING
						occupied_voxels->clear();
					#else
						occupied_voxels->clear(olnp_container);
					#endif
					//octree_lock.unlock();

					#ifdef OCTREE_MULTI_POINTCLOUD_POOL_STATISTICS
						std::cout << "----------------------------------------------------------------" << std::endl;
						std::cout << "After clearing the pool:" << std::endl;
						printContainerPoolStats();

						std::cout << "Actual octree contains " << this->leaf_count_ << " at this point." << std::endl;
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
					//PointList<OctreeMultiPointCloudContainer<PointT>> new_voxels;
					auto *points = &cloud_copy->points;

					#ifndef OCTREE_MULTI_POINTCLOUD_THREADING
						for (int i = 0; i < points->size(); ++i) {
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
							LeafContainerT *voxel = this->addPoint(new_point);
							//new_voxels.insert(voxel);
							//deletable_voxels.remove(voxel);
						}
					#else
						//std::vector<std::thread> workers;
						int segment = points->size() / num_hw_threads;
						int finished_threads = 0;
						for (int i=0; i < num_hw_threads; i++) {
							int thread_start = i * segment;
							int thread_end;
							if (i == num_hw_threads - 1) {
								thread_end = points->size();
							} else {
								thread_end = (i + 1) * segment;
							}
							// TODO Extend to end in last segment
							//workers.push_back(std::thread([thread_start, thread_end, &points, device, cloud_copy, this]() {
							otp.enqueue([i, thread_start, thread_end, &points, &device, &cloud_copy, &finished_threads, this]() {

								for (int p = thread_start; p < thread_end; p++) {
									PointT point = points->at(p);
									if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
										continue;
									}
									#ifndef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
										auto new_point = new OctreeMultiPointCloudPointWrapper<PointT>();
									#else
										auto new_point = OctreeMultiPointCloudPointWrapper<PointT>::getFreeObject();
									#endif
									new_point->init(&(points->at(p)), device, cloud_copy);
									LeafContainerT *voxel = this->addPoint(new_point, i);
									//new_voxels.insert(voxel);
									//deletable_voxels.remove(voxel);
								}
								finished_threads++;
							});
						}
						while (finished_threads < num_hw_threads) {
							usleep(20);
						}
						/*std::for_each(workers.begin(), workers.end(), [](std::thread &t) {
							t.join();
						});*/

					#endif
					end = std::chrono::steady_clock::now();
					std::cout << "Inserting new points took "
							  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
							  << " ms" << std::endl;

					// Save reference to the origin PointCloud for later safe deletion
					PointCloud<PointT> *temp = current_point_clouds_[device->device_id];
					if (temp != nullptr) {
						temp->clear();
						delete temp;
					}
					current_point_clouds_[device->device_id] = cloud_copy;

					// Delete now empty voxels
					//PointList<OctreeMultiPointCloudContainer<PointT>> deletable_voxels;

					/*for (auto item : *occupied_voxels) {
						if (item->getContainer().addedSize() == 0) {
							deletable_voxels.insert(&(item->getContainer()));
						}
					}*/
					/*for (auto item : deletable_voxels) {
						this->removeLeaf(item->getKey());
					}*/
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
					if (!this->running) {
						bool ret = registered_devices_.insert(device).second;
						if (ret) {
							//device_voxel_map_.insert(std::pair<int, std::set<LeafContainerT*>*>(device->device_id, new std::set<LeafContainerT*>()));
							//device_voxel_map_.insert(device_voxel_map_.end(), new PointList<LeafNode>());
							#ifndef OCTREE_MULTI_POINTCLOUD_THREADING
								device_voxel_map_.insert(device_voxel_map_.end(), new PointList<LeafContainerT>());
							#else
								device_voxel_map_.insert(device_voxel_map_.end(), new PointList<LeafContainerT>(num_hw_threads));
							#endif
							current_point_clouds_.insert(current_point_clouds_.end(), nullptr);

							#ifdef OCTREE_MULTI_POINTCLOUD_WRAPPER_POOLING
								OctreeMultiPointCloudPointWrapper<PointT>::reserveMemory(device->point_count * 1.1);
								//auto test = new OctreeObjectPool<OctreeMultiPointCloudPointWrapper<PointT>>();
							#endif

							// ListNode threadable pools
							#ifdef OCTREE_MULTI_POINTCLOUD_THREADING
								olnp.reserveMemory(device->point_count * 2.1, num_hw_threads);
								olnp_container.reserveMemory(device->point_count * 2.1, num_hw_threads);
							#endif

							// ListNode pools
							#ifdef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
								OctreeListNode<LeafContainerT>::reserveMemory(device->point_count * 2);
								OctreeListNode<OctreeMultiPointCloudPointWrapper<PointT>>::reserveMemory(device->point_count * 2.1);
							#endif
						}
					}

					/*for (typename OctreeMultiPointCloud<PointT>::LeafNodeBreadthFirstIterator it = this->leaf_breadth_begin(),
								end=this->leaf_breadth_end(); it!= end; ++it) {
						it.getLeafContainer().registerDevices();
					}*/
					return false;
				}

				int added_points;

				/** \brief Add new point to octree.
				  * \param[in] new_point the new point to add
				  * \return Pointer to the Voxel into which the point was inserted.
				  */
				LeafContainerT*
				addPoint (OctreeMultiPointCloudPointWrapper<PointT>* const& new_point, int poolSegment = 0) {
					this->running = true;

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



					//LeafContainerT* container = this->findLeaf(key);
					//octree_lock.lock();
					LeafContainerT* container = this->createLeaf(key);
					//if (container == nullptr) {
						// add point to octree at key
						//container = this->createLeaf(key);
					//}

					if (container->isVirgin())
						container->registerDevices(&registered_devices_, num_hw_threads);
					#ifndef OCTREE_MULTI_POINTCLOUD_THREADING
						container->addPoint(new_point);
					#else
						container->addPoint(new_point, olnp, poolSegment);
					#endif
					container->setKey(key);

					auto temp = device_voxel_map_.at(new_point->getDevice()->device_id);
					//temp->insert(reinterpret_cast<LeafContainerT*>(&(*leaf_node)));
					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_INSERT
						octree_lock.lock();
					#endif

					#ifndef OCTREE_MULTI_POINTCLOUD_THREADING
						temp->insert(container);
					#else
						temp->insert(container, olnp_container, poolSegment);
					#endif

					#ifdef OCTREE_MULTI_POINTCLOUD_LOCK_INSERT
						octree_lock.unlock();
					#endif
					++added_points;

					//return reinterpret_cast<std::uintptr_t>(container);
					return container;

					/*OctreeKey key;

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
					(*leaf_node)->setKey(key);

					auto temp = device_voxel_map_.at(new_point->getDevice()->device_id);
					//temp->insert(reinterpret_cast<LeafContainerT*>(&(*leaf_node)));
					temp->insert(leaf_node);*/
					//temp->second->insert(reinterpret_cast<LeafContainerT*>(&(*leaf_node)));
					//temp->second->insert(new OctreeMultiPointCloudContainer<PointXYZ>);
				}

			private:

				std::set<SCDevice*> registered_devices_;  // Saves list of all currently registered devices
				std::vector<PointCloud<PointT>*> current_point_clouds_;
				//std::map<int, std::set<LeafContainerT*>*> device_voxel_map_;  // Saves combination of device id and all occupied voxels
				//std::vector<PointList<LeafContainerT>*> device_voxel_map_;  // Saves combination of device id and all occupied voxels
				std::vector<PointList<LeafContainerT>*> device_voxel_map_;   // Saves combination of device id and all occupied voxels
				//PointList* device_voxel_map = new PointList;
				bool running = false;

				OctreeThreadPool otp{num_hw_threads};
				OctreeListNodePool<OctreeMultiPointCloudPointWrapper<PointT>> olnp;
				OctreeListNodePool<LeafContainerT> olnp_container;

		};
	}
}

// TODO Evaluate Notice: Note: Don't precompile this octree type to speed up compilation. It's probably rarely used.
//#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>

#endif  // PCL_OCTREE_MULTI_POINTCLOUD_HPP