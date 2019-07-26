//
// Created by alex on 25.06.19.
//

#ifndef PCL_OCTREEOBJECTPOOL_H
#define PCL_OCTREEOBJECTPOOL_H

#include <pcl/octree/octree_multithreaded_list.h>
#include <pcl/octree/octree_poolable_object.h>


namespace pcl {
	namespace octree {

		template<typename T>
		class OctreeListNode;

		template<typename T>
		class OctreeList;

		template<typename PoolObjectT>
		class OctreeObjectPool {

			public:

				OctreeObjectPool(int segments = 1, OCTREE_MEMORY_STRATEGY strategy = STD_ALLOCATOR) : pool_object_manager_(segments, strategy), free_(&pool_object_manager_, segments), used_(&pool_object_manager_, segments) {
					static_assert(std::is_base_of<OctreePoolableObject<PoolObjectT>, PoolObjectT>::value, "Derived not derived from BaseClass");

					if (segments <= 0)
						segments = 1;

					strategy_ = strategy;

					if (strategy_ == STD_ALLOCATOR) {
						segments = 1;
					}

					used_size = new int[segments];
					free_size = new int[segments];
					capacity = new int[segments];
					requested_objects = new int[segments];
					returned_objects = new int[segments];

					for (int i = 0; i < segments; i++) {
						used_size[i] = 0;
						free_size[i] = 0;
						capacity[i] = 0;
						requested_objects[i] = 0;
						returned_objects[i] = 0;
					}

					segments_ = segments;
				}

				void
				reserveMemory(uint64_t totalSize) {
					if (strategy_ != STD_ALLOCATOR) {
						pool_object_manager_.reserveMemory(totalSize * 1.5);
						for (int i = 0; i < segments_; ++i) {
							this->reserveMemorySegment((int) (totalSize / segments_ * 1.1), i);
						}
					}
				}

				void
				freeMemory() {
					for (auto entry : malloc_pointers_) {
						free(entry);
					}
				}

				PoolObjectT*
				getFreeObject(int segment = 0) {
					++requested_objects[segment];

					PoolObjectT *content;
					OctreeListNode<PoolObjectT> *temp;

					switch (strategy_) {

						case STD_ALLOCATOR: {
							content = new PoolObjectT();
							break;
						}

						case OBJECT_POOLING: {
							temp = free_.popNode(segment);
							content = temp->getContent();

							if (temp != nullptr) {
								#ifdef OCTREE_MULTI_POINTCLOUD_TRACK_USED_NODES
									used_.pushNode(temp, segment);
								#endif
								content->setInUse(true);
							}
						}
					}

					content->setListNodeReference(temp);

					++used_size[segment];
					--free_size[segment];

					return content;
				}

				void
				returnUsedObject(PoolObjectT *object, int segment = 0) {
					++returned_objects[segment];
					switch (strategy_) {

						case STD_ALLOCATOR: {
							delete object;
							break;
						}

						case OBJECT_POOLING: {
							#ifdef OCTREE_MULTI_POINTCLOUD_TRACK_USED_NODES
								if (used_.punchNode(object->getListNodeReference(), segment)) {
							#endif
								free_.pushNode(object->getListNodeReference(), segment);
							#ifdef OCTREE_MULTI_POINTCLOUD_TRACK_USED_NODES
								}
							#endif
						}
					}
					--used_size[segment];
					++free_size[segment];
				}

				int *used_size, *free_size, *capacity;
				int *requested_objects, *returned_objects;

			protected:

				void
				reserveMemorySegment(uint64_t size, int segment = 0) {
					PoolObjectT *temp = (PoolObjectT *) std::malloc(size * sizeof(PoolObjectT));
					malloc_pointers_.push_back(temp);

					for (int i = 0; i < size; i++) {
						PoolObjectT *object = new (temp++) PoolObjectT();
						free_.insert(object, segment);
						++free_size[segment];
						//std::cout << "Created new OctreeListNode at 0x" <<  reinterpret_cast<std::uintptr_t>(node) << " placed in memory at 0x" << reinterpret_cast<std::uintptr_t>(temp) << std::endl;
					}
					capacity[segment] += size;
				}


			private:
				OctreeMultiThreadedList<PoolObjectT> used_;
				OctreeMultiThreadedList<PoolObjectT> free_;

				OctreeListNodeManager<PoolObjectT> pool_object_manager_;

				std::vector<PoolObjectT*> malloc_pointers_;

				int segments_;
				OCTREE_MEMORY_STRATEGY strategy_;
		};
	}
}

// TODO  Circumvent or redirect direct access to methods of OctreeList


#endif //PCL_OCTREEOBJECTPOOL_H
