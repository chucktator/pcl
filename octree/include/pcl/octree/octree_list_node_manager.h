//
// Created by alex on 16.07.19.
//

#ifndef PCL_OCTREE_LIST_NODE_MANAGER_H
#define PCL_OCTREE_LIST_NODE_MANAGER_H

#include <pcl/octree/octree_list_node.h>
#include <new>

namespace pcl {
	namespace octree {

		enum OCTREE_MEMORY_STRATEGY {
			STD_ALLOCATOR,
			OBJECT_POOLING
		};

		template <typename NodeTypeT>
		class OctreeListNodeManager {

			public:

				OctreeListNodeManager(int segments = 1, OCTREE_MEMORY_STRATEGY strategy = STD_ALLOCATOR) {
					if (segments <= 0)
						segments = 1;

					strategy_ = strategy;

					if (strategy_ == STD_ALLOCATOR) {
						segments = 1;
					}

					used_head_ = new OctreeListNode<NodeTypeT> *[segments];
					used_tail_ = new OctreeListNode<NodeTypeT> *[segments];
					free_head_ = new OctreeListNode<NodeTypeT> *[segments];
					free_tail_ = new OctreeListNode<NodeTypeT> *[segments];

					used_size = new int[segments];
					free_size = new int[segments];
					capacity = new int[segments];
					requested_objects = new int[segments];
					returned_objects = new int[segments];

					for (int i = 0; i < segments; i++) {
						used_head_[i] = nullptr;
						used_tail_[i] = nullptr;
						free_head_[i] = nullptr;
						free_tail_[i] = nullptr;

						used_size[i] = 0;
						free_size[i] = 0;
						capacity[i] = 0;
						requested_objects[i] = 0;
						returned_objects[i] = 0;
					}

					segments_ = segments;
				}

				void reserveMemory(uint64_t totalSize) {
					// TODO  add memory to block for new allocations in case of 'STD_ALLOCATOR'
					if (strategy_ != STD_ALLOCATOR) {
						for (int i = 0; i < segments_; i++) {
							reserveMemorySegment((int) (totalSize / segments_ * 1.1), i);
						}
					}
				}

				void
				freeMemory() {
					for (auto entry : malloc_pointers_) {
						free(entry);
					}
				}

				OctreeListNode<NodeTypeT>*
				getFreeNode(int segment = 0) {
					OctreeListNode<NodeTypeT> *temp = nullptr;

					++requested_objects[segment];

					switch (strategy_) {

						case STD_ALLOCATOR: {
							temp = new OctreeListNode<NodeTypeT>();
							break;
						}

						case OBJECT_POOLING: {
							temp = memPop(free_head_[segment], free_tail_[segment], free_size[segment]);
							if (temp != nullptr) {
								#ifdef OCTREE_MULTI_POINTCLOUD_TRACK_USED_NODES
									memPush(used_head_[segment], used_tail_[segment], used_size[segment], temp);
								#endif
								temp->in_use = true;
							}
							break;
						}

						default: {
							break;
						}

					}

					if (temp == nullptr) {
						throw std::bad_alloc();
					}

					++used_size[segment];
					--free_size[segment];
					return temp;
				}

				void
				returnUsedNode(OctreeListNode<NodeTypeT> *node) {
					++returned_objects[node->pool_segment_];
					node->reset();
					switch (strategy_) {

						case STD_ALLOCATOR: {
							delete node;
							break;
						}

						case OBJECT_POOLING: {
							node->recycled = true;
							#ifdef OCTREE_MULTI_POINTCLOUD_TRACK_USED_NODES
								if (memPunch(used_head_[node->pool_segment_], used_tail_[node->pool_segment_], used_size[node->pool_segment_], node)) {
							#endif
								memPush(free_head_[node->pool_segment_], free_tail_[node->pool_segment_], free_size[node->pool_segment_], node);
							#ifdef OCTREE_MULTI_POINTCLOUD_TRACK_USED_NODES
								}
							#endif
							break;
						}

						default: {
							break;
						}

					}
					--used_size[node->pool_segment_];
					++free_size[node->pool_segment_];
				}

			protected:

				void
				reserveMemorySegment(uint64_t size, int segment = 0) {
					auto *temp = (OctreeListNode<NodeTypeT> *) std::malloc(
							size * sizeof(OctreeListNode<NodeTypeT>));
					malloc_pointers_.push_back(temp);
					for (int i = 0; i < size; ++i) {
						auto *node = new(temp++) OctreeListNode<NodeTypeT>();
						memPush(free_head_[segment], free_tail_[segment],
									  free_size[segment], node);
						node->pool_segment_ = segment;
					}
					capacity[segment] += size;
					totalCapacity += size;
				}

				bool
				memPush(OctreeListNode<NodeTypeT> *&head, OctreeListNode<NodeTypeT> *&tail, int &size, OctreeListNode<NodeTypeT> *node) {

					if (node == nullptr)
						return false;

					if (head == nullptr && tail == nullptr) {
						head = node;
						tail = node;
						size++;
						return true;
					}
					tail->mem_next_ = node;
					node->mem_prev_ = tail;
					tail = node;
					size++;

					return true;
				}

				OctreeListNode<NodeTypeT>*
				memPop(OctreeListNode<NodeTypeT> *&head, OctreeListNode<NodeTypeT> *&tail, int &size) {

					if (head == nullptr)
						return nullptr;
					OctreeListNode<NodeTypeT> *temp = head;
					if (head != tail) {
						if (head->mem_next_ == nullptr) {
							return nullptr;
						}
						head->mem_next_->mem_prev_ = nullptr;
						head = head->mem_next_;
					} else {
						head = nullptr;
						tail = nullptr;
					}
					size--;
					temp->reset();

					return temp;
				}

				bool
				memPunch(OctreeListNode<NodeTypeT> *&head, OctreeListNode<NodeTypeT> *&tail, int &size, OctreeListNode<NodeTypeT> *node) {

					if (node == nullptr)
						return false;

					if (node == head) {
						head = head->mem_next_;
					}

					if (node == tail) {
						tail = tail->mem_prev_;
					}


					OctreeListNode<NodeTypeT> *prev = node->mem_prev_, *next = node->mem_next_;

					if (prev != nullptr) {
						prev->mem_next_ = next;
					}

					if (next != nullptr) {
						next->mem_prev_ = prev;
					}

					size--;

					node->reset();

					return true;
				}

			private:
				int *used_size, *free_size, *capacity, totalCapacity = 0;
				int *requested_objects, *returned_objects;
				std::vector<OctreeListNode<NodeTypeT>*> malloc_pointers_;

				OctreeListNode<NodeTypeT> **used_head_, **used_tail_;
				OctreeListNode<NodeTypeT> **free_head_, **free_tail_;

				OCTREE_MEMORY_STRATEGY strategy_;

				int segments_;

		};
	}
}

#endif //PCL_OCTREE_LIST_NODE_MANAGER_H
