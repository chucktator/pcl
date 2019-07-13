//
// Created by alex on 11.07.19.
//

#ifndef PCL_OCTREE_LIST_NODE_POOL_H
#define PCL_OCTREE_LIST_NODE_POOL_H

#include <pcl/octree/octree_list_node.h>

namespace pcl {
	namespace octree {

		template<typename NodeTypeT>
		class OctreeListNodePool {

			public:

				OctreeListNodePool(int segments = 1) {
					used_head_ = new OctreeListNode <NodeTypeT> *[segments];
					used_tail_ = new OctreeListNode <NodeTypeT> *[segments];
					free_head_ = new OctreeListNode <NodeTypeT> *[segments];
					free_tail_ = new OctreeListNode <NodeTypeT> *[segments];

					used_size = new int[segments];
					free_size = new int[segments];
					capacity = new int[segments];
					requested_objects = new int[segments];
					returned_objects = new int[segments];

					for (int i = 0; i < segments; ++i) {
						//this->reserveMemory((size / segments * 1.1), i);
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
				}

				~OctreeListNodePool() {
					delete[] used_head_;
					delete[] used_tail_;
					delete[] used_size;

					delete[] free_head_;
					delete[] free_tail_;
					delete[] free_size;

					delete[] capacity;
					delete[] requested_objects;
					delete[] returned_objects;
				}

				void reserveMemory(uint64_t totalSize, int segments = 1) {
					for (int i = 0; i < segments; ++i) {
						this->reserveMemorySegment((int) (totalSize / segments * 1.1), i);
					}

				}

				void
				freeMemory() {

				}

				OctreeListNode <NodeTypeT> *
				getFreeNode(int segment = 0) {
					++requested_objects[segment];

					OctreeListNode <NodeTypeT> *temp = this->memPop(this->free_head_[segment], this->free_tail_[segment],
																	this->free_size[segment]);
					if (temp != nullptr) {
						this->memPush(this->used_head_[segment], this->used_tail_[segment], this->used_size[segment], temp);
						temp->in_use = true;
					}

					++used_size[segment];
					--free_size[segment];
					return temp;
				}

				void
				returnUsedNode(OctreeListNode <NodeTypeT> *node) {
					++returned_objects[node->pool_segment_];
					node->recycled = true;
					if (this->memPunch(this->used_head_[node->pool_segment_], this->used_tail_[node->pool_segment_],
									   this->used_size[node->pool_segment_], node)) {
						node->resetListPointers();
						this->memPush(this->free_head_[node->pool_segment_], this->free_tail_[node->pool_segment_],
									  this->free_size[node->pool_segment_], node);
					}
					--used_size[node->pool_segment_];
					++free_size[node->pool_segment_];
				}

			protected:

				void
				reserveMemorySegment(uint64_t size, int segment = 0) {
					auto *temp = (OctreeListNode <NodeTypeT> *) std::malloc(size * sizeof(OctreeListNode < NodeTypeT > ));
					this->malloc_pointers_.push_back(temp);
					for (int i = 0; i < size; i++) {
						auto *node = new(temp++) OctreeListNode<NodeTypeT>();
						this->memPush(this->free_head_[segment], this->free_tail_[segment], this->free_size[segment], node);
						node->pool_segment_ = segment;
					}
					this->capacity[segment] += size;
					this->totalCapacity += size;
				}

				bool
				memPush(OctreeListNode <NodeTypeT> *&head, OctreeListNode <NodeTypeT> *&tail, int &size,
						OctreeListNode <NodeTypeT> *node) {


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

				OctreeListNode <NodeTypeT> *
				memPop(OctreeListNode <NodeTypeT> *&head, OctreeListNode <NodeTypeT> *&tail, int &size) {

					if (head == nullptr)
						return nullptr;
					OctreeListNode <NodeTypeT> *temp = head;
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
				memPunch(OctreeListNode <NodeTypeT> *&head, OctreeListNode <NodeTypeT> *&tail, int &size,
						 OctreeListNode <NodeTypeT> *node) {

					if (node == nullptr)
						return false;

					if (node == head) {
						head = head->mem_next_;
					}

					if (node == tail) {
						tail = tail->mem_prev_;
					}


					OctreeListNode <NodeTypeT> *prev = node->mem_prev_, *next = node->mem_next_;

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
				std::vector<OctreeListNode < NodeTypeT>*>
				malloc_pointers_;

				OctreeListNode <NodeTypeT> **used_head_, **used_tail_;
				OctreeListNode <NodeTypeT> **free_head_, **free_tail_;


		};

	}
}

#endif //PCL_OCTREE_LIST_NODE_POOL_H
