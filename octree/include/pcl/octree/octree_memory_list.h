//
// Created by alex on 27.05.19.
//

#ifndef PCL_OCTREE_MEMORY_LIST_H
#define PCL_OCTREE_MEMORY_LIST_H

#include "octree_list.h"

namespace pcl {
	namespace octree {

		template<typename ListT>
		class OctreeList;

		template<typename ContentT>
		class OctreeListNode;


		template<typename ContentT>
		class MemoryList : public OctreeList<ContentT> {

			public:

				MemoryList() : OctreeList<ContentT>() {

				}

				~MemoryList() = default;

				void
				push(ContentT* const& content) {
					return this->insertEnd(content);
				}

				ContentT* const&
				pop() {
					if (this->size_ <= 0) {
						return nullptr;
					}
					ContentT* const& temp = this->head;
					this->head_ = this->head_->getNext();
					this->size_--;
					return temp;
				}

				ContentT* const&

				punch(u_int key) {
					OctreeListNode<ContentT> *curr = this->head_, *prev = nullptr;

					while (curr != nullptr) {
						if (curr->getKey() == key) {
							if (prev == nullptr) {
								this->head_ = this->head_->getNext();
							} else {
								prev->setNext(curr->getNext());
							}
							this->size_--;
							return curr;
						}
						prev = curr;
						curr = curr->getNext();
					}
					return nullptr;
				}

		};
	}
}


#endif //PCL_OCTREE_MEMORY_LIST_H
