//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_VOXEL_LIST_H
#define PCL_OCTREE_VOXEL_LIST_H

#include "octree_list.h"

namespace pcl {
	namespace octree {


		template<typename ListT>
		class VoxelList : public OctreeList<ListT> {


			public:

				VoxelList() : OctreeList<ListT>() {

				}

				~VoxelList() = default;

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				uint8_t
				insertNode(OctreeListNode<ListT> *const &node) override {
					//int head_key, tail_key, curr_key;

					if (this->head_ == nullptr && this->tail_ == nullptr) {
						this->head_ = node;
						this->tail_ = node;
						this->size_++;
						return 2;
					}

					if (this->head_->getKey() == node->getKey() || this->tail_->getKey() == node->getKey()) {
						return 1;
					}

					if (this->head_->getKey() > node->getKey()) {
						this->insertStart(node);
						this->size_++;
						return 0;
					}

					if (this->tail_->getKey() < node->getKey()) {
						this->size_++;
						this->insertEnd(node);
						return 0;
					}

					OctreeListNode<ListT> *curr = this->head_, *curr_next;

					while (curr != nullptr) {
						curr_next = curr->getNext();
						if (curr_next != nullptr && curr_next->getKey() == node->getKey())
							return 1;
						if (curr->getKey() < node->getKey()) {
							//if (curr->next_ != nullptr && node->getKey() < curr->next_->getKey()) {
								this->insertAfter(node, curr);
								this->size_++;
								return 0;
							//}
						}
						curr = curr_next;
					}

					return 3;
				}

				ListT*
				find(u_int key) override {
					OctreeListNode<ListT> *curr = this->head_;

					while (curr != nullptr) {
						if (curr->getKey() == key) {
							return curr->getContent();
						}
						curr = curr->getNext();
					}
				}

		};
	}
}

#endif //PCL_OCTREE_VOXEL_LIST_H
