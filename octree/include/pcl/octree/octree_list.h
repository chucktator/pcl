//
// Created by alex on 13.05.19.
//

#ifndef PCL_OCTREE_LIST_H
#define PCL_OCTREE_LIST_H

#include <mutex>
#include <pcl/octree/octree_list_node.h>
#include <pcl/octree/octree_list_node_pool.h>

namespace pcl {
	namespace octree {

		template<typename T>
		class OctreeObjectPool;


		template<typename ListT>
		class OctreeList {

			friend class OctreeObjectPool<ListT>;

			template <class Type, class UnqualifiedType = std::remove_cv_t<Type>>
			class ForwardIterator : public std::iterator<std::forward_iterator_tag, UnqualifiedType, std::ptrdiff_t, Type*, Type&> {
				friend class OctreeList;
				OctreeListNode<UnqualifiedType>* itr;
				OctreeList<UnqualifiedType>* parent;
				int segment = 0;

				explicit ForwardIterator(OctreeListNode<UnqualifiedType>* nd, OctreeList<UnqualifiedType>* list)
					: itr(nd), parent(list) {}


				public:

					ForwardIterator()   // Default construct gives end.
							: itr(nullptr)
					{
					}

					void swap(ForwardIterator& other) noexcept
					{
						using std::swap;
						swap(itr, other.iter);
					}

					ForwardIterator& operator++ () // Pre-increment
					{
						assert(itr != nullptr && "Out-of-bounds iterator increment!");

						itr = itr->getNext();
						if (itr == nullptr && this->segment < this->parent->segments_) {
							int old_segment = this->segment++;
							while (this->parent->heads_[this->segment] == nullptr && this->segment < this->parent->segments_-1) {
								++this->segment;
							}
							std::cout << "Moving from segment " << old_segment << " to segment " << this->segment << " of " << this->parent->segments_ << std::endl;
							itr = this->parent->heads_[this->segment];
						}
						return *this;
					}

					ForwardIterator operator++ (int) // Post-increment
					{
						assert(itr != nullptr && "Out-of-bounds iterator increment!");
						ForwardIterator tmp(*this);
						++(*this);
						return tmp;
					}

					// two-way comparison: v.begin() == v.cbegin() and vice versa
					template<class OtherType>
					bool operator == (const ForwardIterator<OtherType>& rhs) const
					{
						return itr == rhs.itr;
					}

					template<class OtherType>
					bool operator != (const ForwardIterator<OtherType>& rhs) const
					{
						return itr != rhs.itr;
					}

					Type* operator* () const
					{
						assert(itr != nullptr && "Invalid iterator dereference!");
						return itr->getContent();
					}

					Type& operator-> () const
					{
						assert(itr != nullptr && "Invalid iterator dereference!");
						return *(*itr->getContent());
					}

					// One way conversion: iterator -> const_iterator
					// TODO fix
					operator ForwardIterator<const Type>() const
					{
						return ForwardIterator<const Type>(itr);
					}
			};

			// `iterator` and `const_iterator` used by your class:
			typedef ForwardIterator<ListT> iterator;
			typedef ForwardIterator<const ListT> const_iterator;

			public:

				OctreeList(int segments = 1) {
					std::cerr << "Creating OctreeList with " << segments << " segments!" << std::endl;
					//this->head_ = nullptr;
					//this->tail_ = nullptr;
					this->segments_ = segments;

					this->heads_ = new OctreeListNode <ListT> *[segments];
					this->tails_ = new OctreeListNode <ListT> *[segments];
					this->sizes_ = new uint64_t[segments];

					for (int i = 0; i < segments; ++i) {
						this->heads_[i] = nullptr;
						this->tails_[i] = nullptr;

						this->sizes_[i] = 0;
					}
				}

				~OctreeList() {
					//this->clear();
					delete[] this->heads_;
					delete[] this->tails_;
					delete[] this->sizes_;
				}

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				virtual
				uint8_t
				insert(ListT* const& content, int segment = 0) {
					//return this->insertNode(new OctreeListNode<ListT>(reinterpret_cast<std::uintptr_t>(content), content));
					#ifndef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
						OctreeListNode<ListT> *temp = new OctreeListNode<ListT>();
					#else
						OctreeListNode<ListT> *temp = OctreeListNode<ListT>::getFreeNode();
					#endif
					temp->init(reinterpret_cast<std::uintptr_t>(content), content);
					//temp->setListSegment(segment);
					return this->insertNode(temp, segment);
				}

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				virtual
				uint8_t
				insert(ListT* const& content, OctreeListNodePool<ListT> &olnp, int poolSegment) {
					//return this->insertNode(new OctreeListNode<ListT>(reinterpret_cast<std::uintptr_t>(content), content));
					OctreeListNode<ListT> *temp = olnp.getFreeNode(poolSegment);
					temp->init(reinterpret_cast<std::uintptr_t>(content), content);
					//temp->setListSegment(poolSegment);
					return this->insertNode(temp, poolSegment);
				}


				/*virtual
				bool
				remove(u_int key) {
					OctreeListNode<ListT> *curr = this->head_, *prev = nullptr;

					while (curr != nullptr) {
						if (curr->getKey() == key) {
							if (prev == nullptr) {
								this->head_ = this->head_->getNext();
							} else {
								prev->setNext(curr->getNext());
							}
							OctreeListNode<ListT>::returnUsedNode(curr);
							this->size_--;
							return true;
						}
						prev = curr;
						curr = curr->getNext();
					}
					return false;
				}*/

				virtual
				bool
				remove(u_int key) {
					return this->deleteNode(findNode(key));
				}

				virtual
				ListT*
				find(u_int key) {
					OctreeListNode<ListT> *ret = findNode(key);
					if (ret == nullptr) {
						return nullptr;
					}
					return ret->getContent();
				}

				virtual
				void
				clear(std::function<void(ListT*)> func = nullptr) {
					int i = 0;
					for (int segment=0; segment<this->segments_; ++segment) {
						OctreeListNode<ListT> *curr = this->heads_[segment];
						this->heads_[segment] = nullptr;
						this->tails_[segment] = nullptr;
						//std::cout << "Clearing list of " << typeid(ListT).name() << " containing " << this->size_ << " items. The memory pool contains " << OctreeListNode<ListT>::used_size << " objects" << std::endl;
						//int returned = OctreeListNode<ListT>::returned_objects; //, used = OctreeListNode<ListT>::used_size, free = OctreeListNode<ListT>::free_size, list_size = this->size_;

						while (curr != nullptr) {
							OctreeListNode<ListT> *temp = curr;
							curr = curr->getNext();
							//delete temp->getContent();
							if (func != nullptr) {
								func(temp->getContent());
							}
							#ifndef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
								delete temp;
							#else
								OctreeListNode<ListT>::returnUsedNode(temp);
							#endif
							++i;
						}
					}
					if ((this->size_ - i) > 0)
						std::cout << "Leaving " << (this->size_ - i) << " orphans." << std::endl;
					this->size_ = 0;

					//std::cout << "+++ Tally sheet after clearing +++" << std::endl;
					//std::cout << "Pool now contains " << OctreeListNode<ListT>::used_size << " objects." << std::endl;
					//std::cout << (used - OctreeListNode<ListT>::used_size - list_size) << " objects were not properly removed from the used list." << std::endl;
					//std::cout << (OctreeListNode<ListT>::free_size - free - list_size) << " objects were not properly returned to the free list." << std::endl;
					//std::cout << "Attempted to free " << i << " objects of type " << typeid(ListT).name() << " in " << (OctreeListNode<ListT>::returned_objects - returned) << " tries." << std::endl;
				}

				virtual
				void
				clear(OctreeListNodePool<ListT> &olnp, std::function<void(ListT*)> func = nullptr) {
					int i=0;
					for (int segment=0; segment<this->segments_; ++segment) {
						OctreeListNode<ListT> *curr = this->heads_[segment];
						this->heads_[segment] = nullptr;
						this->tails_[segment] = nullptr;

						while (curr != nullptr) {
							OctreeListNode<ListT> *temp = curr;
							curr = curr->getNext();
							//delete temp->getContent();
							if (func != nullptr) {
								func(temp->getContent());
							}
							olnp.returnUsedNode(temp);
							++i;
						}
					}
					if ((this->size_ - i) > 0)
						std::cout << "Leaving " << (this->size_ - i) << " orphans." << std::endl;
					this->size_ = 0;
				}

				ForwardIterator<ListT>
				begin() {
					OctreeListNode<ListT> * tmp = this->heads_[0];
					int seg = 0;
					while (tmp == nullptr && seg < this->segments_) {
						tmp = this->heads_[seg++];
					}
					return ForwardIterator<ListT>(tmp, this);
				}

				ForwardIterator<ListT>
				end() {
					int seg = this->segments_ - 1;
					OctreeListNode<ListT> * tmp = this->tails_[seg];
					while (tmp == nullptr && seg > 0) {
						tmp = this->tails_[seg--];
					}
					return ForwardIterator<ListT>(tmp, this);
				}


			protected:

				//OctreeListNode<ListT> *head_ = nullptr, *tail_ = nullptr;
				OctreeListNode<ListT> **heads_ = nullptr, **tails_ = nullptr;
				int segments_;
				uint64_t size_ = 0;
				uint64_t *sizes_;

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				virtual
				uint8_t
				insertNode(OctreeListNode<ListT> *const &node, int segment = 0) {
					if (this->heads_[segment] == nullptr && this->tails_[segment] == nullptr) {
						this->heads_[segment] = node;
						this->tails_[segment] = node;
						++this->sizes_[segment];
						++this->size_;
						return 2;
					}

					this->insertEnd(node, segment);

					return 0;
				}

				bool
				pushNode(OctreeListNode<ListT> *node, int segment = 0) {
					return this->insertNode(node, segment);
				}

				OctreeListNode<ListT>*
				popNode(int segment = 0) {
					if (this->heads_[segment] == nullptr)
						return nullptr;

					OctreeListNode<ListT> *temp = this->heads_[segment];

					if (this->heads_[segment] != this->tails_[segment]) {
						if (this->heads_[segment]->getNext() == nullptr)
							return nullptr;
						this->heads_[segment]->getNext()->setPrev(nullptr);
						this->heads_[segment] = this->heads_[segment]->getNext();
					} else {
						this->heads_[segment] = nullptr;
						this->tails_[segment] = nullptr;
					}

					this->sizes_[segment]--;
					this->size_--;
					temp->resetListPointers();
					return temp;
				}

				bool
				punchNode(OctreeListNode<ListT> *const &node, int segment = 0) {
					if (node == nullptr)
						return false;

					if (node == this->heads_[segment]) {
						this->heads_[segment] = this->heads_[segment]->getNext();
					}

					if (node == this->tails_[segment]) {
						this->tails_[segment] = this->tails_[segment]->getPrev();
					}


					OctreeListNode<ListT> *prev = node->getPrev(), *next = node->getNext();

					if (prev != nullptr) {
						prev->setNext(next);
					}

					if (next != nullptr) {
						next->setPrev(prev);
					}

					this->size_--;

					node->resetListPointers();

					return true;
				}

				virtual
				OctreeListNode<ListT>*
				findNode(u_int key) {
					// TODO  Run across all segments
					/*OctreeListNode<ListT> *curr = this->head_;

					while (curr != nullptr) {
						if (curr->getKey() == key) {
							return curr;
						}
						curr = curr->getNext();
					}*/

					return nullptr;
				}

				virtual
				bool
				deleteNode(OctreeListNode<ListT> *const &node, int segment = 0) {
					this->punchNode(node, segment);

					#ifndef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
						delete node;
					#else
						OctreeListNode<ListT>::returnUsedNode(node);
					#endif

					return true;
				}

				void
				insertStart(OctreeListNode<ListT> *const &node, int segment = 0) {
					if (this->heads_[segment] == nullptr && this->tails_[segment] == nullptr) {
						this->heads_[segment] = node;
						this->tails_[segment] = node;
						++this->sizes_[segment];
						++this->size_;
						return;
					}
					OctreeListNode<ListT> *temp = this->heads_[segment];
					this->heads_[segment] = node;

					temp->setPrev(node);
					node->setNext(temp);
					++this->sizes_[segment];
					++this->size_;
				}

				void
				insertEnd(OctreeListNode<ListT> *const &node, int segment = 0) {
					if (this->tails_[segment] == nullptr) {
						this->insertStart(node);
						return;
					}
					this->tails_[segment]->setNext(node);
					node->setPrev(this->tails_[segment]);
					this->tails_[segment] = node;
					++this->sizes_[segment];
					++this->size_;
				}

				void
				insertBefore(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &before, int segment = 0) {
					before->getPrev()->setNext(node);
					before->setPrev(node);
					node->setNext(before);
					++this->sizes_[segment];
					++this->size_;
				}

				void
				insertAfter(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &after, int segment = 0) {
					node->setNext(after->getNext());
					node->setPrev(after);
					after->setNext(node);
					++this->sizes_[segment];
					++this->size_;
				}
		};
	}
}


#endif //PCL_OCTREE_LIST_H
