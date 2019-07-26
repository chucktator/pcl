//
// Created by alex on 13.05.19.
//

#ifndef PCL_OCTREE_LIST_H
#define PCL_OCTREE_LIST_H

#include <pcl/octree/octree_list_node.h>
#include <pcl/octree/octree_list_node_manager.h>
#include <mutex>

namespace pcl {
	namespace octree {

		template<typename T>
		class OctreeObjectPool;

		template<typename T>
		class OctreeMultiThreadedList;

		template<typename ListT>
		class OctreeList {

			friend class OctreeObjectPool<ListT>;
			friend class OctreeMultiThreadedList<ListT>;

			template <class Type, class UnqualifiedType = std::remove_cv_t<Type>>
			class ForwardIterator : public std::iterator<std::forward_iterator_tag, UnqualifiedType, std::ptrdiff_t, Type*, Type&> {
				friend class OctreeList;
				OctreeListNode<UnqualifiedType>* itr;

				explicit ForwardIterator(OctreeListNode<UnqualifiedType>* nd)
						: itr(nd)
				{
				}

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
						return *this;
					}

					ForwardIterator operator++ (int) // Post-increment
					{
						assert(itr != nullptr && "Out-of-bounds iterator increment!");
						ForwardIterator tmp(*this);
						itr = itr->getNext();
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
					operator ForwardIterator<const Type>() const
					{
						return ForwardIterator<const Type>(itr);
					}
			};

			// `iterator` and `const_iterator` used by your class:
			typedef ForwardIterator<ListT> iterator;
			typedef ForwardIterator<const ListT> const_iterator;

			public:

				// TODO  propagate dedicatedToThread??
				OctreeList(OctreeListNodeManager<ListT> *manager) {
					head_ = nullptr;
					tail_ = nullptr;

					manager_ = manager;
				}

				~OctreeList() {
					clear();
				}

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				// TODO  Create multiple lists for different threads
				virtual
				uint8_t
				insert(ListT* const& content, int threadNum = 0) {
					cleared_ = false;
					if (manager_ == nullptr) {
						throw "No memory manager provided!";
					}
					OctreeListNode<ListT> *temp = manager_->getFreeNode(threadNum);
					temp->init(reinterpret_cast<std::uintptr_t>(content), content);
					return insertNode(temp);
				}


				/*virtual
				bool
				remove(u_int key) {
					OctreeListNode<ListT> *curr = head_, *prev = nullptr;

					while (curr != nullptr) {
						if (curr->getKey() == key) {
							if (prev == nullptr) {
								head_ = head_->getNext();
							} else {
								prev->setNext(curr->getNext());
							}
							OctreeListNode<ListT>::returnUsedNode(curr);
							size_--;
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
					return deleteNode(findNode(key));
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
				clear(std::function<void(ListT*)> contentFunction = nullptr) {
					clearing_ = true;
					OctreeListNode<ListT> *curr = head_;
					int thread = -1;
					if (head_ != nullptr)
						thread = head_->getPoolSegment();
					head_ = nullptr;
					tail_ = nullptr;
					//std::cout << "Clearing list of " << typeid(ListT).name() << " containing " << size_ << " items. The memory pool contains " << OctreeListNode<ListT>::used_size << " objects" << std::endl;
					//int returned = OctreeListNode<ListT>::returned_objects; //, used = OctreeListNode<ListT>::used_size, free = OctreeListNode<ListT>::free_size, list_size = size_;

					int i=0;
					while (curr != nullptr) {
						OctreeListNode<ListT> *temp = curr;
						assert(curr->getPoolSegment() == thread);
						curr = curr->getNext();
						//delete temp->getContent();
						if (contentFunction != nullptr) {
							contentFunction(temp->getContent());
						}
						manager_->returnUsedNode(temp);
						i++;
					}
					if ((size_ - i) > 0)
						std::cout << "Leaving " << (size_ - i) << " orphans." << std::endl;
					size_ = 0;

					cleared_ = true;
					clearing_ = false;

					//std::cout << "+++ Tally sheet after clearing +++" << std::endl;
					//std::cout << "Pool now contains " << OctreeListNode<ListT>::used_size << " objects." << std::endl;
					//std::cout << (used - OctreeListNode<ListT>::used_size - list_size) << " objects were not properly removed from the used list." << std::endl;
					//std::cout << (OctreeListNode<ListT>::free_size - free - list_size) << " objects were not properly returned to the free list." << std::endl;
					//std::cout << "Attempted to free " << i << " objects of type " << typeid(ListT).name() << " in " << (OctreeListNode<ListT>::returned_objects - returned) << " tries." << std::endl;
				}

				ForwardIterator<ListT>
				begin() {
					return ForwardIterator<ListT>(head_);
				}

				ForwardIterator<ListT>
				end() {
					return ForwardIterator<ListT>(nullptr);
				}


			protected:

				OctreeListNode<ListT> *head_ = nullptr, *tail_ = nullptr;
				uint64_t size_ = 0;
				OctreeListNodeManager<ListT> *manager_;
				bool clearing_ = false, cleared_ = true;

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				virtual
				uint8_t
				insertNode(OctreeListNode<ListT> *const &node) {
					if (head_ == nullptr && tail_ == nullptr) {
						head_ = node;
						tail_ = node;
						size_++;
						return 2;
					}

					insertEnd(node);

					return 0;
				}

				bool
				pushNode(OctreeListNode<ListT> *node) {
					return insertNode(node);
				}

				OctreeListNode<ListT>*
				popNode() {
					if (head_ == nullptr)
						return nullptr;

					OctreeListNode<ListT> *temp = head_;

					if (head_ != tail_) {
						if (head_->getNext() == nullptr)
							return nullptr;
						head_->getNext()->setPrev(nullptr);
						head_ = head_->getNext();
					} else {
						head_ = nullptr;
						tail_ = nullptr;
					}

					size_--;
					temp->resetListPointers();
					return temp;
				}

				bool
				punchNode(OctreeListNode<ListT> *const &node) {
					if (node == nullptr)
						return false;

					if (node == head_) {
						head_ = head_->getNext();
					}

					if (node == tail_) {
						tail_ = tail_->getPrev();
					}


					OctreeListNode<ListT> *prev = node->getPrev(), *next = node->getNext();

					if (prev != nullptr) {
						prev->setNext(next);
					}

					if (next != nullptr) {
						next->setPrev(prev);
					}

					size_--;

					node->resetListPointers();

					return true;
				}

				virtual
				OctreeListNode<ListT>*
				findNode(u_int key) {
					OctreeListNode<ListT> *curr = head_;

					while (curr != nullptr) {
						if (curr->getKey() == key) {
							return curr;
						}
						curr = curr->getNext();
					}

					return nullptr;
				}

				virtual
				bool
				deleteNode(OctreeListNode<ListT> *const &node) {
					punchNode(node);

					manager_->returnUsedNode(node);

					return true;
				}

				void
				insertStart(OctreeListNode<ListT> *const &node) {
					if (head_ == nullptr && tail_ == nullptr) {
						head_ = node;
						tail_ = node;
						size_++;
						return;
					}
					OctreeListNode<ListT> *temp = head_;
					head_ = node;

					temp->setPrev(node);
					node->setNext(temp);
					size_++;
				}

				void
				insertEnd(OctreeListNode<ListT> *const &node) {
					if (tail_ == nullptr) {
						insertStart(node);
						return;
					}
					tail_->setNext(node);
					node->setPrev(tail_);
					tail_ = node;
					size_++;
				}

				void
				insertBefore(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &before) {
					before->getPrev()->setNext(node);
					before->setPrev(node);
					node->setNext(before);
					size_++;
				}

				void
				insertAfter(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &after) {
					node->setNext(after->getNext());
					node->setPrev(after);
					after->setNext(node);
					size_++;
				}
		};
	}
}


#endif //PCL_OCTREE_LIST_H
