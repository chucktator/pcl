//
// Created by alex on 13.05.19.
//

#ifndef PCL_OCTREE_LIST_H
#define PCL_OCTREE_LIST_H

namespace pcl {
	namespace octree {

		template<typename ListT>
		class OctreeList;

		template<typename ContentT>
		class OctreeListNode {

			public:
				explicit
				OctreeListNode(u_int key) {
					this->key_ = key;
				}

				explicit
				OctreeListNode(u_int key, ContentT *content) {
					this->key_ = key;
					this->content_ = content;
				}

				~OctreeListNode() = default;

				inline
				u_int
				getKey() {
					return this->key_;
				}

				inline
				ContentT*
				getContent() {
					return this->content_;
				}

				inline
				OctreeListNode*
				getNext() {
					return this->next_;
				}

				inline
				void
				setNext(OctreeListNode* next) {
					this->next_ = next;
				}

			private:
				u_int key_ = 0;
				ContentT *content_ = nullptr;
				OctreeListNode *next_ = nullptr;
		};

		template<typename ListT>
		class OctreeList {

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

				OctreeList() {
					this->head_ = nullptr;
					this->tail_ = nullptr;
				}

				~OctreeList() {
					this->clear();
				}

				virtual
				uint8_t
				insert(ListT* const& content) {
					return this->insertNode(new OctreeListNode<ListT>(reinterpret_cast<std::uintptr_t>(content), content));
				}

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				virtual
				uint8_t
				insertNode(OctreeListNode<ListT> *const &node) {
					if (this->head_ == nullptr && this->tail_ == nullptr) {
						this->head_ = node;
						this->tail_ = node;
						this->size_++;
						return 2;
					}

					this->insertEnd(node);
					this->size_++;

					return 0;
				}

				virtual
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
							delete curr;
							this->size_--;
							return true;
						}
						prev = curr;
						curr = curr->getNext();
					}
					return false;
				}

				virtual
				ListT*
				find(u_int key) {
					OctreeListNode<ListT> *curr = this->head_;

					while (curr != nullptr) {
						if (curr->getKey() == key) {
							return curr->getContent();
						}
						curr = curr->getNext();
					}
				}

				virtual
				void
				clear() {
					OctreeListNode<ListT> *curr = this->head_;
					this->head_ = nullptr;
					this->tail_ = nullptr;

					int i=0;
					while (curr != nullptr) {
						OctreeListNode<ListT> *temp = curr;
						curr = curr->getNext();
						delete temp;
						i++;
					}
					this->size_ = 0;
				}

				ForwardIterator<ListT>
				begin() {
					return ForwardIterator<ListT>(this->head_);
				}

				ForwardIterator<ListT>
				end() {
					return ForwardIterator<ListT>(this->tail_);
				}


			protected:

				OctreeListNode<ListT> *head_, *tail_;
				uint64_t size_ = 0;

				void
				insertStart(OctreeListNode<ListT> *const &node) {
					OctreeListNode<ListT> *temp = this->head_;
					this->head_ = node;
					if (temp != NULL)
						node->setNext(temp);
					this->size_++;
				}

				void
				insertEnd(OctreeListNode<ListT> *const &node) {
					if (this->tail_ == NULL) {
						this->insertStart(node);
						return;
					}
					this->tail_->setNext(node);
					this->tail_ = node;
					this->size_++;
				}

				void
				insertBefore(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &before,
							 OctreeListNode<ListT> *const &beforeThat) {
					beforeThat->setNext(node);
					node->setNext(before);
					this->size_++;
				}

				void
				insertAfter(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &after) {
					node->setNext(after->getNext());
					after->setNext(node);
					this->size_++;
				}
		};
	}
}


#endif //PCL_OCTREE_LIST_H
