//
// Created by alex on 13.05.19.
//

#ifndef PCL_OCTREE_LIST_H
#define PCL_OCTREE_LIST_H


namespace pcl {
	namespace octree {

		template<typename ContentT>
		class OctreeListNode {

			public:

				inline
				void
				init(u_int key, ContentT *content) {
					this->key_ = key;
					this->content_ = content;

				}

				inline
				void
				reset() {
					this->key_ = 666;
					this->content_ = nullptr;
					this->next_ = nullptr;
					this->mem_next_ = nullptr;
					this->mem_prev_ = nullptr;
					this->in_use = false;
				}

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
				OctreeListNode<ContentT>*
				getNext() {
					return this->next_;
				}

				inline
				void
				setNext(OctreeListNode<ContentT>* next) {
					this->next_ = next;
				}

				static
				void
				reserveMemory(u_int size) {
					OctreeListNode<ContentT> *temp = (OctreeListNode<ContentT> *) std::malloc(size * sizeof(OctreeListNode<ContentT>));
					malloc_pointers.push_back(temp);
					for (int i=0; i<size; i++) {
						OctreeListNode<ContentT> *node = new (temp++) OctreeListNode<ContentT>();
						OctreeListNode<ContentT>::push(OctreeListNode<ContentT>::free_head, OctreeListNode<ContentT>::free_tail, OctreeListNode<ContentT>::free_size, node);
 						//std::cout << "Created new OctreeListNode at 0x" <<  reinterpret_cast<std::uintptr_t>(node) << " placed in memory at 0x" << reinterpret_cast<std::uintptr_t>(temp) << std::endl;
					}
					OctreeListNode<ContentT>::capacity += size;
				}

				static
				void
				freeMemory() {

				}

				static
				OctreeListNode<ContentT>*
				getFreeNode() {
					requested_objects++;
					OctreeListNode<ContentT> *temp = OctreeListNode<ContentT>::pop(OctreeListNode<ContentT>::free_head, OctreeListNode<ContentT>::free_tail, OctreeListNode<ContentT>::free_size);
					if (temp != nullptr) {
						OctreeListNode<ContentT>::push(OctreeListNode<ContentT>::used_head, OctreeListNode<ContentT>::used_tail, OctreeListNode<ContentT>::used_size, temp);
						temp->in_use = true;
					}
					return temp;
				}

				static
				void
				returnUsedNode(OctreeListNode<ContentT> *node) {
					returned_objects++;
					node->recycled = true;
					if (OctreeListNode<ContentT>::punch(OctreeListNode<ContentT>::used_head, OctreeListNode<ContentT>::used_tail, OctreeListNode<ContentT>::used_size, node)) {
						OctreeListNode<ContentT>::push(OctreeListNode<ContentT>::free_head, OctreeListNode<ContentT>::free_tail, OctreeListNode<ContentT>::free_size, node);
					}
				}

				static int used_size, free_size, capacity;
				static int requested_objects, returned_objects;

			protected:

				explicit
				OctreeListNode() = default;

				~OctreeListNode() = default;

				static
				bool
				push(OctreeListNode<ContentT> *&head , OctreeListNode<ContentT> *&tail, int &size, OctreeListNode<ContentT> *node) {
					if (node == nullptr)
						return false;
					if (head == nullptr && tail == nullptr) {
						head = node;
						tail = node;
						size++;
						return true;
					}
					/*if (tail == nullptr) {
						OctreeListNode<ContentT> *temp = head;
						head = node;
						if (temp != nullptr) {
							node->mem_next_ = temp;
						}
						size++;
						return true;
					}*/
					tail->mem_next_ = node;
					node->mem_prev_ = tail;
					tail = node;
					size++;
					return true;
				}

				static
				OctreeListNode<ContentT>*
				pop(OctreeListNode<ContentT> *&head , OctreeListNode<ContentT> *&tail, int &size) {
					if (head == nullptr)
						return nullptr;
					OctreeListNode<ContentT> *temp = head;
					// TODO  Figure out a way to return last item
					if (head->mem_next_ == nullptr)
						return nullptr;
					head->mem_next_->mem_prev_ = nullptr;
					head = head->mem_next_;
					size--;
					temp->reset();
					return temp;
				}

				static
				bool
				punch(OctreeListNode<ContentT> *&head , OctreeListNode<ContentT> *&tail, int &size, OctreeListNode<ContentT> *node) {
					if (node == nullptr)
						return false;

					if (node == head) {
						head = head->mem_next_;
					}

					if (node == tail) {
						tail = tail->mem_prev_;
					}


					OctreeListNode<ContentT> *prev = node->mem_prev_, *next = node->mem_next_;

					if (prev != nullptr) {
						prev->mem_next_ = next;
					}

					if (next != nullptr) {
						next->mem_prev_ = prev;
					}

					size--;

					node->reset();

					return true;

					/*OctreeListNode<ContentT> *curr = head, *prev = nullptr;

					while (curr != nullptr) {
						if (curr->key_ == node->key_) {
							if (prev == nullptr) {
								head = head->mem_next_;
							} else {
								prev->mem_next_ = curr->mem_next_;
							}
							size--;
							node->mem_next_ = nullptr;
							return true;
						}
						prev = curr;
						curr = curr->mem_next_;
					}
					return false;*/
				}

			private:
				u_int key_ = 0;
				ContentT *content_ = nullptr;
				OctreeListNode *next_ = nullptr;

				bool in_use = false, recycled = false;
				OctreeListNode<ContentT> *mem_next_ = nullptr, *mem_prev_ = nullptr;

				static OctreeListNode<ContentT> *used_head, *used_tail;
				static OctreeListNode<ContentT> *free_head, *free_tail;
				static std::vector<OctreeListNode<ContentT>*> malloc_pointers;
		};

		template<typename ContentT>
		int OctreeListNode<ContentT>::used_size = 0;
		template<typename ContentT>
		int OctreeListNode<ContentT>::free_size = 0;
		template<typename ContentT>
		int OctreeListNode<ContentT>::capacity = 0;

		template<typename ContentT>
		int OctreeListNode<ContentT>::requested_objects = 0;
		template<typename ContentT>
		int OctreeListNode<ContentT>::returned_objects = 0;

		template<typename ContentT>
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::used_head = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::used_tail = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::free_head = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::free_tail = nullptr;

		template<typename ContentT>
		std::vector<OctreeListNode<ContentT>*> OctreeListNode<ContentT>::malloc_pointers = std::vector<OctreeListNode<ContentT>*>();




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
					//this->clear();
				}

				/** \brief @b Octree multi-pointcloud point wrapper
				  * \returns 	0 for success
				  * 			1 for key already exists
				  * 			2 for first element inserted
				  * 			3 for unknown error
				  */
				virtual
				uint8_t
				insert(ListT* const& content) {
					//return this->insertNode(new OctreeListNode<ListT>(reinterpret_cast<std::uintptr_t>(content), content));
					OctreeListNode<ListT> *temp = OctreeListNode<ListT>::getFreeNode();
					temp->init(reinterpret_cast<std::uintptr_t>(content), content);
					return this->insertNode(temp);
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
							OctreeListNode<ListT>::returnUsedNode(curr);
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
					//std::cout << "Clearing list of " << typeid(ListT).name() << " containing " << this->size_ << " items. The memory pool contains " << OctreeListNode<ListT>::used_size << " objects" << std::endl;
					//int returned = OctreeListNode<ListT>::returned_objects; //, used = OctreeListNode<ListT>::used_size, free = OctreeListNode<ListT>::free_size, list_size = this->size_;

					int i=0;
					while (curr != nullptr) {
						OctreeListNode<ListT> *temp = curr;
						curr = curr->getNext();
						//delete temp->getContent();
						OctreeListNode<ListT>::returnUsedNode(temp);
						i++;
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

				ForwardIterator<ListT>
				begin() {
					return ForwardIterator<ListT>(this->head_);
				}

				ForwardIterator<ListT>
				end() {
					return ForwardIterator<ListT>(this->tail_);
				}


			protected:

				OctreeListNode<ListT> *head_ = nullptr, *tail_ = nullptr;
				uint64_t size_ = 0;

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

					return 0;
				}

				void
				insertStart(OctreeListNode<ListT> *const &node) {
					if (this->head_ == nullptr && this->tail_ == nullptr) {
						this->head_ = node;
						this->tail_ = node;
						this->size_++;
						return;
					}
					OctreeListNode<ListT> *temp = this->head_;
					this->head_ = node;

					node->setNext(temp);
					this->size_++;
				}

				void
				insertEnd(OctreeListNode<ListT> *const &node) {
					if (this->tail_ == nullptr) {
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
