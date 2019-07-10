//
// Created by alex on 13.05.19.
//

#ifndef PCL_OCTREE_LIST_H
#define PCL_OCTREE_LIST_H

#include <mutex>

namespace pcl {
	namespace octree {

		template<typename T>
		class OctreeObjectPool;

		template <typename NodeTypeT>
		class OctreeListNodePool;

		template<typename ContentT>
		class OctreeListNode {
			friend class OctreeListNodePool<ContentT>;

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
					this->resetContent();
					this->resetPointers();
				}

				inline
				void
				resetPointers() {
					this->resetListPointers();
					this->resetMemPointers();
				}

				inline
				void
				resetMemPointers() {
					this->mem_next_ = nullptr;
					this->mem_prev_ = nullptr;
				}

				inline
				void
				resetListPointers() {
					this->next_ = nullptr;
					this->prev_ = nullptr;
				}

				inline
				void
				resetContent() {
					this->key_ = 666;
					this->content_ = nullptr;
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

				inline
				OctreeListNode<ContentT>*
				getPrev() {
					return this->prev_;
				}

				inline
				void
				setPrev(OctreeListNode<ContentT>* prev) {
					this->prev_ = prev;
				}

				static
				void
				reserveMemory(uint64_t size) {
					OctreeListNode<ContentT> *temp = (OctreeListNode<ContentT> *) std::malloc(size * sizeof(OctreeListNode<ContentT>));
					malloc_pointers_.push_back(temp);
					for (int i=0; i<size; i++) {
						OctreeListNode<ContentT> *node = new (temp++) OctreeListNode<ContentT>();
						OctreeListNode<ContentT>::memPush(OctreeListNode<ContentT>::free_head_,
														  OctreeListNode<ContentT>::free_tail_,
														  OctreeListNode<ContentT>::free_size,
														  OctreeListNode<ContentT>::free_lock_, node);
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
					OctreeListNode<ContentT> *temp = OctreeListNode<ContentT>::memPop(
							OctreeListNode<ContentT>::free_head_, OctreeListNode<ContentT>::free_tail_,
							OctreeListNode<ContentT>::free_size, OctreeListNode<ContentT>::free_lock_);
					if (temp != nullptr) {
						OctreeListNode<ContentT>::memPush(OctreeListNode<ContentT>::used_head_,
														  OctreeListNode<ContentT>::used_tail_,
														  OctreeListNode<ContentT>::used_size,
														  OctreeListNode<ContentT>::used_lock_,temp);
						temp->in_use = true;
					}
					return temp;
				}

				static
				void
				returnUsedNode(OctreeListNode<ContentT> *node) {
					returned_objects++;
					node->recycled = true;
					if (OctreeListNode<ContentT>::memPunch(OctreeListNode<ContentT>::used_head_,
														   OctreeListNode<ContentT>::used_tail_,
														   OctreeListNode<ContentT>::used_size,
														   OctreeListNode<ContentT>::used_lock_, node)) {
						OctreeListNode<ContentT>::memPush(OctreeListNode<ContentT>::free_head_,
														  OctreeListNode<ContentT>::free_tail_,
														  OctreeListNode<ContentT>::free_size,
														  OctreeListNode<ContentT>::free_lock_,node);
					}
				}

				static int used_size, free_size, capacity;
				static int requested_objects, returned_objects;

				explicit
				OctreeListNode() = default;

				~OctreeListNode() = default;

			protected:


				static
				bool
				memPush(OctreeListNode<ContentT> *&head, OctreeListNode<ContentT> *&tail, int &size, std::mutex &lock,
						OctreeListNode<ContentT> *node) {


					if (node == nullptr)
						return false;

					lock.lock();
					if (head == nullptr && tail == nullptr) {
						head = node;
						tail = node;
						size++;
						lock.unlock();
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

					lock.unlock();

					return true;
				}

				static
				OctreeListNode<ContentT>*
				memPop(OctreeListNode<ContentT> *&head, OctreeListNode<ContentT> *&tail, int &size, std::mutex &lock) {

					lock.lock();

					if (head == nullptr)
						return nullptr;
					OctreeListNode<ContentT> *temp = head;
					if (head != tail) {
						if (head->mem_next_ == nullptr) {
							lock.unlock();
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

					lock.unlock();

					return temp;
				}

				static
				bool
				memPunch(OctreeListNode<ContentT> *&head, OctreeListNode<ContentT> *&tail, int &size, std::mutex &lock,
						 OctreeListNode<ContentT> *node) {

					if (node == nullptr)
						return false;

					lock.lock();

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

					lock.unlock();

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
				OctreeListNode *next_ = nullptr, *prev_ = nullptr;

				bool in_use = false, recycled = false;
				OctreeListNode<ContentT> *mem_next_ = nullptr, *mem_prev_ = nullptr;

				static OctreeListNode<ContentT> *used_head_, *used_tail_;
				static OctreeListNode<ContentT> *free_head_, *free_tail_;
				static std::vector<OctreeListNode<ContentT>*> malloc_pointers_;

				static std::mutex free_lock_, used_lock_;

				int pool_segment_;
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
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::used_head_ = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::used_tail_ = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::free_head_ = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT>* OctreeListNode<ContentT>::free_tail_ = nullptr;

		template<typename ContentT>
		std::mutex OctreeListNode<ContentT>::free_lock_;
		template<typename ContentT>
		std::mutex OctreeListNode<ContentT>::used_lock_;

		template<typename ContentT>
		std::vector<OctreeListNode<ContentT>*> OctreeListNode<ContentT>::malloc_pointers_ = std::vector<OctreeListNode<ContentT>*>();



		template <typename NodeTypeT>
		class OctreeListNodePool {

			public:

				OctreeListNodePool(int segments = 1) {
					used_head_ = new OctreeListNode<NodeTypeT>*[segments];
					used_tail_ = new OctreeListNode<NodeTypeT>*[segments];
					free_head_ = new OctreeListNode<NodeTypeT>*[segments];
					free_tail_ = new OctreeListNode<NodeTypeT>*[segments];

					used_size = new int[segments];
					free_size = new int[segments];
					capacity = new int[segments];
					requested_objects = new int[segments];
					returned_objects = new int[segments];
					for (int i=0; i<segments; i++) {
						//this->reserveMemory((size / segments * 1.1), i);
						used_head_[i] = nullptr;
						used_tail_[i] = nullptr;
						free_head_[i] = nullptr;
						free_tail_[i] = nullptr;

						used_size[i] = 0;
						free_size [i] = 0;
						capacity[i] = 0;
						requested_objects[i] = 0;
						returned_objects[i] = 0;
					}
				}

				void reserveMemory(uint64_t totalSize, int segments = 1) {
					for (int i=0; i<segments; i++) {
						this->reserveMemorySegment((int) (totalSize / segments * 1.1), i);
					}

				}

				void
				freeMemory() {

				}

				OctreeListNode<NodeTypeT>*
				getFreeNode(int segment = 0) {
					requested_objects[segment]++;

					OctreeListNode<NodeTypeT> *temp = this->memPop(this->free_head_[segment], this->free_tail_[segment], this->free_size[segment]);
					if (temp != nullptr) {
						this->memPush(this->used_head_[segment], this->used_tail_[segment], this->used_size[segment], temp);
						temp->in_use = true;
					}

					used_size[segment]++;
					free_size[segment]--;
					return temp;
				}

				void
				returnUsedNode(OctreeListNode<NodeTypeT> *node) {
					returned_objects[node->pool_segment_]++;
					node->recycled = true;
					if (this->memPunch(this->used_head_[node->pool_segment_], this->used_tail_[node->pool_segment_], this->used_size[node->pool_segment_], node)) {
						this->memPush(this->free_head_[node->pool_segment_], this->free_tail_[node->pool_segment_], this->free_size[node->pool_segment_], node);
					}
					used_size[node->pool_segment_]--;
					free_size[node->pool_segment_]++;
				}

			protected:

				void
				reserveMemorySegment(uint64_t size, int segment = 0) {
					auto *temp = (OctreeListNode<NodeTypeT> *) std::malloc(size * sizeof(OctreeListNode<NodeTypeT>));
					this->malloc_pointers_.push_back(temp);
					for (int i=0; i<size; i++) {
						auto *node = new(temp++) OctreeListNode<NodeTypeT>();
						this->memPush(this->free_head_[segment], this->free_tail_[segment], this->free_size[segment], node);
						node->pool_segment_ = segment;
					}
					this->capacity[segment] += size;
					this->totalCapacity += size;
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


		};





		template<typename ListT>
		class OctreeList {

			friend class OctreeObjectPool<ListT>;

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
					#ifndef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
						OctreeListNode<ListT> *temp = new OctreeListNode<ListT>();
					#else
						OctreeListNode<ListT> *temp = OctreeListNode<ListT>::getFreeNode();
					#endif
					temp->init(reinterpret_cast<std::uintptr_t>(content), content);
					return this->insertNode(temp);
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
					return this->insertNode(temp);
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
						if (func != nullptr) {
							func(temp->getContent());
						}
						#ifndef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
							delete temp;
						#else
							OctreeListNode<ListT>::returnUsedNode(temp);
						#endif
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

				virtual
				void
				clear(OctreeListNodePool<ListT> &olnp, std::function<void(ListT*)> func = nullptr) {
					OctreeListNode<ListT> *curr = this->head_;
					this->head_ = nullptr;
					this->tail_ = nullptr;

					int i=0;
					while (curr != nullptr) {
						OctreeListNode<ListT> *temp = curr;
						curr = curr->getNext();
						//delete temp->getContent();
						if (func != nullptr) {
							func(temp->getContent());
						}
						olnp.returnUsedNode(temp);
						i++;
					}
					if ((this->size_ - i) > 0)
						std::cout << "Leaving " << (this->size_ - i) << " orphans." << std::endl;
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

				bool
				pushNode(OctreeListNode<ListT> *node) {
					return this->insertNode(node);
				}

				OctreeListNode<ListT>*
				popNode() {
					if (this->head_ == nullptr)
						return nullptr;

					OctreeListNode<ListT> *temp = this->head_;

					if (this->head_ != this->tail_) {
						if (this->head_->getNext() == nullptr)
							return nullptr;
						this->head_->getNext()->setPrev(nullptr);
						this->head_ = this->head_->getNext();
					} else {
						this->head_ = nullptr;
						this->tail_ = nullptr;
					}

					this->size_--;
					temp->resetListPointers();
					return temp;
				}

				bool
				punchNode(OctreeListNode<ListT> *const &node) {
					if (node == nullptr)
						return false;

					if (node == this->head_) {
						this->head_ = this->head_->getNext();
					}

					if (node == this->tail_) {
						this->tail_ = this->tail_->getPrev();
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
					OctreeListNode<ListT> *curr = this->head_;

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
					this->punchNode(node);

					#ifndef OCTREE_MULTI_POINTCLOUD_LISTNODE_POOLING
						delete node;
					#else
						OctreeListNode<ListT>::returnUsedNode(node);
					#endif

					return true;
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

					temp->setPrev(node);
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
					node->setPrev(this->tail_);
					this->tail_ = node;
					this->size_++;
				}

				void
				insertBefore(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &before) {
					before->getPrev()->setNext(node);
					before->setPrev(node);
					node->setNext(before);
					this->size_++;
				}

				void
				insertAfter(OctreeListNode<ListT> *const &node, OctreeListNode<ListT> *const &after) {
					node->setNext(after->getNext());
					node->setPrev(after);
					after->setNext(node);
					this->size_++;
				}
		};
	}
}


#endif //PCL_OCTREE_LIST_H
