//
// Created by alex on 11.07.19.
//

#ifndef PCL_OCTREE_LIST_NODE_H
#define PCL_OCTREE_LIST_NODE_H

namespace pcl {
	namespace octree {

		template<typename NodeTypeT>
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
					//this->list_segment_ = INT_MAX;
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
				ContentT *
				getContent() {
					return this->content_;
				}

				inline
				OctreeListNode<ContentT> *
				getNext() {
					return this->next_;
				}

				inline
				void
				setNext(OctreeListNode<ContentT> *next) {
					this->next_ = next;
				}

				inline
				OctreeListNode<ContentT> *
				getPrev() {
					return this->prev_;
				}

				inline
				void
				setPrev(OctreeListNode<ContentT> *prev) {
					this->prev_ = prev;
				}

				static
				void
				reserveMemory(uint64_t size) {
					OctreeListNode<ContentT> *temp = (OctreeListNode<ContentT> *) std::malloc(
							size * sizeof(OctreeListNode<ContentT>));
					malloc_pointers_.push_back(temp);
					for (int i = 0; i < size; ++i) {
						OctreeListNode<ContentT> *node = new(++temp) OctreeListNode<ContentT>();
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
				OctreeListNode<ContentT> *
				getFreeNode() {
					++requested_objects;
					OctreeListNode<ContentT> *temp = OctreeListNode<ContentT>::memPop(
							OctreeListNode<ContentT>::free_head_, OctreeListNode<ContentT>::free_tail_,
							OctreeListNode<ContentT>::free_size, OctreeListNode<ContentT>::free_lock_);
					if (temp != nullptr) {
						OctreeListNode<ContentT>::memPush(OctreeListNode<ContentT>::used_head_,
														  OctreeListNode<ContentT>::used_tail_,
														  OctreeListNode<ContentT>::used_size,
														  OctreeListNode<ContentT>::used_lock_, temp);
						temp->in_use = true;
					}
					return temp;
				}

				static
				void
				returnUsedNode(OctreeListNode<ContentT> *node) {
					++returned_objects;
					node->recycled = true;
					if (OctreeListNode<ContentT>::memPunch(OctreeListNode<ContentT>::used_head_,
														   OctreeListNode<ContentT>::used_tail_,
														   OctreeListNode<ContentT>::used_size,
														   OctreeListNode<ContentT>::used_lock_, node)) {
						OctreeListNode<ContentT>::memPush(OctreeListNode<ContentT>::free_head_,
														  OctreeListNode<ContentT>::free_tail_,
														  OctreeListNode<ContentT>::free_size,
														  OctreeListNode<ContentT>::free_lock_, node);
					}
				}

				static uint64_t used_size, free_size, capacity;
				static uint64_t requested_objects, returned_objects;

				explicit
				OctreeListNode() = default;

				~OctreeListNode() = default;

			protected:


				static
				bool
				memPush(OctreeListNode<ContentT> *&head, OctreeListNode<ContentT> *&tail, uint64_t &size, std::mutex &lock,
						OctreeListNode<ContentT> *node) {


					if (node == nullptr)
						return false;

					lock.lock();
					if (head == nullptr && tail == nullptr) {
						head = node;
						tail = node;
						++size;
						lock.unlock();
						return true;
					}
					/*if (tail == nullptr) {
						OctreeListNode<ContentT> *temp = head;
						head = node;
						if (temp != nullptr) {
							node->mem_next_ = temp;
						}
						++size;
						return true;
					}*/
					tail->mem_next_ = node;
					node->mem_prev_ = tail;
					tail = node;
					++size;

					lock.unlock();

					return true;
				}

				static
				OctreeListNode<ContentT> *
				memPop(OctreeListNode<ContentT> *&head, OctreeListNode<ContentT> *&tail, uint64_t &size, std::mutex &lock) {

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
					--size;
					temp->reset();

					lock.unlock();

					return temp;
				}

				static
				bool
				memPunch(OctreeListNode<ContentT> *&head, OctreeListNode<ContentT> *&tail, uint64_t &size, std::mutex &lock,
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

					--size;

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
							--size;
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
				static std::vector<OctreeListNode<ContentT> *> malloc_pointers_;

				static std::mutex free_lock_, used_lock_;

				int pool_segment_;
		};

		template<typename ContentT>
		uint64_t OctreeListNode<ContentT>::used_size = 0;
		template<typename ContentT>
		uint64_t OctreeListNode<ContentT>::free_size = 0;
		template<typename ContentT>
		uint64_t OctreeListNode<ContentT>::capacity = 0;

		template<typename ContentT>
		uint64_t OctreeListNode<ContentT>::requested_objects = 0;
		template<typename ContentT>
		uint64_t OctreeListNode<ContentT>::returned_objects = 0;

		template<typename ContentT>
		OctreeListNode<ContentT> *OctreeListNode<ContentT>::used_head_ = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT> *OctreeListNode<ContentT>::used_tail_ = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT> *OctreeListNode<ContentT>::free_head_ = nullptr;
		template<typename ContentT>
		OctreeListNode<ContentT> *OctreeListNode<ContentT>::free_tail_ = nullptr;

		template<typename ContentT>
		std::mutex OctreeListNode<ContentT>::free_lock_;
		template<typename ContentT>
		std::mutex OctreeListNode<ContentT>::used_lock_;

		template<typename ContentT>
		std::vector<OctreeListNode<ContentT> *> OctreeListNode<ContentT>::malloc_pointers_ = std::vector<OctreeListNode<ContentT> *>();

	}
}

#endif //PCL_OCTREE_LIST_NODE_H
