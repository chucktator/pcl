//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_VOXEL_LIST_H
#define PCL_OCTREE_VOXEL_LIST_H

namespace pcl {
	namespace octree {

		template<typename ListT>
		class VoxelList;

		template<typename ContentT>
		class VoxelNode {
			friend class VoxelList<ContentT>;

		public:
			VoxelNode(u_int voxel_key) {
				this->voxel_key_ = voxel_key;
			}

			VoxelNode(u_int voxel_key, ContentT *content) {
				this->voxel_key_ = voxel_key;
				this->content_ = content;
			}

			~VoxelNode() = default;

		private:
			u_int voxel_key_ = 0;
			ContentT *content_ = nullptr;
			VoxelNode *next_ = nullptr;
		};

		template<typename ListT>
		class VoxelList {

			template <class Type, class UnqualifiedType = std::remove_cv_t<Type>>
			class ForwardIterator : public std::iterator<std::forward_iterator_tag, UnqualifiedType, std::ptrdiff_t, Type*, Type&> {
				friend class VoxelList;
				VoxelNode<UnqualifiedType>* itr;

				explicit ForwardIterator(VoxelNode<UnqualifiedType>* nd)
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
					itr = itr->next_;
					return *this;
				}

				ForwardIterator operator++ (int) // Post-increment
				{
					assert(itr != nullptr && "Out-of-bounds iterator increment!");
					ForwardIterator tmp(*this);
					itr = itr->next_;
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
					return itr->content_;
				}

				Type& operator-> () const
				{
					assert(itr != nullptr && "Invalid iterator dereference!");
					return *(*itr->content_);
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

			VoxelList() {
				this->head_ = nullptr;
				this->tail_ = nullptr;
			}

			~VoxelList() {
				this->clear();
			}

			uint8_t
			insert(ListT *const &content) {
				return this->insert(new VoxelNode<ListT>(reinterpret_cast<std::uintptr_t>(content), content));
			}

			/** \brief @b Octree multi-pointcloud point wrapper
			  * \returns 	0 for success
			  * 			1 for key already exists
			  * 			2 for first element inserted
			  * 			3 for unknown error
			  */
			uint8_t
			insert(VoxelNode<ListT> *const &node) {
				//int head_key, tail_key, curr_key;

				if (this->head_ == nullptr && this->tail_ == nullptr) {
					this->head_ = node;
					this->tail_ = node;
					this->size_++;
					return 2;
				}

				if (this->head_->voxel_key_ == node->voxel_key_ || this->tail_->voxel_key_ == node->voxel_key_) {
					return 1;
				}

				if (this->head_->voxel_key_ > node->voxel_key_) {
					insertStart(node);
					this->size_++;
					return 0;
				}

				if (this->tail_->voxel_key_ < node->voxel_key_) {
					this->size_++;
					insertEnd(node);
					return 0;
				}

				VoxelNode<ListT> *curr = this->head_, *curr_next;

				// TODO Divide and Conquer approach
				// TODO Possibly Doubly Linked List
				while (curr != nullptr) {
					curr_next = curr->next_;
					if (curr_next != nullptr && curr_next->voxel_key_ == node->voxel_key_)
						return 1;
					if (curr->voxel_key_ < node->voxel_key_) {
						//if (curr->next_ != nullptr && node->voxel_key_ < curr->next_->voxel_key_) {
							insertAfter(node, curr);
							this->size_++;
							return 0;
						//}
					}
					curr = curr_next;
				}

				return 3;
			}

			bool
			deleteVoxel(u_int voxel_key) {
				//std::cout << "Deleting voxel with key " << voxel_key << std::endl;
				VoxelNode<ListT> *curr = this->head_, *prev = nullptr;

				/*if (this->head_->voxel_key_ == voxel_key) {
					this->head_ = this->head_->next_;
					delete curr;
					return true;
				}*/

				while (curr != nullptr) {
					if (curr->voxel_key_ == voxel_key) {
						if (prev == nullptr) {
							this->head_ = this->head_->next_;
						} else {
							prev->next_ = curr->next_;
						}
						delete curr;
						return true;
					}
					prev = curr;
					curr = curr->next_;
				}
				return false;
			}

			ListT*
			find(u_int voxel_key) {
				VoxelNode<ListT> *curr = this->head_;

				while (curr != nullptr) {
					if (curr->voxel_key_ == voxel_key) {
						return curr->content_;
					}
					curr = curr->next_;
				}
			}

			void
			clear() {
				VoxelNode<ListT> *curr = this->head_;
				this->head_ = nullptr;
				this->tail_ = nullptr;

				int i=0;
				while (curr != nullptr) {
					VoxelNode<ListT> *temp = curr;
					curr = curr->next_;
					delete temp;
					i++;
				}
				if (this->size_ - i > 0)
					std::cout << "I have deleted " << i << " of " << this->size_ << " VoxelNodes, leaving " << (this->size_ - i) << " in memory." << std::endl;
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


		private:

			VoxelNode<ListT> *head_, *tail_;
			uint64_t size_ = 0;

			void
			insertStart(VoxelNode<ListT> *const &node) {
				VoxelNode<ListT> *temp = this->head_;
				this->head_ = node;
				if (temp != NULL)
					node->next_ = temp;
			}

			void
			insertEnd(VoxelNode<ListT> *const &node) {
				if (this->tail_ == NULL) {
					this->insertStart(node);
					return;
				}
				this->tail_->next_ = node;
				this->tail_ = node;
			}

			void
			insertBefore(VoxelNode<ListT> *const &node, VoxelNode<ListT> *const &before,
						 VoxelNode<ListT> *const &beforeThat) {
				beforeThat->next_ = node;
				node->next_ = before;
			}

			void
			insertAfter(VoxelNode<ListT> *const &node, VoxelNode<ListT> *const &after) {
				node->next_ = after->next_;
				after->next_ = node;
			}
		};
	}
}

#endif //PCL_OCTREE_VOXEL_LIST_H
