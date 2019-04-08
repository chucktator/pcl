//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_POINT_LIST_H
#define PCL_OCTREE_POINT_LIST_H

namespace pcl {
	namespace octree {

		template<typename PointT>
		class PointList;

		template<typename ContentT>
		class PointNode {
			friend class PointList<ContentT>;

		public:

			PointNode(ContentT *content) {
				this->content_ = content;
			}

			~PointNode() {
				//if (this->content_ != nullptr)
					//delete this->content_;
			}

		private:
			ContentT *content_ = nullptr;
			PointNode *next_ = nullptr;
		};

		template<typename PointT>
		class PointList {

			template <class Type, class UnqualifiedType = std::remove_cv_t<Type>>
			class ForwardIterator : public std::iterator<std::forward_iterator_tag, UnqualifiedType, std::ptrdiff_t, Type*,	Type&> {
				friend class PointList;
				PointNode<UnqualifiedType>* itr;

				explicit ForwardIterator(PointNode<UnqualifiedType>* nd)
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
			typedef ForwardIterator<PointT> iterator;
			typedef ForwardIterator<const PointT> const_iterator;

		public:

			PointList() {
				this->head_ = nullptr;
				this->tail_ = nullptr;
			}

			~PointList() {
				this->clear();
			}

			uint8_t
			insert(PointT *const &content) {
				return this->insert(new PointNode<PointT>(content));
			}

			/** \brief @b Octree multi-pointcloud point wrapper
			  * \returns 	0 for success
			  * 			1 for key already exists
			  * 			2 for first element inserted
			  * 			3 for unknown error
			  */
			uint8_t
			insert(PointNode<PointT> *const &node) {
				//int head_key, tail_key, curr_key;

				if (this->head_ == nullptr && this->tail_ == nullptr) {
					this->head_ = node;
					this->tail_ = node;
					this->size++;
					return 2;
				}

				insertEnd(node);

				return 0;
			}

			void
			clear() {
				PointNode<PointT> *curr = this->head_;
				this->head_ = nullptr;
				this->tail_ = nullptr;

				while (curr != nullptr) {
					PointNode<PointT> *temp = curr;
					curr = curr->next_;
					delete temp;
				}
				this->size = 0;
			}

			ForwardIterator<PointT>
			begin() {
				return ForwardIterator<PointT>(this->head_);
			}

			ForwardIterator<PointT>
			end() {
				return ForwardIterator<PointT>(this->tail_);
			}


		private:

			PointNode<PointT> *head_, *tail_;
			uint64_t size = 0;

			void
			insertStart(PointNode<PointT> *const &node) {
				PointNode<PointT> *temp = this->head_;
				this->head_ = node;
				if (temp != NULL)
					node->next_ = temp;
			}

			void
			insertEnd(PointNode<PointT> *const &node) {
				if (this->tail_ == NULL) {
					this->insertStart(node);
					return;
				}
				this->tail_->next_ = node;
				this->tail_ = node;
			}

			void
			insertBefore(PointNode<PointT> *const &node, PointNode<PointT> *const &before,
						 PointNode<PointT> *const &beforeThat) {
				beforeThat->next_ = node;
				node->next_ = before;
			}

			void
			insertAfter(PointNode<PointT> *const &node, PointNode<PointT> *const &after) {
				node->next_ = after->next_;
				after->next_ = node;
			}
		};
	}
}

#endif //PCL_OCTREE_POINT_LIST_H