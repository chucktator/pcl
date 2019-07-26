//
// Created by alex on 16.07.19.
//

#ifndef PCL_OCTREE_LIST_NODE_H
#define PCL_OCTREE_LIST_NODE_H

namespace pcl {
	namespace octree {

		template <typename NodeTypeT>
		class OctreeListNodeManager;

		template<typename ContentT>
		class OctreeListNode {
			friend class OctreeListNodeManager<ContentT>;

			public:

				inline
				void
				init(u_int key, ContentT *content) {
					key_ = key;
					content_ = content;

				}

				inline
				void
				reset() {
					resetContent();
					resetPointers();
				}

				inline
				void
				resetPointers() {
					resetListPointers();
					resetMemPointers();
				}

				inline
				void
				resetMemPointers() {
					mem_next_ = nullptr;
					mem_prev_ = nullptr;
				}

				inline
				void
				resetListPointers() {
					next_ = nullptr;
					prev_ = nullptr;
				}

				inline
				void
				resetContent() {
					key_ = 666;
					content_ = nullptr;
					in_use = false;
				}

				inline
				u_int
				getKey() {
					return key_;
				}

				inline
				ContentT*
				getContent() {
					return content_;
				}

				inline
				OctreeListNode<ContentT>*
				getNext() {
					return next_;
				}

				inline
				void
				setNext(OctreeListNode<ContentT>* next) {
					next_ = next;
				}

				inline
				OctreeListNode<ContentT>*
				getPrev() {
					return prev_;
				}

				inline
				void
				setPrev(OctreeListNode<ContentT>* prev) {
					prev_ = prev;
				}

				inline
				int
				getPoolSegment() {
					return pool_segment_;
				}

				explicit
				OctreeListNode() = default;

				~OctreeListNode() = default;


			private:
				u_int key_ = 0;
				ContentT *content_ = nullptr;
				OctreeListNode *next_ = nullptr, *prev_ = nullptr;

				bool in_use = false, recycled = false;
				OctreeListNode<ContentT> *mem_next_ = nullptr, *mem_prev_ = nullptr;

				int pool_segment_;
		};
	}
}

#endif //PCL_OCTREE_LIST_NODE_H
