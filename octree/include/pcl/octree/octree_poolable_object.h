//
// Created by alex on 25.06.19.
//

#ifndef PCL_OCTREE_POOLABLE_OBJECT_H
#define PCL_OCTREE_POOLABLE_OBJECT_H

#include <pcl/octree/octree_list_node.h>

namespace pcl {
	namespace octree {

		template <typename PoolObjectT>
		class OctreePoolableObject {

			public:

				virtual
				void
				reset() = 0;

				void
				setListNodeReference(OctreeListNode<PoolObjectT> *listNode) {
					listNode_ = listNode;
				}

				OctreeListNode<PoolObjectT>*
				getListNodeReference() {
					return listNode_;
				}

				bool
				getInUse() const {
					return in_use_;
				}

				void
				setInUse(bool inUse) {
					in_use_ = inUse;
				}

				static
				void
				initPool(OCTREE_MEMORY_STRATEGY strategy, int threads = 1) {
					OctreePoolableObject::pool_ = new OctreeObjectPool<PoolObjectT>(threads, strategy);
				}

				static
				void
				reserveMemory(uint64_t size) {
					OctreePoolableObject::pool_->reserveMemory(size);
				}

				static
				PoolObjectT*
				getFreeObject(int threadNum = 0) {
					PoolObjectT *temp = OctreePoolableObject::pool_->getFreeObject(threadNum);
					temp->segment_ = threadNum;
					return temp;
				}

				static
				void
				returnUsedObject(PoolObjectT* object) {
					OctreePoolableObject::pool_->returnUsedObject(object, object->segment_);
				}

			private:

				OctreeListNode<PoolObjectT> *listNode_;
				static OctreeObjectPool<PoolObjectT> *pool_;
				bool in_use_;
				int segment_;

		};

		template <typename PoolObjectT>
		OctreeObjectPool<PoolObjectT> *OctreePoolableObject<PoolObjectT>::pool_ = nullptr;
		//OctreeObjectPool<PoolObjectT> OctreePoolableObject<PoolObjectT>::pool_ = OctreeObjectPool<PoolObjectT>();
	}
}



#endif //PCL_OCTREE_POOLABLE_OBJECT_H
