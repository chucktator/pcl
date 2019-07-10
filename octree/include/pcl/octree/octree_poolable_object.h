//
// Created by alex on 25.06.19.
//

#ifndef PCL_OCTREE_POOLABLE_OBJECT_H
#define PCL_OCTREE_POOLABLE_OBJECT_H

#include <pcl/octree/octree_list.h>

namespace pcl {
	namespace octree {

		template<typename T>
		class OctreeListNode;

		template <typename PoolObjectT>
		class OctreePoolableObject {

			public:

				virtual
				void
				reset() = 0;

				void
				setListNodeReference(OctreeListNode<PoolObjectT> *listNode) {
					this->listNode_ = listNode;
				}

				OctreeListNode<PoolObjectT>*
				getListNodeReference() {
					return this->listNode_;
				}

				bool
				getInUse() const {
					return in_use;
				}

				void
				setInUse(bool inUse) {
					in_use = inUse;
				}

				static
				void
				reserveMemory(uint64_t size) {
					OctreePoolableObject::pool_.reserveMemory(size);
				}

				static
				PoolObjectT*
				getFreeObject() {
					return OctreePoolableObject::pool_.getFreeObject();
				}

				static
				void
				returnUsedObject(PoolObjectT* object) {
					OctreePoolableObject::pool_.returnUsedObject(object->listNode_);
				}

			private:

				OctreeListNode<PoolObjectT> *listNode_;
				static OctreeObjectPool<PoolObjectT> pool_;
				bool in_use;

		};

		template <typename PoolObjectT>
		OctreeObjectPool<PoolObjectT> OctreePoolableObject<PoolObjectT>::pool_ = OctreeObjectPool<PoolObjectT>();
	}
}



#endif //PCL_OCTREE_POOLABLE_OBJECT_H
