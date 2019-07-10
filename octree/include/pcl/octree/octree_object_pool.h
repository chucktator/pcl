//
// Created by alex on 25.06.19.
//

#ifndef PCL_OCTREEOBJECTPOOL_H
#define PCL_OCTREEOBJECTPOOL_H

#include <pcl/octree/octree_list.h>
#include <pcl/octree/octree_poolable_object.h>


namespace pcl {
	namespace octree {

		template<typename T>
		class OctreeListNode;

		template<typename T>
		class OctreeList;

		template<typename PoolObjectT>
		class OctreeObjectPool {

			public:

				OctreeObjectPool() {
					static_assert(std::is_base_of<OctreePoolableObject<PoolObjectT>, PoolObjectT>::value, "Derived not derived from BaseClass");
				}

				void
				reserveMemory(uint64_t size) {
					PoolObjectT *temp = (PoolObjectT *) std::malloc(size * sizeof(PoolObjectT));
					this->malloc_pointers_.push_back(temp);
					OctreeListNode<PoolObjectT>::reserveMemory(size);

					for (int i = 0; i < size; i++) {
						PoolObjectT *object = new (temp++) PoolObjectT();
						this->free_.insert(object);
						this->free_size++;
						//std::cout << "Created new OctreeListNode at 0x" <<  reinterpret_cast<std::uintptr_t>(node) << " placed in memory at 0x" << reinterpret_cast<std::uintptr_t>(temp) << std::endl;
					}
					this->capacity += size;
				}

				void
				freeMemory() {
					//  TODO  Implement memory freeing
				}

				PoolObjectT*
				getFreeObject() {
					this->requested_objects++;
					OctreeListNode<PoolObjectT> *temp = this->free_.popNode();
					PoolObjectT *content = temp->getContent();

					if (temp != nullptr) {
						this->used_.pushNode(temp);
						//if (std::is_base_of<OctreePoolableObject<PoolObjectT>, PoolObjectT>::value) {
						content->setListNodeReference(temp);
						content->setInUse(true);
						//}
					}

					this->used_size++;
					this->free_size--;

					return content;
				}

				void
				returnUsedObject(OctreeListNode<PoolObjectT> *node) {
					this->returned_objects++;
					if (this->used_.punchNode(node)) {
						this->free_.pushNode(node);
					}
					this->used_size--;
					this->free_size++;
				}

				int used_size, free_size, capacity;
				int requested_objects, returned_objects;


			private:
				OctreeList<PoolObjectT> used_;
				OctreeList<PoolObjectT> free_;

				std::vector<PoolObjectT *> malloc_pointers_;

		};
	}
}


#endif //PCL_OCTREEOBJECTPOOL_H
