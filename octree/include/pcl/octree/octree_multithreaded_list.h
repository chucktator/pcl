//
// Created by alex on 16.07.19.
//

#ifndef PCL_OCTREE_MULTITHREADED_LIST_H
#define PCL_OCTREE_MULTITHREADED_LIST_H

#include <pcl/octree/octree_list.h>
#include <vector>
#include <functional>

namespace pcl {
	namespace octree {

		template <typename ListT>
		class OctreeMultiThreadedList {

			public:

				OctreeMultiThreadedList(OctreeListNodeManager<ListT>* const& manager, int threads = 1) {
					if (threads <= 0)
						threads = 1;

					lists_ = new OctreeList<ListT>*[threads];

					threads_ = threads;
					manager_ = manager;

					for (int i=0; i<threads; ++i) {
						//lists_.push_back(new OctreeList<ListT>(manager));
						lists_[i] = new OctreeList<ListT>(manager);
					}
				}

				void
				clear(std::function<void(ListT*)> contentFunction = nullptr) {
					for (int i=0; i<threads_; ++i) {
						//lists_.at(i)->clear(contentFunction);
						lists_[i]->clear(contentFunction);
					}
				}

				void
				clearSegment(int segment, std::function<void(ListT*)> contentFunction = nullptr) {
					lists_[segment]->clear(contentFunction);
				}

				inline
				bool
				segmentCleared(int threadNum = 0) {
					return lists_[threadNum]->cleared_;
				}

				inline
				bool
				segmentBeingCleared(int threadNum = 0) {
					return lists_[threadNum]->clearing_;
				}

				inline
				bool
				segmentIsClear(int threadNum = 0) {
					return segmentCleared(threadNum) || segmentBeingCleared(threadNum);
				}

				inline
				bool
				isClear(int segment = -1) {
					bool ret = true;
					if (segment >= 0) {
						for (int i = 0; i < threads_; ++i) {
							ret = ret && segmentIsClear(i);
						}
					} else {
						ret = segmentIsClear(segment);
					}
					return ret;
				}

				void
				insert(ListT* const& content, int threadNum = 0) {
					//lists_.at(threadNum)->insert(content, threadNum);
					lists_[threadNum]->insert(content, threadNum);
				}

				void
				pushNode(OctreeListNode<ListT>* const&node, int threadNum = 0) {
					//lists_.at(threadNum)->pushNode(node);
					lists_[threadNum]->pushNode(node);
				}

				bool
				punchNode(OctreeListNode<ListT>* const& node, int threadNum = 0) {
					//return lists_.at(threadNum)->punchNode(node);
					return lists_[threadNum]->punchNode(node);
				}

				OctreeListNode<ListT>*
				popNode(int threadNum = 0) {
					//return lists_.at(threadNum)->popNode();
					return lists_[threadNum]->popNode();
				}

				void
				foreach(std::function<void(ListT*)> contentFunction) {
					if (contentFunction != nullptr) {
						//for (OctreeList<ListT>* list : lists_) {
						for (int i=0; i<threads_; ++i) {
							OctreeList<ListT>* list = lists_[i];
							for (auto item = list->begin(), end = list->end(); item != end; ++item) {
									contentFunction(*item);
							}
						}
					}
				}

				void
				foreachList(std::function<void(OctreeList<ListT>*)> contentFunction) {
					if (contentFunction != nullptr) {
						for (int i=0; i<threads_; ++i) {
							contentFunction(lists_[i]);
						}
					}
				}

			private:
				int threads_;
				OctreeListNodeManager<ListT>* manager_;
				//std::vector<OctreeList<ListT>*> lists_;
				OctreeList<ListT>** lists_;

		};
	}
}

#endif //PCL_OCTREE_MULTITHREADED_LIST_H
