//
// Created by alex on 01.07.19.
//

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_BRANCH_CONTAINER_H
#define PCL_OCTREE_MULTI_POINTCLOUD_BRANCH_CONTAINER_H

#include "octree_container.h"
#include <mutex>
#include <cassert>

namespace pcl {
	namespace octree {


		class OctreeMultiPointCloudBranchContainer : public OctreeContainerEmpty {

				void
				lockBranch() {
					this->mutex_.lock();
				}

				void
				releaseBranch() {
					this->mutex_.unlock();
				}

			private:

				mutable std::mutex mutex_;

		};
	}
}

#endif //PCL_OCTREE_MULTI_POINTCLOUD_BRANCH_CONTAINER_H
