//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_POINT_LIST_H
#define PCL_OCTREE_POINT_LIST_H

#include "octree_list.h"

namespace pcl {
	namespace octree {


		template<typename PointT>
		class PointList : public OctreeList<PointT> {

			public:

				PointList() : OctreeList<PointT>() {

				}

				PointList(int segments) : OctreeList<PointT>(segments) {

				}

				~PointList() = default;

		};
	}
}

#endif //PCL_OCTREE_POINT_LIST_H
