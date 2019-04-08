//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_DEVICE_H
#define PCL_OCTREE_MULTI_POINTCLOUD_DEVICE_H

namespace pcl {
	namespace octree {
		struct SCDevice {
		public:

			int device_id = SCDevice::id++;
			std::string type;
			std::string identifier;
			int point_count;

		private:
			static int id;

		};

		int SCDevice::id = 0;
	}
}

#endif //PCL_OCTREE_MULTI_POINTCLOUD_DEVICE_H
