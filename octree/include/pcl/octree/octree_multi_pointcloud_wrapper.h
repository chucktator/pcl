//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_WRAPPER_H
#define PCL_OCTREE_MULTI_POINTCLOUD_WRAPPER_H

#include <pcl/octree/octree_multi_pointcloud_device.h>

namespace pcl {
	namespace octree {

		/** \brief @b Octree multi-pointcloud point wrapper
		  * \note This class implements a wrapper that stores a point from a pointcloud together with its's corresponding device.
		  * \author Alexander Poeppel (poeppel@isse.de)
		  */
		template<typename PointT>
		class OctreeMultiPointCloudPointWrapper
		{
		public:
			/** \brief Class initialization. */
			OctreeMultiPointCloudPointWrapper (PointT *point) {
				this(point, NULL, NULL);
			}

			OctreeMultiPointCloudPointWrapper (PointT *point, SCDevice *device) {
				this(point, device, NULL);
			}

			OctreeMultiPointCloudPointWrapper (PointT *point, SCDevice *device, PointCloud<PointT> *cloud) {
				this->point_ = point;
				this->device_ = device;
				this->cloud_ = cloud;
			}

			/** \brief Empty class deconstructor. */
			~OctreeMultiPointCloudPointWrapper () {
				this->point_ = nullptr;
				this->device_ = nullptr;
				this->cloud_ = nullptr;
			}


			SCDevice*
			getDevice() {
				return this->device_;
			}

			PointT*
			getPoint() {
				return this->point_;
			}

			PointCloud<PointT>*
			getPointCloud() {
				return this->cloud_;
			}




		private:
			SCDevice *device_;
			PointT *point_;
			PointCloud<PointT> *cloud_;
		};

	}
}

#endif //PCL_OCTREE_MULTI_POINTCLOUD_WRAPPER_H