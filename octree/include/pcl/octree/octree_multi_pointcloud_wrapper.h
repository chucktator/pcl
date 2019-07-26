//
// Created by alex on 08.04.19.
//

#ifndef PCL_OCTREE_MULTI_POINTCLOUD_WRAPPER_H
#define PCL_OCTREE_MULTI_POINTCLOUD_WRAPPER_H

#include <pcl/octree/octree_multi_pointcloud_device.h>
#include <pcl/octree/octree_object_pool.h>
#include <pcl/octree/octree_poolable_object.h>

namespace pcl {
	namespace octree {

		/** \brief @b Octree multi-pointcloud point wrapper
		  * \note This class implements a wrapper that stores a point from a pointcloud together with its's corresponding device.
		  * \author Alexander Poeppel (poeppel@isse.de)
		  */
		template<typename PointT>
		class OctreeMultiPointCloudPointWrapper : public OctreePoolableObject<OctreeMultiPointCloudPointWrapper<PointT>> {
			public:

				/** \brief Class initialization. */
				OctreeMultiPointCloudPointWrapper() = default;

				void
				init(PointT *point) {
					init(point, nullptr, nullptr);
				}

				void
				init(PointT *point, SCDevice *device) {
					init(point, device, nullptr);
				}

				void
				init(PointT *point, SCDevice *device, PointCloud<PointT> *cloud) {
					point_ = point;
					device_ = device;
					cloud_ = cloud;
				}

				/** \brief Class deconstructor. */
				~OctreeMultiPointCloudPointWrapper () {
					reset();
				}

				virtual
				void
				reset() override {
					point_ = nullptr;
					device_ = nullptr;
					cloud_ = nullptr;
				}


				SCDevice*
				getDevice() {
					return device_;
				}

				PointT*
				getPoint() {
					return point_;
				}

				PointCloud<PointT>*
				getPointCloud() {
					return cloud_;
				}




			private:
				SCDevice *device_ = nullptr;
				PointT *point_ = nullptr;
				PointCloud<PointT> *cloud_ = nullptr;

		};

	}
}

#endif //PCL_OCTREE_MULTI_POINTCLOUD_WRAPPER_H
