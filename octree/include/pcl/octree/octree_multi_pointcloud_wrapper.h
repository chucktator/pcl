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
					this->point_ = point;
					this->device_ = device;
					this->cloud_ = cloud;
				}

				/** \brief Class deconstructor. */
				~OctreeMultiPointCloudPointWrapper () {
					this->reset();
				}

				virtual
				void
				reset() override {
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
				SCDevice *device_ = nullptr;
				PointT *point_ = nullptr;
				PointCloud<PointT> *cloud_ = nullptr;

		};

	}
}

#endif //PCL_OCTREE_MULTI_POINTCLOUD_WRAPPER_H
