//
// Created by alex on 15.07.19.
//

#ifndef PCL_OCTREE_THREAD_POOL_H
#define PCL_OCTREE_THREAD_POOL_H

#include <memory>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

struct OctreeThreadPool {
	public:
		typedef std::unique_ptr<boost::asio::io_service::work> asio_worker;

		OctreeThreadPool(int threads) :service(), service_worker(new asio_worker::element_type(service)) {
			for (int i = 0; i < threads; ++i) {
				auto worker = [this] { return service.run(); };
				grp.add_thread(new boost::thread(worker));
			}
		}

		template<class F>
		void enqueue(F f) {
			service.post(f);
		}

		~OctreeThreadPool() {
			std::cout << "Destructing OctreeThreadPool!!" << std::endl;
			service_worker.reset();
			grp.join_all();
			service.stop();
		}

	private:
		boost::asio::io_service service;
		asio_worker service_worker;
		boost::thread_group grp;
};

#endif //PCL_OCTREE_THREAD_POOL_H
