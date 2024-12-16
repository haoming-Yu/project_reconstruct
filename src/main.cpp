#include <iomanip>
#include <chrono>
#include <thread>
#include "rate.h"
#include "oToCAM.h"
#include "depth_interface.h"
#include "kinectfusion.h"

#define DESIRED_FRAME_NUM 60

int main()
{
	std::cout << "point cloud interface testing!" << std::endl;
	
	depth_interface device;
	std::cout << " interface initialized" << std::endl;
 
	Rate rate(20);

	kinectfusion::CameraParameters cam_param;
	cam_param.image_width = 640;
	cam_param.image_height = 480;
	cam_param.focal_x = 579.5500601467432;
	cam_param.focal_y = 579.2731841251699;
	cam_param.principal_x = 323.36953874778385;
	cam_param.principal_y = 229.61187825621099;
	kinectfusion::GlobalConfiguration configuration;
	configuration.voxel_scale = 2.f;
	configuration.init_depth = 700.f;
	configuration.distance_threshold = 10.f;
	configuration.angle_threshold = 20.f;

	kinectfusion::Pipeline pipeline {cam_param, configuration};

	int store_idx = 0;
	while (store_idx < DESIRED_FRAME_NUM) {
		auto start = std::chrono::steady_clock::now();

		// std::this_thread::sleep_for(std::chrono::milliseconds(50));
		
		// ------------ code start ------------
		device.get_depth_raw(); // bottleneck, the max time consuming limited this to be less than 19 fps
		device.get_ir_raw();
		device.displayAndSaveImage(device.showDepthImage(device.width, device.height, device.depth_data), std::string("../sample/Depth/") + std::to_string(store_idx) + std::string(".jpg"));
		// device.displayAndSaveImage(device.showIRImage(device.width, device.height, device.ir_data), std::string("../sample/IR/") + std::to_string(store_idx) + std::string(".jpg"));
		bool success = pipeline.process_frame(cv::Mat_<float>(device.height, device.width, const_cast<float*>(device.depth_float32.data())));
		if (!success) {
    		std::cout << "Frame could not be processed" << std::endl;
		}
		// ------------ code end   ------------
		
		rate.sleep();
		auto end = std::chrono::steady_clock::now();
		auto frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "Actual frame time: " << frame_time << " ms" << std::endl;
		store_idx++;
	}

	// Retrieve camera poses
	auto poses = pipeline.get_poses();

	// Export surface mesh
	auto mesh = pipeline.extract_mesh();
	kinectfusion::export_ply("../sample/mesh.ply", mesh);

	// Export pointcloud
	auto pointcloud = pipeline.extract_pointcloud();
	kinectfusion::export_ply("../sample/pointcloud.ply", pointcloud);

    return 0;
	/*
	 * for debug: test whether the data has been read out.
	 * 
	for (int i = 0; i < device.height; ++i) {
		for (int j = 0; j < device.width; ++j) {
			std::cout << std::hex << std::setfill('0') << std::setw(2) \
			       	<< (unsigned int)(unsigned char)device.depth_data[i*device.width+j] \
				<< std::endl;
		}
	}
	 *
	*/
}