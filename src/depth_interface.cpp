#include "depth_interface.h"
#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <cmath>

#define DEPTH_MIN 10
#define DEPTH_MAX 5000
#define COMFIDENT_GAMMA         ( 1/2.2 )
#define COMFIDENT_MAX_VALUE     ( 8191.0 ) //0x1FFF

depth_interface::depth_interface()
{
	bool res = depth_interface::setup_devices();
	if (res == false) {
		std::cerr << "Device Setup Failed! Exiting..." << std::endl;
		exit(-1);
	}

	// skip first blank frame to get the first frame.
	this->get_depth_raw();
	this->get_depth_raw();
	this->get_depth_raw();
	this->get_depth_raw();
}

depth_interface::~depth_interface()
{
	depth_interface::remove_devices();
}

/*
 * As for this function, we call the interface provided 
 * by Sony, and thus setup the only depth mapping device.
 * Get the device ready for data processing.
 *
 * We only need depth image in this interface, thus just
 * get the device ready for depth processing.
 */

bool depth_interface::setup_devices()
{
	if (!depth_interface::init_sdk()) {
		std::cerr << "[Error]: init_sdk() failed" << std::endl;	
		return false;
	}	

	if (!depth_interface::get_camera_list()) {
		std::cerr << "[Error]: get_camera_list() failed" << std::endl;
		return false;
	}

	// select an device to process, default to use 0 as the device number
	// cause the system is designed as having only one depth mapping device
	if (!depth_interface::select_device(this->m_all_camera_id[0])) {
		std::cerr << "[Error]: select_device(intend_id) failed" << std::endl;
		return false;
	}

	if (!depth_interface::start_camera()) {
		std::cerr << "[Error]: start_camera() failed" << std::endl;
		return false;
	}

	return true;
}

/*
 * This function is used to remove all devices when doing the destructor funcions.
 */
void depth_interface::remove_devices()
{
	otocam_disconnect_all_camera();
}

bool depth_interface::init_sdk()
{
	int res = otocam_sdk_initialize();
	if (res != OTOCAM_OK) {
		std::cerr << "[Error]: otocam_sdk_initialize failed!" << std::endl;
		return false;
	}

	return true;
}

bool depth_interface::get_camera_list()
{
	int count = otocam_get_camera_count();
	
	this->m_all_camera_id.clear(); // ensure the camera ids are clear
	this->m_camera_name.clear(); // ensure the name vector is clear

	for (int idx = 0; idx < count; ++idx) {
		this->m_all_camera_id.push_back(otocam_get_camera_id(idx));
		std::cout << "otocam_get_camera_id: " << otocam_get_camera_id(idx) << std::endl;
		this->m_camera_name.push_back(std::string(otocam_get_camera_name(idx)));
		std::cout << "otocam_get_camera_name: " << otocam_get_camera_name(idx) << std::endl;
	}

	return true;
}

bool depth_interface::select_device(int intend_id)
{
	for (unsigned int idx = 0; idx < this->m_all_camera_id.size(); ++idx) {
		if (intend_id == this->m_all_camera_id[idx]) {
			this->m_selected_device_id = intend_id;
			return true;
		}
	}

	return false;
}

bool depth_interface::start_camera()
{
	int connect = otocam_connect_camera(this->m_selected_device_id);
	int mode_setting = otocam_switch_camera_distance_mode(this->m_selected_device_id, OTOCAM_MODE_FAR);
	int scenario_setting = otocam_switch_camera_scenario(this->m_selected_device_id, OTOCAM_SCENARIO_OUTDOOR);
	int res = otocam_start_stream(this->m_selected_device_id);
	if (connect != OTOCAM_OK || mode_setting != OTOCAM_OK || scenario_setting != OTOCAM_OK || res != OTOCAM_OK) {
		std::cerr << "[Error]: otocam_start_stream failed!" << std::endl;
		return false;
	}

	return true;
}

void depth_interface::get_depth_raw()
{
	otocam_frame_t frame_t;

	int status = otocam_get_camera_status(m_selected_device_id);
	if (status == OTOCAM_STATUS_STREAM_ON) {
		frame_t = otocam_wait_new_frame(m_selected_device_id);
		if (frame_t.length == 0) {
			std::cerr << "[Error]: No Input Data." << std::endl;
			return;
		}
		
		this->depth_data.reserve(frame_t.width * frame_t.height * sizeof(short));
		this->depth_float32.reserve(frame_t.width * frame_t.height * sizeof(float));

		int error;
		otocam_frame_t frame_depth = otocam_extract_frame(frame_t, OTOCAM_FRAME_DATA_DEPTH, &error);
		unsigned short* depth_data_address = (unsigned short*)this->depth_data.data();
		memcpy(depth_data_address, frame_depth.data, frame_t.width * frame_t.height * sizeof(short));
		this->width = frame_t.width;
		this->height = frame_t.height;
		const unsigned short* depth_src = reinterpret_cast<const unsigned short*>(this->depth_data.data());
		for (int i = 0; i < this->height; ++i) {
			for (int j = 0; j < this->width; ++j) {
				this->depth_float32[i * this->width + j] = (float)depth_src[i * this->width + j];
			}
		}
	} else {
		std::cerr << "[Error]: Camera Stopped, stream not started" << std::endl;
		return;
	}
}


void depth_interface::get_ir_raw()
{
	otocam_frame_t frame_t;

	int status = otocam_get_camera_status(m_selected_device_id);
	if (status == OTOCAM_STATUS_STREAM_ON) {
		frame_t = otocam_wait_new_frame(m_selected_device_id);
		if (frame_t.length == 0) {
			std::cerr << "[Error]: No Input Data" << std::endl;
			return;
		}

		this->ir_data.reserve(frame_t.width * frame_t.height * sizeof(short));

		int error;
		otocam_frame_t frame_ir = otocam_extract_frame(frame_t, OTOCAM_FRAME_DATA_IR, &error);
		unsigned short* ir_data_address = (unsigned short*)this->ir_data.data();
		memcpy(ir_data_address, frame_ir.data, frame_t.width * frame_t.height * sizeof(short));
		this->width = frame_t.width;
		this->height = frame_t.height;
	} else {
		std::cerr << "[Error]: Camera Stopped, stream not started" << std::endl;
		return;
	}
}

cv::Mat depth_interface::showIRImage(int width, int height, const std::vector<char>& frame)
{
    unsigned short* confident_src = (unsigned short*)frame.data();

    static unsigned char *confident_filter = NULL;
    if( confident_filter != NULL ) free( confident_filter );
    confident_filter = (unsigned char*) malloc( width * height * sizeof(unsigned char) );

    //convert confident data to grayscale
    for( int i=0; i < width*height; i++ ){
        float gamma = COMFIDENT_GAMMA;
        float filter = ( ( confident_src[ i ] >> 3 ) / COMFIDENT_MAX_VALUE );

        confident_filter[i] = (unsigned char)( pow( filter, gamma ) * 255.0 );
    }

    cv::Mat gray_image(height, width, CV_8UC1, confident_filter);
    
	return gray_image;
}

cv::Mat depth_interface::showDepthImage(int width, int height, const std::vector<char>& frame) 
{
    const unsigned short* depth_src = reinterpret_cast<const unsigned short*>(frame.data());
    cv::Mat depth_mat(height, width, CV_16UC1, const_cast<unsigned short*>(depth_src));
    cv::Mat depth_visual;
    depth_mat.convertTo(depth_visual, CV_8UC1, 255.0 / (DEPTH_MAX - DEPTH_MIN), -DEPTH_MIN * 255.0 / (DEPTH_MAX - DEPTH_MIN));

    cv::threshold(depth_visual, depth_visual, 255, 255, cv::THRESH_TRUNC);
    cv::threshold(depth_visual, depth_visual, 0, 0, cv::THRESH_TOZERO);

    return depth_visual;
}

bool depth_interface::displayAndSaveImage(const cv::Mat& image, const std::string& savePath) 
{
    if (image.empty()) {
        std::cerr << "Error: Image is empty. Cannot display or save." << std::endl;
        return false;
    }

    if (cv::imwrite(savePath, image)) {
        std::cout << "Image saved successfully to: " << savePath << std::endl;
		return true;
    } else {
        std::cerr << "Error: Failed to save image to: " << savePath << std::endl;
		return false;
    }
}