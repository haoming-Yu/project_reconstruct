#include "oToCAM.h" // include the full interface provided by Sony
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

class depth_interface {
	public:
		depth_interface();
		~depth_interface();
		void get_depth_raw();
		void get_ir_raw();
		cv::Mat showIRImage(int width, int height, const std::vector<char>& frame);
		cv::Mat showDepthImage(int width, int height, const std::vector<char>& frame);
		cv::Mat showMaskImage(int width, int height, std::vector<unsigned char>& frame);
		bool displayAndSaveImage(const cv::Mat& image, const std::string& savePath);
		void filter();

		int width;
		int height;
		std::vector<char> depth_data; // raw_data
		std::vector<float> depth_float32; // converted_data
		std::vector<unsigned char> depth_confidence_mask; // mask of confident depth
		std::vector<char> ir_data; // raw_data
		std::vector<unsigned char> ir_unsigned_char; // converted_ir
	private:
		bool setup_devices();
		void remove_devices();
		bool init_sdk();
		bool get_camera_list();
		bool start_camera();
		bool select_device(int intend_id);
		bool set_auto_exposure(otocam_exposure_t auto_exposure);

		// object members
		std::vector<std::string> m_camera_name;
		std::vector<int> m_all_camera_id;
		int m_selected_device_id;
		otocam_exposure_t auto_exposure;
};
