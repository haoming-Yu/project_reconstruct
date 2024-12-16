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
		bool displayAndSaveImage(const cv::Mat& image, const std::string& savePath);

		int width;
		int height;
		std::vector<char> depth_data; // raw_data
		std::vector<float> depth_float32; // converted_data
		std::vector<char> ir_data; // raw_data
	private:
		bool setup_devices();
		void remove_devices();
		bool init_sdk();
		bool get_camera_list();
		bool start_camera();
		bool select_device(int intend_id);

		// object members
		std::vector<std::string> m_camera_name;
		std::vector<int> m_all_camera_id;
		int m_selected_device_id;
};
