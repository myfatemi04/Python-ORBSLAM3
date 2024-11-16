#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include "System.h"
#include <opencv2/core/core.hpp>

namespace py = pybind11;

// Helper function to convert numpy array to cv::Mat
cv::Mat numpy_to_mat(py::array_t<unsigned char> array)
{
	if (array.ndim() == 2)
		return cv::Mat(array.shape(0), array.shape(1), CV_8UC1, (unsigned char *)array.data());
	else if (array.ndim() == 3)
		return cv::Mat(array.shape(0), array.shape(1), CV_8UC3, (unsigned char *)array.data());
	throw std::runtime_error("Unsupported array dimensions");
}

// Helper function to convert numpy float array to cv::Mat
cv::Mat numpy_float_to_mat(py::array_t<float> array)
{
	return cv::Mat(array.shape(0), array.shape(1), CV_32F, (float *)array.data());
}

PYBIND11_MODULE(_C, m)
{
	m.doc() = "Python bindings for ORB-SLAM3";

	// Bind the Verbose class
	py::enum_<ORB_SLAM3::Verbose::eLevel>(m, "VerbosityLevel")
			.value("QUIET", ORB_SLAM3::Verbose::VERBOSITY_QUIET)
			.value("NORMAL", ORB_SLAM3::Verbose::VERBOSITY_NORMAL)
			.value("VERBOSE", ORB_SLAM3::Verbose::VERBOSITY_VERBOSE)
			.value("VERY_VERBOSE", ORB_SLAM3::Verbose::VERBOSITY_VERY_VERBOSE)
			.value("DEBUG", ORB_SLAM3::Verbose::VERBOSITY_DEBUG)
			.export_values();

	// Bind the System::eSensor enum
	py::enum_<ORB_SLAM3::System::eSensor>(m, "Sensor")
			.value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
			.value("STEREO", ORB_SLAM3::System::STEREO)
			.value("RGBD", ORB_SLAM3::System::RGBD)
			.value("IMU_MONOCULAR", ORB_SLAM3::System::IMU_MONOCULAR)
			.value("IMU_STEREO", ORB_SLAM3::System::IMU_STEREO)
			.value("IMU_RGBD", ORB_SLAM3::System::IMU_RGBD)
			.export_values();

	// Bind IMU::Point struct
	py::class_<ORB_SLAM3::IMU::Point>(m, "IMUPoint")
			.def(py::init<const float, const float, const float, const float, const float, const float, const double>(),
					 py::arg("ax"), py::arg("ay"), py::arg("az"),
					 py::arg("wx"), py::arg("wy"), py::arg("wz"),
					 py::arg("t"));

	// Bind the System class
	py::class_<ORB_SLAM3::System>(m, "System")
			.def(py::init<const string &, const string &, const ORB_SLAM3::System::eSensor, const int, const string &>(),
					 py::arg("vocab_file"),
					 py::arg("settings_file"),
					 py::arg("sensor"),
					 py::arg("init_frame") = 0,
					 py::arg("sequence") = std::string())

			.def("track_stereo", [](ORB_SLAM3::System &self, py::array_t<unsigned char> left_img, py::array_t<unsigned char> right_img, double timestamp, const std::vector<ORB_SLAM3::IMU::Point> &imu_meas = std::vector<ORB_SLAM3::IMU::Point>(), const std::string &filename = "")
					 { return self.TrackStereo(numpy_to_mat(left_img),
																		 numpy_to_mat(right_img),
																		 timestamp,
																		 imu_meas,
																		 filename); }, py::arg("left_img"), py::arg("right_img"), py::arg("timestamp"), py::arg("imu_meas") = std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename") = "")

			.def("track_rgbd", [](ORB_SLAM3::System &self, py::array_t<unsigned char> rgb_img, py::array_t<float> depth_map, double timestamp, const std::vector<ORB_SLAM3::IMU::Point> &imu_meas = std::vector<ORB_SLAM3::IMU::Point>(), const std::string &filename = "")
					 { return self.TrackRGBD(numpy_to_mat(rgb_img),
																	 numpy_float_to_mat(depth_map),
																	 timestamp,
																	 imu_meas,
																	 filename); }, py::arg("rgb_img"), py::arg("depth_map"), py::arg("timestamp"), py::arg("imu_meas") = std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename") = "")

			.def("track_monocular", [](ORB_SLAM3::System &self, py::array_t<unsigned char> img, double timestamp, const std::vector<ORB_SLAM3::IMU::Point> &imu_meas = std::vector<ORB_SLAM3::IMU::Point>(), const std::string &filename = "")
					 { return self.TrackMonocular(numpy_to_mat(img),
																				timestamp,
																				imu_meas,
																				filename); }, py::arg("img"), py::arg("timestamp"), py::arg("imu_meas") = std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename") = "")

			.def("activate_localization_mode", &ORB_SLAM3::System::ActivateLocalizationMode)
			.def("deactivate_localization_mode", &ORB_SLAM3::System::DeactivateLocalizationMode)
			.def("map_changed", &ORB_SLAM3::System::MapChanged)
			.def("reset", &ORB_SLAM3::System::Reset)
			.def("reset_active_map", &ORB_SLAM3::System::ResetActiveMap)
			.def("shutdown", &ORB_SLAM3::System::Shutdown)
			.def("is_shutdown", &ORB_SLAM3::System::isShutDown)

			.def("save_trajectory_tum", &ORB_SLAM3::System::SaveTrajectoryTUM)
			.def("save_keyframe_trajectory_tum", &ORB_SLAM3::System::SaveKeyFrameTrajectoryTUM)
			.def("save_trajectory_euroc", py::overload_cast<const string &>(&ORB_SLAM3::System::SaveTrajectoryEuRoC))
			.def("save_keyframe_trajectory_euroc", py::overload_cast<const string &>(&ORB_SLAM3::System::SaveKeyFrameTrajectoryEuRoC))
			.def("save_trajectory_kitti", &ORB_SLAM3::System::SaveTrajectoryKITTI)

			.def("get_tracking_state", &ORB_SLAM3::System::GetTrackingState)
			.def("get_tracked_mappoints", &ORB_SLAM3::System::GetTrackedMapPoints)
			.def("get_tracked_keypoints", &ORB_SLAM3::System::GetTrackedKeyPointsUn)

			.def("get_time_from_imu_init", &ORB_SLAM3::System::GetTimeFromIMUInit)
			.def("is_lost", &ORB_SLAM3::System::isLost)
			.def("is_finished", &ORB_SLAM3::System::isFinished)
			.def("change_dataset", &ORB_SLAM3::System::ChangeDataset)
			.def("get_image_scale", &ORB_SLAM3::System::GetImageScale);
}