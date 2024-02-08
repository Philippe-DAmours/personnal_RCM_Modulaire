/******************************************************************************
 * Samuel Faucher [Introlab]
 *****************************************************************************/

#include "ArenaApi.h"
#include "SaveApi.h"

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

using namespace std;

//----------------------------------------------------------------------
// GLOBAL VARIABLES

#define TAB1 "  "
#define TAB2 "                                "

Arena::ISystem* pSystem = nullptr;
Arena::IDevice* pDevice = nullptr;

// pixel_format has to be Coord3D_ABCY16 for this to work
struct Pixel_raw {uint16_t x; uint16_t y; uint16_t z; uint16_t i;};
struct Pixel_pc {float x; float y; float z; uint16_t i;};
struct helios2_ros_msg {sensor_msgs::Image intensity; sensor_msgs::PointCloud2 depth;};

bool debug_print_once = false;

//----------------------------------------------------------------------
// FONCTIONS

bool searchDevice()
{
	pSystem = Arena::OpenSystem();
	pSystem->UpdateDevices(100);
	std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();

	if (deviceInfos.size() == 0)
	{
		Arena::CloseSystem(pSystem);
		pSystem = nullptr;
		return false;
	}
	else
	{
		pDevice = pSystem->CreateDevice(deviceInfos[0]);
		ROS_INFO_STREAM(deviceInfos[0].VendorName() << "'s camera is found!\n" << TAB2 <<
						"model: " << deviceInfos[0].ModelName() << "\n" << TAB2 <<
						"serial number: " << deviceInfos[0].SerialNumber() << "\n" TAB2 <<
						"IP address: " << deviceInfos[0].IpAddress());
      	return true;
	}
}

bool helios2_cam_used(Arena::IDevice* pDevice, GenApi::INodeMap* pNodeMap)
{
	bool isHelios2 = false;

	// validate if Scan3dCoordinateSelector node exists. If not - probaly not
	// Helios camera used running the example
	GenApi::CEnumerationPtr checkpCoordSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
	if (!checkpCoordSelector)
	{
		ROS_INFO_STREAM(TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used for the example.");
		return isHelios2;
	}

	// validate if Scan3dCoordinateOffset node exists. If not - probaly Helios
	// has an old firmware
	GenApi::CFloatPtr checkpCoord = pNodeMap->GetNode("Scan3dCoordinateOffset");
	if (!checkpCoord)
	{
		ROS_INFO_STREAM(TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.");
		return isHelios2;
	}

	// check if Helios2 camera used for the example
	GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	std::string deviceModelName_tmp = deviceModelName.c_str();
	if (deviceModelName_tmp.rfind("HLT", 0) == 0)
	{
		isHelios2 = true;
	}

	return isHelios2;

}

void helios2_init_node(Arena::IDevice* pDevice, ros::NodeHandle node_handle)
{
	GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

	bool isHelios2 = helios2_cam_used(pDevice, pNodeMap);  // For safety
	if(!isHelios2)
	{
		ROS_ERROR_STREAM("Wrong camera model detected. Node may not work properly.");
	}
	else
	{
		ROS_INFO_STREAM("Right camera model confirmed, camera parameters:");
	}

	// CAMERA PARAMETERS -------------------------------------------------------------------------------------------------------------
	// Init with current camera parameters
	GenICam::gcstring operatingMode = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode");
	GenICam::gcstring exposureTime = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector");
	GenICam::gcstring conversionGain = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain");
	GenICam::gcstring pixelFormat = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat");
	int64_t imageAccumulation = Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation");
	bool confidenceThreshold = Arena::GetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable");
	int64_t confidenceThresholdMin = Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdMin");
	bool spatialFilter = Arena::GetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable");

	// Update with values from launch file
	std::string operatingModeTemp = "";
	node_handle.getParam("/helios_camera_node/operating_mode", operatingModeTemp);
	operatingMode = operatingModeTemp.c_str();
	ROS_INFO_STREAM(TAB1 << "Set 3D operating mode to " << operatingMode);
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", operatingMode);

	std::string exposureTimeTemp = "";
	node_handle.getParam("/helios_camera_node/exposure_time", exposureTimeTemp);
	exposureTime = exposureTimeTemp.c_str();
	ROS_INFO_STREAM(TAB1 << "Set time selector to " << exposureTime);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", exposureTime);

	std::string conversionGainTemp = "";
	node_handle.getParam("/helios_camera_node/conversion_gain", conversionGainTemp);
	conversionGain = conversionGainTemp.c_str();
	ROS_INFO_STREAM(TAB1 << "Set conversion gain to " << conversionGain);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", conversionGain);

	pixelFormat = "Coord3D_ABCY16";  // Fix for this application
	ROS_INFO_STREAM(TAB1 << "Set " << pixelFormat << " to pixel format");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", pixelFormat);

	int imageAccumulationTemp = 1;
	node_handle.getParam("/helios_camera_node/image_accumulation", imageAccumulationTemp);
	imageAccumulation = imageAccumulationTemp;
	ROS_INFO_STREAM(TAB1 << "Set image accumulation to " << imageAccumulation);
	Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", imageAccumulation);
	
	node_handle.getParam("/helios_camera_node/confidence_threshold_enable", confidenceThreshold);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);
	if(confidenceThreshold)
	{
		int confidenceThresholdMinTemp = 1;
		node_handle.getParam("/helios_camera_node/confidence_threshold_min", confidenceThresholdMinTemp);
		confidenceThresholdMin = confidenceThresholdMinTemp;
		ROS_INFO_STREAM(TAB1 << "Enable confidence threshold, with min at " << confidenceThresholdMin);
		Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdMin", confidenceThresholdMin);
	}
	else
	{
		ROS_INFO_STREAM(TAB1 << "Disable confidence threshold");
	}

	node_handle.getParam("/helios_camera_node/spatial_threshold", spatialFilter);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", spatialFilter);
	if(spatialFilter)
	{
		ROS_INFO_STREAM(TAB1 << "Enable spatial threshold");
	}
	else
	{
		ROS_INFO_STREAM(TAB1 << "Disable spatial threshold");
	}

	// for simplicity
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);  // enable stream auto negotiate packet size	
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);  // enable stream packet resend

	// Extra informations:
	GenICam::gcstring dist_unit = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dDistanceUnit");
	ROS_INFO_STREAM(TAB1 << "Unit: " << dist_unit);
	GenICam::gcstring coord_sys = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSystem");
	ROS_INFO_STREAM(TAB1 << "Coordinate system: " << coord_sys);

	// Add ros param usefull in the algo node
	int64_t intensity_width = Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "Width");
	node_handle.setParam("/helios_camera_node/intensity_width", int(intensity_width));
	ROS_INFO_STREAM(TAB1 << "Intensity image width: " << intensity_width);

	int64_t intensity_height = Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "Height");
	node_handle.setParam("/helios_camera_node/intensity_height", int(intensity_height));
	ROS_INFO_STREAM(TAB1 << "Intensity image height: " << intensity_height);	

	pDevice->StartStream();


}

void helios2_get_data(Arena::IDevice* pDevice, ros::NodeHandle node_handle, helios2_ros_msg &to_publish)
{

	double scale = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale");

	std::string imageEncoding = sensor_msgs::image_encodings::MONO16;  // Fix for this application

	Arena::IImage* pImage = pDevice->GetImage(2000);
	uint32_t image_height = pImage->GetHeight();
	uint32_t image_width = pImage->GetWidth();

	// http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
	sensor_msgs::Image rosImage;
	rosImage.header.stamp = ros::Time::now();  // time
	rosImage.header.frame_id = "camera_frame";  // string
	rosImage.height = image_height;  // uint32
	rosImage.width = image_width;  // uint32
	rosImage.encoding = imageEncoding;  // string
	rosImage.step = rosImage.width * sizeof(uint16_t);  // uint32
	rosImage.data.resize(rosImage.height * rosImage.step);

	// http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html
	sensor_msgs::PointCloud2 rosPointCloud;
	rosPointCloud.header.stamp = rosImage.header.stamp;  // time => indentical to make shure they are synchronize
	rosPointCloud.header.frame_id = rosImage.header.frame_id ;  // string
	rosPointCloud.height = image_height;  // uint32
	rosPointCloud.width = image_width;  // uint32
	rosPointCloud.point_step = sizeof(Pixel_pc);  // uint32
	rosPointCloud.row_step = rosPointCloud.point_step * rosPointCloud.width;  // uint32
	rosPointCloud.data.resize(rosPointCloud.height * rosPointCloud.width * rosPointCloud.point_step);
	rosPointCloud.is_dense = true;

	rosPointCloud.fields.resize(4);
	rosPointCloud.fields[0].name = "x";
	rosPointCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	rosPointCloud.fields[0].offset = 0;
	rosPointCloud.fields[0].count = 1;

	rosPointCloud.fields[1].name = "y";
	rosPointCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	rosPointCloud.fields[1].offset = 4;
	rosPointCloud.fields[1].count = 1;

	rosPointCloud.fields[2].name = "z";
	rosPointCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	rosPointCloud.fields[2].offset = 8;
	rosPointCloud.fields[2].count = 1;

	rosPointCloud.fields[3].name = "i";
	rosPointCloud.fields[3].datatype = sensor_msgs::PointField::UINT16;
	rosPointCloud.fields[3].offset = 12;
	rosPointCloud.fields[3].count = 1;

	Pixel_raw *raw_from_cam = (Pixel_raw*)pImage->GetData();  // freeze data
	Pixel_pc *pc_inprogress = (Pixel_pc*)&rosPointCloud.data[0];
	uint16_t *image_intensity = (uint16_t*)&rosImage.data[0];
	for (int i = 0; i < image_height*image_width; ++i)
	{
		int half_size = (pow(2, (sizeof(raw_from_cam[i].x))*8)/2);  // should be identical for y
		image_intensity[i] = raw_from_cam[i].i;
		pc_inprogress[i].x = ((int(raw_from_cam[i].x)-half_size) * scale)/1000.0;
		pc_inprogress[i].y = ((int(raw_from_cam[i].y)-half_size) * scale)/1000.0;
		pc_inprogress[i].z = (raw_from_cam[i].z * scale)/1000.0;
		pc_inprogress[i].i = raw_from_cam[i].i;
	}

	pDevice->RequeueBuffer(pImage);  // release data

	// Write output in pointer
	to_publish.intensity = rosImage;
	to_publish.depth = rosPointCloud;
}

//----------------------------------------------------------------------
// MAIN

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "helios2");
	ros::NodeHandle n;

	bool offline = false;
	n.getParam("/sensor_used/offline", offline);

	if(offline == false)
	{
		// Tell ROS how fast to run
		int rate = 5;
		n.getParam("/helios_camera_node/node_rate", rate);
		ros::Rate r(rate);  // Hz

		ros::Publisher intensity_helios2_pub = n.advertise<sensor_msgs::Image>("intensity_helios2", 1000);
		ros::Publisher depth_helios2_pub = n.advertise<sensor_msgs::PointCloud2>("depth_helios2", 1000);

		//----------------------------------------------------------------------
		ROS_INFO_STREAM("Searching for a camera");

		bool device_found = false;
		device_found = searchDevice();

		// wait and retry until a camera is present
		ros::Time end = ros::Time::now() + ros::Duration(15.0);
		while (ros::ok() && device_found == false)
		{
			device_found = searchDevice();
			if (ros::Time::now() > end)
			{
				ROS_WARN_STREAM("No camera present. Keep waiting ...");
				end = ros::Time::now() + ros::Duration(10.0);
			}
			r.sleep();
			ros::spinOnce();
		}

		//----------------------------------------------------------------------
		ROS_INFO_STREAM("Starting node with found camera");

		helios2_init_node(pDevice, n);
		helios2_ros_msg current_data;
		ROS_INFO_STREAM("Camera node up and running");

		while (n.ok())
		{
			helios2_get_data(pDevice, n, current_data);
			intensity_helios2_pub.publish(current_data.intensity);
			depth_helios2_pub.publish(current_data.depth);
			ros::spinOnce();
			r.sleep();
		}

		// clean up node
		pDevice->StopStream();
		pSystem->DestroyDevice(pDevice);
		pDevice = nullptr;
		Arena::CloseSystem(pSystem);
		pSystem = nullptr;
	}
	else
	{
		ROS_WARN_STREAM("Offline mode selected. Camera node shutdown");
	}

	return 0;
}