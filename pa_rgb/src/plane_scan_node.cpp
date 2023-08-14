/*
Samuel Faucher
Display cloud points and highlight planes with data from lidar
Requires: roslaunch velodyne_pointcloud VLP16_points.launch
*/

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

using namespace std;

bool new_point_cloud = false;

class PointCloudFromRos
{
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr ros_cloud;
public:
  PointCloudFromRos();
  ~PointCloudFromRos();
  void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr getcloud();
};

PointCloudFromRos::PointCloudFromRos() : ros_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

PointCloudFromRos::~PointCloudFromRos()
{
}

void PointCloudFromRos::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  // ros_cloud += msg;  // No overload available
  ros_cloud->width = msg->width;
  ros_cloud->height = msg->height;
  ros_cloud->points = msg->points;
  ros_cloud->header = msg->header;
  ros_cloud->sensor_origin_ = msg->sensor_origin_;
  ros_cloud->sensor_orientation_ = msg->sensor_orientation_;
  ros_cloud->is_dense = msg->is_dense; 
  new_point_cloud = true; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFromRos::getcloud()
{
  return ros_cloud;
}


void CopieAndAddRGB (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out->header   = cloud_in->header;
  cloud_out->width    = cloud_in->width;
  cloud_out->height   = cloud_in->height;
  cloud_out->is_dense = cloud_in->is_dense;
  cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;
  cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
  cloud_out->points.resize (cloud_in->points.size ());

  if (cloud_in->points.empty())
    return;

  // Iterate over each point
  for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
    pcl::copyPoint (cloud_in->points[i], cloud_out->points[i]);
}


float ConvRGB(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  /* pack r/g/b into rgb -> special case for pcl structure */
  // std::uint8_t r = 255, g = 0, b = 0;    // Example: Red color
  // std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
  // ros_cloud_rgb->points[idx].rgb = *reinterpret_cast<float*>(&rgb);

  std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}


int nbr_plane = 3;

int main(int argc, char** argv)
{
  // ROS setup
  PointCloudFromRos velodyne_pc;

  ros::init(argc, argv, "point_cloud_plane_detection_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("velodyne_points", 1, &PointCloudFromRos::callback, &velodyne_pc);
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("plane_points", 1);
  // ros::spin();


  // Plane setup
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;  // Create the segmentation object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients (true);       // Optional
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setMaxIterations (1000);
  seg.setAxis({{1},{0},{0}});
  seg.setEpsAngle(0.175); // approx 10deg

  
  // // Visualise results setup -> publish instead
  // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZ> (velodyne_pc.getcloud(), "sample cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  // viewer->addCoordinateSystem (1.0);
  // viewer->initCameraParameters ();

  // Cloud points initialisation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ros_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ros_cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ros_cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);

  ros::Rate loop_rate(10);
  // display loop
  while (ros::ok())
  {
    // ROS callback
    ros::spinOnce();
    if(new_point_cloud == true)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ros_cloud_now = velodyne_pc.getcloud();
    

      // All points in White
      CopieAndAddRGB(ros_cloud_now, ros_cloud_rgb);
      for (int i=0; i<ros_cloud_now->points.size(); i++)
        ros_cloud_rgb->points[i].rgb = ConvRGB(255, 255, 255);

      // Prints for debug
      // std::cerr << "Model points before copie: " << ros_cloud_now->points.size() << std::endl;
      // std::cerr << "Model points rgb: " << ros_cloud_rgb->points.size() << std::endl;
        
      if(ros_cloud_now->points.size() != 0)   // To filter out the first few times with no data
      {
        // Compute planes
        for(int i=0; i<nbr_plane; i++)
        {
          seg.setInputCloud (ros_cloud_now);
          seg.segment (*inliers, *coefficients);

          if (inliers->indices.size () == 0)
          {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
          }

          // Extract the inliers
          extract.setInputCloud (ros_cloud_now);
          extract.setIndices (inliers);
          extract.setNegative (false);
          extract.filter (*ros_cloud_plane);    // TODO: save in an array for later use
          // std::cerr << "PointCloud representing the planar component: " << ros_cloud_plan1->width * ros_cloud_plan1->height << " data points." << std::endl;

          // All inliers of planes in Color
          // std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
          for (const auto& idx: inliers->indices)
          {
            int n = int(255/nbr_plane);
            ros_cloud_rgb->points[idx].rgb = ConvRGB(int(255-n*i), int(n*i), 255);
          }

          // Create the filtering object -­> remaning points
          extract.setNegative (true);
          extract.filter (*ros_cloud_temp);
          ros_cloud_now.swap (ros_cloud_temp);

        }
      }
      pub.publish (ros_cloud_rgb);
      new_point_cloud = false;
    }

    // // Update display -> publish instead
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handle(ros_cloud_rgb);
    // viewer->updatePointCloud<pcl::PointXYZRGB> (ros_cloud_rgb, rgb_handle, "sample cloud");
    // viewer->spinOnce (200, true);
  }

  return (0);
}