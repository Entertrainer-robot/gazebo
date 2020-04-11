#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/PoseArray.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sstream>
#include <vector>

class collision_checker
{
  public:
  // NodeHandle
  ros::NodeHandle n;

  // publisher for the collision checking result
  ros::Publisher collision_pub = n.advertise<std_msgs::Bool>("collision_check", 100);

  // publisher for the collision occurance time
  ros::Publisher collision_time_pub = n.advertise<std_msgs::Int32>("collision_time", 100);

  // subscriber for point cloud input
  ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 1, &collision_checker::pointCloudCallback, this);

  // subscriber for trajectory input
  ros::Subscriber trajectory_sub = n.subscribe("/Traj_arc_path", 1, &collision_checker::trajectoryCallback, this);

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;

  // callback function for receiving the point cloud
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
  {
    // the current input point cloud
    point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // conversion to PCL point cloud
    pcl::PCLPointCloud2 input_cloud_pc2;
    pcl_conversions::toPCL(*input_cloud,input_cloud_pc2);
    pcl::fromPCLPointCloud2(input_cloud_pc2,*point_cloud);

  }


  // callback function for receiving the trajectory to check collision
  void trajectoryCallback(const geometry_msgs::PoseArrayConstPtr& trajectory)
  {
    if(point_cloud->size()>1)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // clip the point cloud in the desired X-Z plane
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (point_cloud);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (-0.1, 0.1);
      pass.filter (*plane_cloud);

      // create KD Tree
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud (plane_cloud);

      // set search parameters
      float radius = 0.1;
      std_msgs::Bool result;
      std_msgs::Int32 collision_time;
      result.data = false;

      // search through th trajectory, checking for collisions
      for(int i = 0; i < trajectory->poses.size(); i++)
      {
        // create search point
        pcl::PointXYZ searchPoint;
        searchPoint.x = trajectory->poses[i].position.y;
        searchPoint.y = 0;
        searchPoint.z = trajectory->poses[i].position.z;

        // setup search function
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // search
        if(kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  	    {
	        // publish collision = true
	        result.data = true;
                collision_time.data = i;
		collision_time_pub.publish(collision_time);
    	        collision_pub.publish(result);
	        break;
  	    }
      }

    // publish collision = false
    if(result.data == false)
      {
	      collision_time.data = -1;
 	      collision_time_pub.publish(collision_time);
	      collision_pub.publish(result);
      }
    }
  }

};



int main(int argc, char **argv)
{
  // init
  ros::init(argc, argv, "collision_check");

  collision_checker cc;

  ros::spin();

  return 0;
}
