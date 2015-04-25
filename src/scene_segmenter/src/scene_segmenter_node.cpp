#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>

#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
//#include "perception_msgs/SegmentedObject.h"
//#include "perception_msgs/SegmentedObjectList.h"
//#include "perception_msgs/ObjectCenterProperty.h"

#include "cluster_extractor.h"

pcl::PCDWriter writer;

namespace scene_segmenter_node
{
    class SceneSegmenterNode
    {
        private:
            ros::NodeHandle node_handle;

            ros::Subscriber pointCloudSubscriber;
            void pointCloudMessageCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

            ros::Publisher segmentedObjectsPublisher;

        public:
            SceneSegmenterNode();
    };


    SceneSegmenterNode::SceneSegmenterNode(): node_handle("")
    {
        pointCloudSubscriber = node_handle.subscribe("/head_mount_kinect/depth/points", 10, &SceneSegmenterNode::pointCloudMessageCallback, this);
        std::cout<<"update cloud";

        segmentedObjectsPublisher = node_handle.advertise<geometry_msgs::Pose>("segmented_objects",10);

        ROS_INFO("scene_segmenter_node ready");
    }


    void SceneSegmenterNode::pointCloudMessageCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        //convert cloud to pcl cloud2
        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *cloud);

        //extract clusters
        ClusterExtractor *clusterExtractor = new ClusterExtractor();
        clusterExtractor->setCloud(cloud);
        clusterExtractor->computeClusters();
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = clusterExtractor->getCloudClusters();
        delete clusterExtractor;

        //publish clusters
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        
        int max_size = 0;
        int idx = 0;
        bool find_object = false;
        for(int i = 0; i < cloudClusters.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (cloudClusters.at(i));
            if (cloudCluster->size() > max_size)
            {
                max_size = cloudCluster->size();
                idx = i;
                find_object = true;
            }
            
        }//end of cluster list loop
        
        if(find_object == true) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (cloudClusters.at(idx));
            

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
                pcl::copyPointCloud(*cloudCluster,*cloud_xyz);

                Eigen::Vector4f centroid (0.f, 0.f, 0.f, 1.f);
                pcl::compute3DCentroid (*cloudCluster, centroid); 
                centroid.w () = 1.f;
                pose.position.x = centroid.x();
                pose.position.y = centroid.y();
                pose.position.z = centroid.z();
                segmentedObjectsPublisher.publish(pose);
                std::cout<<"x: "<<pose.position.x<<" y: "<<pose.position.y<<" z: "<<pose.position.z<<std::endl;
            //std::stringstream ss;
            //ss<< "cloud_cluster_"<< idx<<".pcd";
            //writer.write<pcl::PointXYZ> (ss.str(), *cloudCluster, false);

        }
    }
}



int main(int argc, char **argv) 
{

  ros::init(argc, argv, "scene_segmenter_node");
  ros::NodeHandle nh;

  scene_segmenter_node::SceneSegmenterNode node;

  ros::spin();
  return 0;
}
