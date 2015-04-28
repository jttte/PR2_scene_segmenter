#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>

#include <boost/shared_ptr.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>

 #include <pcl/ModelCoefficients.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/features/normal_3d.h>
 #include <pcl/filters/extract_indices.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include "cluster_extractor.h"
#include "scene_segmenter/XylophonePose.h"


pcl::PCDWriter writer;


namespace scene_segmenter_node
{
    class SceneSegmenterNode
    {
        private:
            ros::NodeHandle node_handle;
            ros::ServiceServer service;
            ros::Subscriber pointCloudSubscriber;
            void pointCloudMessageCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
            bool service_callback( scene_segmenter::XylophonePose::Request &req,
                      scene_segmenter::XylophonePose::Response &res );
            geometry_msgs::Pose current_pose;

            //ros::Publisher segmentedObjectsPublisher;

        public:
            SceneSegmenterNode();
    };

    SceneSegmenterNode::SceneSegmenterNode(): node_handle("")
    {
        pointCloudSubscriber = node_handle.subscribe("/head_mount_kinect/depth/points", 10, &SceneSegmenterNode::pointCloudMessageCallback, this);
        std::cout<<"update cloud";

        //segmentedObjectsPublisher = node_handle.advertise<geometry_msgs::Pose>("segmented_objects",10);
        //service
        service = node_handle.advertiseService("get_xylophone_pose", &SceneSegmenterNode::service_callback, this);

        ros::spin();
        ROS_INFO("scene_segmenter_node ready");
    }


    bool SceneSegmenterNode::service_callback(  scene_segmenter::XylophonePose::Request &req,
                      scene_segmenter::XylophonePose::Response &res ){
        std::cout<<"receive call";


        res.pose = current_pose;
        //pose = process_cloud();

        return true;

    }



    void SceneSegmenterNode::pointCloudMessageCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {

        //convert cloud to pcl cloud2
        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *cloud);
        std::cout<<msg->header<<std::endl;

        //transform cloud to world coordinate
        //tf::TransformListener listener;
        //tf::StampedTransform transform;
        //listener.lookupTransform("/head_mount_kinect_rgb_link", "/odom_combined", ros::Time(0), transform);
        //bool success = pcl_ros::transformPointCloud(*cloud,*cloud,transform);

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
std::stringstream ss;
                ss<< "cloud_cluster_"<< i<<".pcd";
                writer.write<pcl::PointXYZ> (ss.str(), *cloudCluster, false);
            
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
                //segmentedObjectsPublisher.publish(pose);
                std::cout<<"x: "<<pose.position.x<<" y: "<<pose.position.y<<" z: "<<pose.position.z<<std::endl;
            //std::stringstream ss;
            //ss<< "cloud_cluster_"<< idx<<".pcd";
            //writer.write<pcl::PointXYZ> (ss.str(), *cloudCluster, false);
            current_pose = pose; 
        }

    }//end of pointCloudMessageCallback
}



int main(int argc, char **argv) 
{

  ros::init(argc, argv, "scene_segmenter_node");
  scene_segmenter_node::SceneSegmenterNode node;

  return 0;
}
