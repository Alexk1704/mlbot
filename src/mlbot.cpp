// ROS
#include "ros/ros.h" 
#include "std_msgs/String.h"
#include "kobuki_msgs/BumperEvent.h"
#include "kobuki_msgs/MotorPower.h"
#include "geometry_msgs/Twist.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

// C Libraries
#include <cmath>
#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>


using namespace std;

// shared pointer for incoming point clouds we will segment later on
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

//global, to see if new_data arrived
bool new_data = false; 
// This is a signal that the robot is in search mode (looking for object to collide with)
bool search_obj = false;
// This is a signal that we have found an object and want to collide with it
bool found_obj = false;
// for bumper events, tells us if we collided with an object to start detecting if we can shove smth
bool received_bump = false;
kobuki_msgs::BumperEvent bump_event;

/* CALLBACKS */

void bump_cb(const kobuki_msgs::BumperEvent &e) {
    bump_event = e; received_bump = true;
}

/* callback gets called if new data available so we can process it,
 * we copy to global var (cloud_ptr) and set new_data to true.
 */
void points_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::fromROSMsg (*input, *cloud_ptr);
	new_data = true ;	
}

/* SEGMENTATION FUNCTIONS */

pcl::PointIndices::Ptr subsampleIndices(pcl::PointIndices::Ptr indices, int fact) {
    pcl::PointIndices::Ptr ret (new pcl::PointIndices) ;
    for (int i=0; i<indices->indices.size(); i+=fact) {
        ret->indices.push_back(indices->indices[i]) ;
    }
    return ret ;
}

pcl::PointIndices::Ptr invertIndices(pcl::PointIndices::Ptr indices, int mx) {
    pcl::PointIndices::Ptr ret (new pcl::PointIndices) ;
    bool * hits = new bool [mx] ;
    for (int i=0; i<mx; i++) hits[i] = false;
    for (int i=0; i<indices->indices.size(); i++) {
        hits[indices->indices[i]] = true;
    }
    for (int i=0; i<mx; i++) {
        if (hits[i]==false) ret->indices.push_back(i) ;
    }
    return ret ;
}

pcl::PointIndices::Ptr intersectIndices(pcl::PointIndices::Ptr indices1, pcl::PointIndices::Ptr indices2) {
    pcl::PointIndices::Ptr ret (new pcl::PointIndices) ;
    int mx = indices1->indices.size() ;
    int tmp = indices2->indices.size() ;
    if (tmp>mx) mx=tmp ;
    mx = 640*480 ;
    cout << indices1->indices.size() << " " << indices2->indices.size() << " " << mx << endl ;
  
    bool * hits1 = new bool [mx] ;
    bool * hits2 = new bool [mx] ;
    for (int i=0; i<mx; i++) { hits1[i] = false; hits2[i] = false; }
    for (int i=0; i<indices1->indices.size(); i++) {
        hits1[indices1->indices[i]] = true;
    }
    for (int i=0; i<indices2->indices.size(); i++) {
        hits2[indices2->indices[i]] = true;
    }
    for (int i=0; i<mx; i++) {
        if (hits1[i] && hits2[i]) ret->indices.push_back(i) ;
    }
    delete hits1;
    delete hits2;
  
    return ret ;
}

void IndicesDeleter(pcl::PointIndices * p) { } // TO-DO: MISSING???


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "controller");
    // Main access point for communication with ROS
    ros::NodeHandle n_handle; 

    /* PUBLISHING TOPICS */
    //ros::Publisher cloud_pub_small = n_handle.advertise<sensor_msgs::PointCloud2>("/subsampled", 1000);
	ros::Publisher cloud_pub_plane = n_handle.advertise<sensor_msgs::PointCloud2>("/plane", 1000);
    ros::Publisher control_motor = n_handle.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1000);
    ros::Publisher control_robot = n_handle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    /* SUBSCRIBED TOPICS */
    ros::Subscriber sub_bump = n_handle.subscribe("/mobile_base/events/bumper", 1000, bump_cb);
    ros::Subscriber sub_points = n_handle.subscribe("/camera/depth/points", 1000, points_cb) ;

    double delta = 10; // run with 10Hz
    ros::Rate loop_rate(delta); // Loops with delta

    double time = -100.0;
    
    geometry_msgs::Twist tw;
    float forward_spd = 0.2; 
    float rotational_spd = 0.0; 
	pcl::PointXYZRGB center_p;
	int gp_pts; int nongp_pts;

    /*
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message
	 */	 
    int count = 0;
    std::srand(0); // random seed gen

    // VoxelGrid needs point types of the cloud we are going to manipulate later
	//pcl::VoxelGrid <pcl::PointXYZRGB> down_sampler; // downsampled cloud
	// allocate two point clouds, first for downsampled cloud, second for found plane points
	// if we put our own pcd into those objects, they will be published automatically
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_small (new pcl::PointCloud<pcl::PointXYZRGB>);	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	// usage of pointclouds: cloud_small->size(); 
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// init state search object
	bool search_obj = true;

    while(ros::ok()) {
        if(count==0) { // Start motor
            kobuki_msgs::MotorPower m_pow;
            m_pow.state = m_pow.ON;
            control_motor.publish(m_pow); 
        }

		/* Routine after first colission/bump with an object */
		if(received_bump == true) {
            std::cout << "COLLISION: DETECTED A BUMP" << std::endl;
			// STOP ROBOT
			geometry_msgs::Twist move_cmd;
			move_cmd.linear.x = move_cmd.linear.y = move_cmd.linear.z = 0;
			move_cmd.angular.x = move_cmd.angular.y = move_cmd.angular.z = 0;
			control_robot.publish(move_cmd);
		    // Add some stuff here

            received_bump = false;
        } 

		/* Routine if robot is looking for a pointcloud to target */
		if(search_obj == true){		
			cout << "Searching for object" << endl;
      		geometry_msgs::Twist move_cmd;
      		move_cmd.linear.x = move_cmd.linear.y = move_cmd.linear.z = 0;
      		move_cmd.angular.x = move_cmd.angular.y = move_cmd.angular.z = 0;

     		move_cmd.angular.z = 2. * 3.14 / 10;   // one rotation every 10 seconds
			//float rot_angle = std::rand()/float(RAND_MAX)*2.*3.14159265-3.14159265;

			control_robot.publish(move_cmd);	
    	} 

		/* Routine if an object was found we want to navigate (slowlyyy.....) to */
        if(found_obj == true && received_bump == false){
			geometry_msgs::Twist move_cmd;
			move_cmd.linear.x = move_cmd.linear.y = move_cmd.linear.z = 0;
			move_cmd.angular.x = move_cmd.angular.y = move_cmd.angular.z = 0;
			if(nongp_pts/gp_pts > 0.25){ // we still have much ground, so probably not near / in front of object
				if(center_p.x > 0.2){		
					move_cmd.angular.z = -0.25;
					move_cmd.linear.x = 0.1;
				} else if(center_p.x < -0.2){
					move_cmd.angular.z = 0.25;
					move_cmd.linear.x = 0.1;
				} else {
					move_cmd.linear.x = 0.25; // pedal to the metal
				}
			} else { // probably in front of object, gogogo
				move_cmd.linear.x = 0.25;
			}
			control_robot.publish(move_cmd);
        }

		/* PCL part here */
        if(new_data){ // wait for new data from pointcloud callback!
            /* START FILTERING HERE */
			cout << "_____________________________________________________________________" << endl;
			std::cerr << "Original PC [Size: "<< cloud_ptr->size() <<  " Height: " << cloud_ptr->height 
				<< " Width: " << cloud_ptr->height << "]" << std::endl;
			
			/* DOWNSAMPLING */
			/*
			down_sampler.setInputCloud(cloud_ptr);
			down_sampler.setLeafSize(0.04, 0.04, 0.04);
			down_sampler.filter(*cloud_small); // result in cloud small
			std::cerr << "PC after filtering: " << cloud_small->width*cloud_small->height
				<< " data points (" << pcl::getFieldsList (*cloud_small) << ")" << std::endl;
			std:cerr << "Point cloud data: " << cloud_small->points.size() << "pts" << std::endl;
			*/

			/* NAN FILTERING */
			vector<int> indices(0);
    		pcl::removeNaNFromPointCloud(*cloud_ptr, indices) ;
    		pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices());
    		indices_ptr->indices = indices;
  
    		cout << "Cloud has " << indices.size() << " points after NAN elimination" << endl;
    		//cout << "ORGANIZED: " << cloud_ptr->isOrganized() << endl ;

    		pcl::PointIndices::Ptr outliers_not_NAN = indices_ptr ;
    		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

			/* EUCLIDEAN CLUSTER EXTRACTION / RANSAC */

			/* Setup model and method type for segmentation.
			 * distance threshold, determines how close a point must be 
			 * to the model in order to be considered an inlier.
			 * We use RANSAC as robust estimator of choice here.
			 * Goal: Find ground plane and remove it!
			 */
			while(true) {
				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				seg.setOptimizeCoefficients(true); // Optional
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);

				/* MAYBE TWEAK NUMBERS HERE */
				seg.setDistanceThreshold(0.05);
				seg.setMaxIterations(100);

				seg.setInputCloud(cloud_ptr);
				seg.setIndices(outliers_not_NAN); // indices that represent the input cloud (without NANs)
				seg.segment(*inliers, *coefficients);

				cout << "Ground plane points: " << inliers->indices.size() << endl;
				gp_pts = inliers->indices.size();

				pcl::PointIndices::Ptr outliers = invertIndices(inliers, 640*480); // Get outliers by swapping indices
				cout << "Non ground plane points: " << outliers->indices.size() << endl;
				nongp_pts = outliers->indices.size();

				outliers_not_NAN = intersectIndices(outliers, outliers_not_NAN);

				// Inliers: all indices on ground plane that are not NAN
				if(((fabs(coefficients->values[1]) > 0.9)) && (inliers->indices.size()>30000)) {
					outliers_not_NAN = intersectIndices(indices_ptr, outliers);
					break;
				}
			}

			// outliers_not_NAN corresponds to all points except ground plane points,
			// all other planesare still there, onle one plane removed so far
			pcl::PointIndices::Ptr outliers_not_NAN_subsample = subsampleIndices(outliers_not_NAN, 4);
			// remove ground plane and store in cloud_noplane
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_noplane (new pcl::PointCloud<pcl::PointXYZRGB>);
   			
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    		extract.setInputCloud(cloud_ptr);
    		extract.setIndices(outliers_not_NAN_subsample);
    		extract.setKeepOrganized(false);
    		extract.setNegative(false);
    		extract.filter(*cloud_noplane);
    		cout << "SUBSAMPLED OBJECT CLOUD (CLOUD_NOPLANE), WITH SIZE: " << cloud_noplane->points.size() << endl;

			/* Extract planar inliers from input cloud */
			/*
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(cloud_small);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*cloud_filtered);
			*/
		
			/* CLUSTERING
			 * Creating KDTree object for search method of extraction
			 * performs clustering on remaining points, set upper/lower limits to disregard noise clusters.
			 */
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			
			/*
			if(cloud_filtered->size() != 0) 
				kd_tree->setInputCloud(cloud_filtered);
			*/
			if(cloud_noplane->size() != 0) 
				kd_tree->setInputCloud(cloud_noplane);

			/* Vector contains actual index info, indices of each detected cluster are saved here, 
			* contains one instance of PointIndices for each detected cluster
			* cluster_indices[0] for first cluster in point cloud, and so on...
			*/
			std::vector<pcl::PointIndices> cluster_indices;

			/* Create ECE with point type XYZRGB, also setting params for extraction
			* setClusterTolerance(), if small value, it can happen that actual object can be seen
			* as multiple clusters, on the other hand if too high, that multiple objects are seen as one cluster.
			* We impose: clusters found have at least setMinClusterSize() points and maximum setMaxClusterSize() points.
			* Then extract cluster out of point cloud and save indices in cluster_indices
			*/		
			pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
			ec.setClusterTolerance(0.1); // 10cm
			ec.setMinClusterSize(100);
			ec.setMaxClusterSize(1000000);
			ec.setSearchMethod(kd_tree);
			//ec.setInputCloud(cloud_filtered);
			ec.setInputCloud(cloud_noplane);
			ec.extract (cluster_indices);
			
			if(cluster_indices.size() > 0)
				cout << "FOUND CLUSTERS: " << cluster_indices.size() << endl;

			/* CLUSTER PROCESSING
			 * To separate each cluster out of vector<PointIndices> we have to iterate through cluster_indices,
			 * create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
			 */

			/* Iterate over found clusters and choose the closest one */
    		double smallest_dist = 1000000000;
    		int closest_cluster = -1;
    		pcl::PointXYZRGB centr;
    		pcl::PointCloud<pcl::PointXYZRGB>::Ptr closest_obj;

			/* Get closest cluster index from cluster_indices */
			/*
    		pcl::PointIndices::Ptr ptr;
   			for (int i=0; i<cluster_indices.size(); i++) {
     			cout << "cluster "<< i << " has " << cluster_indices[i].indices.size() << " elements " << endl;
      		  	ptr = pcl::PointIndices::Ptr (&(cluster_indices[i]), IndicesDeleter) ;
       			 // extract cluster points to temporary cloud 'obj'
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj (new pcl::PointCloud<pcl::PointXYZRGB> );
				extract.setInputCloud(cloud_noplane);
				extract.setIndices(ptr);
				extract.setNegative(false);
				extract.filter(*obj);
				pcl::computeCentroid (*obj, centr);
				double dist_to_center = sqrt(centr.x*centr.x + centr.y*centr.y*0 + centr.z * centr.z);
				cout << "cluster  " << i << " has dist " << dist_to_center << endl;

				if (dist_to_center < smallest_dist) {
					closest_cluster = i;
					smallest_dist = dist_to_center;
					closest_obj = obj;
				}
    		}
    		cout << "closest cluster has index " << closest_cluster << endl;
			*/

			/* Extraction of centroid & avg. RGB values of found cluster */
			if(cluster_indices.size() > 0) {
				for(int cluster_id = 0 ; cluster_id < cluster_indices.size(); cluster_id++){
					pcl::PointIndices &id = cluster_indices[cluster_id];
					float avgR = 0 ; float avgG = 0; float avgB = 0;
					float avgX = 0 ; float avgY = 0; float avgZ = 0;

					// For calculating center of point cloud
					pcl::CentroidPoint<pcl::PointXYZRGB> centroid;

					for (int point_id = 0; point_id < id.indices.size(); point_id++) {
						avgR += (*cloud_noplane)[point_id].r ;
						avgG += (*cloud_noplane)[point_id].g ;
						avgB += (*cloud_noplane)[point_id].b ;
						avgX += (*cloud_noplane)[point_id].x ;
						avgY += (*cloud_noplane)[point_id].y ;
						avgZ += (*cloud_noplane)[point_id].z ;

						centroid.add((*cloud_noplane)[point_id]);
					}

					// Fetch centroid using `get()`
					centroid.get (center_p);
					cout << "CENTER POINT COORDS [x: " << center_p.x << " y: " 
						<< center_p.y << " z: " << center_p.z << "]" << endl;
			
					avgR /= float(id.indices.size()) ;
					avgG /= float(id.indices.size()) ;
					avgB /= float(id.indices.size()) ;
					avgX /= float(id.indices.size()) ;
					avgY /= float(id.indices.size()) ;
					avgZ /= float(id.indices.size()) ;
					cout << "CLUSTER ID: " << cluster_id << " Avg. RGB: " << (avgR+avgG+avgB)/3. << endl ;
					search_obj = false; 
					found_obj = true;
					/*
					if((((avgR+avgG+avgB)/3.) > 130) && (fabs(avgX) < 0.1)) {
						cout << "White!!!" << endl ;
						looking_for_white = false;
					}
					*/
				}
			}

			/*
			// conversion of downsampling result to ROS sensor_msg
			pcl::PCLPointCloud2 conv1;
			sensor_msgs::PointCloud2 toROS;	
			pcl::toPCLPointCloud2(*cloud_noplane, conv1);
			pcl_conversions::fromPCL(conv1, toROS);	
			// publish downsampling result
			cloud_pub_small.publish(toROS);
			*/
		
			pcl::PCLPointCloud2 conv1;
			sensor_msgs::PointCloud2 toROS;
			// conversion of plane_points to PointCloud2
			pcl::toPCLPointCloud2(*cloud_noplane, conv1) ;
			// conversion to ROS sensor_msg
			pcl_conversions::fromPCL(conv1, toROS);		
			// publish inliers result
			cloud_pub_plane.publish(toROS);
			new_data = false;
        }

        ros::spinOnce(); // process single round of callbacks
        loop_rate.sleep();
        ++count;

    }
	
    return 0;
}