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

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// C Libraries
#include <cmath>
#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>


using namespace std;

// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointIndices::Ptr IndicesPtr;
typedef pcl::PointIndices Indices;
typedef std::vector<pcl::PointIndices> IndicesVector;
typedef pcl::ExtractIndices<pcl::PointXYZRGB> ExtractIndices;
typedef pcl::CentroidPoint<pcl::PointXYZRGB> Centroid;

#define PI 3.14159265359

// shared pointer for incoming point clouds we will segment later on
PointCloudPtr cloudPtr(new PointCloud);
PointCloudPtr cloudPtrSmall(new PointCloud); // for downsampling a point cloud
Point centerPoint; // cluster centroid

// global, to see if new data to process arrived
bool newData = false; 
// This is a signal that the robot is in search mode (looking for a object in a circular motion)
bool searchObject = false;
// This is a signal that we have found an object we want to collide with it
bool foundObject = false;
// This is a signal that we are returning to center location after a collision attempt
bool isReturning = false;
// for incoming bumper events, tells us if we collided with an object to start detecting if we can shove smth
bool receivedBump = false;
// after incoming bumper events, starts the shoveling part and observing if we in fact can move the object
bool isPushing = false;

kobuki_msgs::BumperEvent bumperEvent;

/* CALLBACKS */
void bumpCallback(const kobuki_msgs::BumperEvent &e){
    bumperEvent = e; receivedBump = true;
}

/* callback gets called if new data is available so we can process it,
 * we copy the point cloud to a global var (cloudPtr) and set newData to true
 */
void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg (*input, *cloudPtr); newData = true;	
}

ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
   std_msgs::Header msgHeader = msg->header;
   std::string frameId = msgHeader.frame_id.c_str();
   ROS_INFO_STREAM("New Image from " << frameId);

   cv_bridge::CvImagePtr cvPtr;
   try
   {
     cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
 }

/* SEGMENTATION FUNCTIONS */

IndicesPtr subsampleIndices(IndicesPtr indices, int fact){
    IndicesPtr ret (new Indices);
	// sub sample indices from a given integer value i+=fact same as i=i+fact
    for(int i=0; i<indices->indices.size(); i+=fact){
        ret->indices.push_back(indices->indices[i]);
    }
    return ret;
}

IndicesPtr invertIndices(IndicesPtr indices, int mx){
    IndicesPtr ret (new Indices);
    bool* hits = new bool[mx];
	// init bool array with false values
    for(int i=0; i<mx; i++) hits[i] = false;
	// go through all indices and fill bool array with true for indices found
    for(int i=0; i<indices->indices.size(); i++){
        hits[indices->indices[i]] = true;
    }
	// each index set to false is pushed back into a new PointIndices 
    for(int i=0; i<mx; i++){
        if (hits[i]==false) ret->indices.push_back(i);
    }
    return ret;
}

IndicesPtr intersectIndices(IndicesPtr indices1, IndicesPtr indices2){
    IndicesPtr ret (new Indices);
    int mx = indices1->indices.size();
    int tmp = indices2->indices.size();
    if (tmp>mx) mx=tmp;
    mx = 640*480; // dimension of points
    //cout << indices1->indices.size() << " " << indices2->indices.size() << " " << mx << endl;
    bool* hits1 = new bool[mx];
    bool* hits2 = new bool[mx];
    for(int i=0; i<mx; i++){ hits1[i] = false; hits2[i] = false; } // init bool arrays
    // set indices from both pointindices in corresponding bool arrays
	for(int i=0; i<indices1->indices.size(); i++){
        hits1[indices1->indices[i]] = true;
    }
    for (int i=0; i<indices2->indices.size(); i++){
        hits2[indices2->indices[i]] = true;
    }
	// if indices intersect, push back into returning object
    for(int i=0; i<mx; i++){
        if(hits1[i] && hits2[i]) ret->indices.push_back(i);
    }
    delete hits1;
    delete hits2;
  
    return ret;
}

/* get closest cluster index from given cluster indices and an input point cloud, we iterate over found clusters and choose closest returning the index */
int getClosestCluster(IndicesVector clusterIndices, PointCloudPtr cloud){
	int closestClusterId = -1;
	double distance = 999999;
	double centerP;
	Point cp;
	float avgR = 0 ; float avgG = 0; float avgB = 0;
	float avgX = 0 ; float avgY = 0; float avgZ = 0;
	Centroid centroid;

	if(cloud->size() != 0){
		for (int i=0; i<clusterIndices.size(); i++) {
			cout << "[CLUSTER]\tCluster "<< i << " has " << clusterIndices[i].indices.size() << " elements" << endl;
			for (int pointId = 0; pointId < clusterIndices[i].indices.size(); pointId++) {
				avgR += (*cloud)[pointId].r;
				avgG += (*cloud)[pointId].g;
				avgB += (*cloud)[pointId].b;
				avgX += (*cloud)[pointId].x;
				avgY += (*cloud)[pointId].y;
				avgZ += (*cloud)[pointId].z;
				centroid.add((*cloud)[pointId]);
			}
			centroid.get(cp); // Fetch centroid using `get()`
			if(cp.x < 0) centerP = cp.x * -1;
			else centerP = cp.x;
			if(centerP < distance){
				distance = cp.x;
				closestClusterId = i;
			}
		}	
	}
	cout << "[CLUSTER]\tClosest cluster has index " << closestClusterId << " with x distance: " << distance << endl;
	return closestClusterId;
}

/* Extraction of centroid & avg. RGB values of found cluster */
void postProcessClusters(PointCloudPtr cloud, Indices closestIndices){

	float avgR = 0 ; float avgG = 0; float avgB = 0;
	float avgX = 0 ; float avgY = 0; float avgZ = 0;
	// For calculating the center of a point cloud
	Centroid centroid;
	if(cloud->size() != 0){
		for (int pointId = 0; pointId < closestIndices.indices.size(); pointId++) {
			avgR += (*cloud)[pointId].r;
			avgG += (*cloud)[pointId].g;
			avgB += (*cloud)[pointId].b;
			avgX += (*cloud)[pointId].x;
			avgY += (*cloud)[pointId].y;
			avgZ += (*cloud)[pointId].z;
			centroid.add((*cloud)[pointId]);
		}
		// Fetch centroid using `get()`
		centroid.get(centerPoint);
		cout << "[CLUSTER]\tCenter point coordinate [X: " << centerPoint.x << " Y: " << centerPoint.y << " Z: " << centerPoint.z << "]" << endl;
		// calculate avg. RGBXYZ values
		avgR /= float(closestIndices.indices.size());
		avgG /= float(closestIndices.indices.size());
		avgB /= float(closestIndices.indices.size());
		avgX /= float(closestIndices.indices.size());
		avgY /= float(closestIndices.indices.size());
		avgZ /= float(closestIndices.indices.size());
		cout << "[CLUSTER]\tAverage RGB: " << (avgR+avgG+avgB)/3. << endl ;
	}
	/* Check if avg. RGB value over threshold to detect a color of clustered object
	if((((avgR+avgG+avgB)/3.) > 130) && (fabs(avgX) < 0.1)) { ... }
	*/
}

/* returns a geometry_msgs::Twist move command we can send to our bot */
geometry_msgs::Twist moveBot(float linX, float linY, float linZ, float angX, float angY, float angZ){
	geometry_msgs::Twist moveCommand;
	moveCommand.linear.x = linX; moveCommand.linear.y = linY; moveCommand.linear.z = linZ;
	moveCommand.angular.x = angX; moveCommand.angular.y = angY; moveCommand.angular.z = angZ;
	return moveCommand;
}
			
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "controller");
    ros::NodeHandle nodeHandle; // Main access point for communication with ROS

    /* PUBLISHING TOPICS */
 	static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
 	static const std::string PUBLISH_TOPIC = "/image_converter/output_video";
	
	ros::Publisher cloudPubPlane = nodeHandle.advertise<sensor_msgs::PointCloud2>("/plane", 1000);
	ros::Publisher cloudPubGPlane = nodeHandle.advertise<sensor_msgs::PointCloud2>("/groundplane", 1000);
    ros::Publisher controlMotor = nodeHandle.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1000);
    ros::Publisher controlRobot = nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    /* SUBSCRIBED TOPICS */
    ros::Subscriber bumpSub = nodeHandle.subscribe("/mobile_base/events/bumper", 1000, bumpCallback);
    ros::Subscriber pointsSub = nodeHandle.subscribe("/camera/depth/points", 1000, pointsCallback);

    ros::Rate loopRate(10); // loop with with x Hz (10 Hz -> 10 main loop calls per sec)
    int count = 0; // simple counter that gets incremented every main loop access
    std::srand(std::time(NULL)); // random seed gen

	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	Point centerPoint; // cluster object centroid

	bool visited[8]; // bool array for equally distributing push attempts on each object
	int angles[8]; // array to hold angles for each object
	double currentAngle; // current heading of robot
	bool locatedObject; // has pulled a magic number and is turning to destination point
	double relativeAngle, angularSpeed;
	int magic;

    while(ros::ok()) {
        if(count==0) { // Start motor
            kobuki_msgs::MotorPower mPower;
            mPower.state = mPower.ON;
            controlMotor.publish(mPower); 
			for(int i = 0; i < 8; i++) visited[i] = false;
			for(int i = 0; i < 8; i++) angles[i] = 45 * i;
			currentAngle = 0;
			searchObject = true; locatedObject = false;
        }
		/* Routine after colission/bump with an object */
		if(receivedBump) {
			foundObject = false;
			geometry_msgs::Twist moveCommand = moveBot(0, 0, 0, 0, 0 ,0); // STOP ROBOT	
			controlRobot.publish(moveCommand);
			std::cout << "[BUMPER]\tCOLISSION - DETECTED A BUMP!" << std::endl;
			int pushCount = 0;    
			while(true){ // 3 seconds push
				pushCount++;
				moveCommand.linear.x = 0.1;
				controlRobot.publish(moveCommand);
				
				// COLLISION ROUTINE HERE TO-DO
				
				if(pushCount == 30) {
					break; receivedBump = false; 
				} 
			}
            
        } 
		/* Routine if robot is looking for a pointcloud to target */
		if(searchObject){		
			if(!locatedObject){
				std::cout << "[INFO]\tSEARCHING FOR OBJECT..." << endl;			
				// check and reset visited array if all have been visited
				int vc = 0;
				for(int i = 0; i < 8; i++) if(visited[i] == false) vc++;		
				if (vc == 0) for(int i = 0; i < 8; i++) visited[i] == true;
				// generate random target number
				magic = rand()%7;
				while(visited[magic]) magic = rand()%7;
				double speed = 22.5; // degrees/sec
				double dif = currentAngle / 2. / PI * 360;
				// set angle we want to hit 
				double angle;
				if(dif > angles[magic]) angle = dif - angles[magic]; 
				else angle = angles[magic] - dif; 
				// conversion from angle to radians
				angularSpeed = speed * 2. * PI / 360;
				relativeAngle = angle * 2. * PI / 360;
				std::cout << "[INFO]\tTargeted object: " << magic << " Rel. angle: " << relativeAngle << " Cur. angle: " << currentAngle << " Angular spd: " << angularSpeed << endl;
				locatedObject = true;
			}


			geometry_msgs::Twist moveCommand;
			moveCommand.linear.x = moveCommand.linear.y = moveCommand.linear.z = 0;
			moveCommand.angular.x = moveCommand.angular.y = moveCommand.angular.z = 0;
			//moveCommand.angular.z = abs(angularSpeed);
			moveCommand.angular.z = abs(angularSpeed);   
			controlRobot.publish(moveCommand);
			if(count==160) std::cout << "16 sec, 22,5 deg/sec" << endl;

			// STOP after angle hit
			if(count==160){
				geometry_msgs::Twist moveCommand = moveBot(0,0,0,0,0,0);
				controlRobot.publish(moveCommand);
				visited[magic] = true;
				searchObject = false;	
			}
    	} 

		/* Routine for found object we want to navigate to */
        if(foundObject && !receivedBump) {
			geometry_msgs::Twist moveCommand;
			// TO-DO  cloud still small, so probably not near / in front of object, do turning
			// check cloud size and if certain threshold is reached, just go with a full linear speed
			if(centerPoint.x > 0.2){	
				moveCommand.angular.z = -0.2;
				moveCommand.linear.x = 0.1;
			} else if(centerPoint.x < -0.2){
				moveCommand.angular.z = 0.2;
				moveCommand.linear.x = 0.1;
			} else {
				moveCommand.linear.x = 0.2; // pedal to the metal
			}
			controlRobot.publish(moveCommand);
        }

		/* Routine to return to center point */
		if(isReturning) {
			// TO DO
			// idea here is to go for the cloud in ground plane, same logic as with collision
			// if cloud is under a certain threshold, come to stop and start with searching for object again
		}

		/* Routine for pushing an object after initial bump */
		if(isPushing){
			// TO DO
		}

		/* PCL part here */
        if(newData && foundObject){ // wait for new data from pointcloud callback!
			/* START FILTERING HERE */
			std::cerr << "\n[POINT CLOUD]\tUnprocessed point cloud [size: "<< cloudPtr->size() <<  " height: " << cloudPtr->height 
				<< " width: " << cloudPtr->width << "]" << std::endl;

			/* DOWN SAMPLING 
			 * Create a VoxelGrid filter and perform down sampling on the input cloud 
			 */
			/*
			pcl::VoxelGrid <pcl::PointXYZRGB> downSampler;
			downSampler.setInputCloud(cloudPtr);
			downSampler.setLeafSize(0.04, 0.04, 0.04);
			downSampler.filter(*cloudPtrSmall); // result is saved in cloudSmall
			std::cerr << "[POINT CLOUD]\tPoint cloud after filtering [size: "<< cloudPtrSmall->points.size() << " height: " << cloudPtrSmall->width*cloudPtrSmall->height
						<< " data points (" << pcl::getFieldsList (*cloudPtrSmall) << ")]" << std::endl;
			*/

			/* NAN FILTERING */
			vector<int> indices(0);
    		pcl::removeNaNFromPointCloud(*cloudPtr, indices);

    		IndicesPtr indicesPtr (new pcl::PointIndices());
    		indicesPtr->indices = indices;
    		IndicesPtr outliersNotNaN = indicesPtr;
    		IndicesPtr inliers (new pcl::PointIndices);
			//cout << "[POINT CLOUD]\tPoint cloud has " << indices.size() << " points after NAN elimination" << endl;
			//cout << "[POINT CLOUD]\tORGANIZED: " << cloudPtr->isOrganized() << endl;

			/* Setup model and method type for segmentation.
			* Distance threshold, determines how close a point must be to the model in order to be considered an inlier.
			* We use RANSAC as robust estimator of choice here with the goal of finding the ground plane and removing it.
			* The algorithm assumes that all of the data in a dataset is composed of both inliers and outliers.
			* 		Inliers: can be defined by a particular model with a specific set of parameters.
			*   	Outliers: if that model does not fit then it gets discarded.
			*/
			while(true) {
				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				seg.setOptimizeCoefficients(true); // Optional
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);

				/* MAYBE TWEAK NUMBERS HERE */
				seg.setDistanceThreshold(0.05);
				seg.setMaxIterations(100);

				seg.setInputCloud(cloudPtr);
				seg.setIndices(outliersNotNaN); // indices that represent the input cloud (without NANs)
				seg.segment(*inliers, *coefficients);

				cout << "[CLUSTER]\tGround plane points: " << inliers->indices.size() << endl;
				pcl::PointIndices::Ptr outliers = invertIndices(inliers, 640*480); // Get outliers by swapping indices
				cout << "[CLUSTER]\tNon ground plane points: " << outliers->indices.size() << endl;
				outliersNotNaN = intersectIndices(outliers, outliersNotNaN);

				// Inliers: all indices on ground plane that are not NAN
				if(((fabs(coefficients->values[1]) > 0.9)) && (inliers->indices.size()>30000)) {
					outliersNotNaN = intersectIndices(indicesPtr, outliers);
					break;
				}
			}

			// outliers_not_NAN corresponds to all points except ground plane points
			// all other planes are still there, only one plane removed so far
			pcl::PointIndices::Ptr outliersNotNaNSubSample = subsampleIndices(outliersNotNaN, 4);
			// remove ground plane and store in cloud_noplane
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNoPlane (new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    		extract.setInputCloud(cloudPtr);
    		extract.setIndices(outliersNotNaNSubSample);
    		extract.setKeepOrganized(false);
    		extract.setNegative(false);
    		extract.filter(*cloudNoPlane);
    		cout << "[POINT CLOUD]\tSUBSAMPLED OBJECT CLOUD (cloudNoPlane), WITH SIZE: " << cloudNoPlane->points.size() << endl;

			/* CLUSTERING
			 * Creating KDTree object for search method of extraction
			 * performs clustering on remaining points, set upper/lower limits to disregard noise clusters.
			 */
			IndicesVector clusterIndices;
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			if(cloudNoPlane->size() != 0) 
				kdTree->setInputCloud(cloudNoPlane);
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
			ec.setSearchMethod(kdTree);
			ec.setInputCloud(cloudNoPlane);
			ec.extract(clusterIndices);

			if(clusterIndices.size() > 0)
				cout << "[CLUSTER]\tFOUND CLUSTERS: " << clusterIndices.size() << endl;
			
			/* CLUSTER PROCESSING
			 * To separate each cluster out of vector<PointIndices> we have to iterate through cluster_indices,
			 * create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
			 */
			int closestClusterId = getClosestCluster(clusterIndices, cloudNoPlane);
			postProcessClusters(cloudNoPlane, clusterIndices[closestClusterId]);

			/* PCL TO ROS CONVERSION FOR PUBLISHING */
			pcl::PCLPointCloud2 conv1;
			//pcl::PCLPointCloud2 conv2;
			sensor_msgs::PointCloud2 toROS1;
			//sensor_msgs::PointCloud2 toROS2;
			// plane points to PCL2 format
			pcl::toPCLPointCloud2(*cloudNoPlane, conv1);
			//pcl::toPCLPointCloud2(*cloudGroundPlane, conv2);
			// conversion to ROS sensor_msg
			pcl_conversions::fromPCL(conv1, toROS1);
			//pcl_conversions::fromPCL(conv2, toROS2);		
			// publishing results here
			cloudPubPlane.publish(toROS1);
			//cloudPubGPlane.publish(toROS2);
			newData = false; // done processing point clouds
        }
        ros::spinOnce(); // process single round of callbacks
        loopRate.sleep();
        ++count;

    }
    return 0;
}
