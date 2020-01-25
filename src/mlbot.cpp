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

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointIndices::Ptr PointIndicesPtr;
typedef pcl::PointIndices Indices;
typedef std::vector<pcl::PointIndices> PointIndicesVector;
typedef pcl::ExtractIndices<pcl::PointXYZRGB> ExtractIndices;

#define PI 3.14159265359

// shared pointer for incoming point clouds we will segment later on
PointCloudPtr cloudPtr(new PointCloud);
PointCloudPtr cloudPtrSmall(new PointCloud); // for downsampling a point cloud
Point centerPoint; // cluster centroid

// global, to see if new data to process arrived
bool newData = false; 
// This is a signal that the robot is in search mode (looking for a object in a circular motion)
bool isSpinning = false;
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

/* SEGMENTATION FUNCTIONS */

PointIndicesPtr subsampleIndices(PointIndicesPtr indices, int fact){
    PointIndicesPtr ret (new Indices);
	// sub sample indices from a given integer value i+=fact same as i=i+fact
    for(int i=0; i<indices->indices.size(); i+=fact){
        ret->indices.push_back(indices->indices[i]);
    }
    return ret;
}

PointIndicesPtr invertIndices(PointIndicesPtr indices, int mx){
    PointIndicesPtr ret (new Indices);
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

PointIndicesPtr intersectIndices(PointIndicesPtr indices1, PointIndicesPtr indices2){
    PointIndicesPtr ret (new Indices);
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
int getClosestCluster(PointIndicesVector clusterIndices, PointCloudPtr cloudNoPlane){
	double smallestDistance = 1000000000;
	int closestClusterId = -1;
	Point centerPoint;
	PointCloudPtr closestObject;
	PointIndicesPtr ptr;
	ExtractIndices extract;

	for (int i=0; i<clusterIndices.size(); i++) {
		cout << "[CLUSTER]\tCluster "<< i << " has " << clusterIndices[i].indices.size() << " elements" << endl;
		ptr = PointIndicesPtr(&clusterIndices[i]);
		// extract cluster points to temporary cloud 'obj'
		PointCloudPtr obj (new PointCloud);
		extract.setInputCloud(cloudNoPlane);
		extract.setIndices(ptr);
		extract.setNegative(false);
		extract.filter(*obj);
		// compute centroid point of current temporary cloud
		pcl::computeCentroid (*obj, centerPoint);
		double distanceToCenter = sqrt(centerPoint.x * centerPoint.x + centerPoint.y * centerPoint.y * 0 + centerPoint.z * centerPoint.z);
		cout << "[CLUSTER]\tCluster  " << i << " has distance " << distanceToCenter << endl;
		if (distanceToCenter < smallestDistance){
			closestClusterId = i;
			smallestDistance = distanceToCenter;
			closestObject = obj;
		}
		cout << "[CLUSTER]\tClosest cluster has index " << closestClusterId << endl;
	}
	return closestClusterId;
}

// sub sampling method
void extractCluster(PointCloudPtr cloud, PointCloudPtr filter, PointIndicesPtr indices, bool keepOrganized, bool setNegative){
	ExtractIndices extract;
	extract.setInputCloud(cloudPtr);
	extract.setIndices(indices); // e.g. inlierIndices, outliersNanFiltered
	extract.setKeepOrganized(false);
	extract.setNegative(false);
	extract.filter(*filter); // e.g. cloudNoPlane, cloudGroundPlane
	cout << "[CLUSTER]\tSubsampled object cloud [size: " << filter->points.size() << "]" << endl;				
}
/* Euclidian Cluster Extraction
 * cloud is the original cloud obtained from sensors
 * 0.1, 100, 1000000 params
 */
void euclideanClusterExt(PointCloudPtr cloud, PointIndicesVector clusterIndices, float tolerance, int min, int max){
	if(cloudPtr->size() != 0){
		/* Creating KDTree object for search method of extraction
		 * performs clustering on remaining points, set upper/lower limits to disregard noise clusters.
		 */
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		/* Create ECE with point type XYZRGB, also setting params for extraction:
		 * setClusterTolerance(), if small value, it can happen that actual object can be seen as multiple clusters, 
		 * on the other hand if too high, that multiple objects are seen as one cluster.
		 * We impose: clusters found have at least setMinClusterSize() points and maximum setMaxClusterSize() points.
		 * Then extract cluster out of point cloud and save indices in clusterIndices
		 */	
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
		kdTree->setInputCloud(cloudPtr);

		ece.setClusterTolerance(tolerance); // 0.1 corresponds to 10cm
		ece.setMinClusterSize(min); // min size of cluster to extract e.g. 100
		ece.setMaxClusterSize(max); // max size e.g. 1000000 (640x480 -> 307200)
		ece.setSearchMethod(kdTree);
		ece.setInputCloud(cloudPtr);
		ece.extract(clusterIndices);
	}
}

/* Cluster post processing
 * to separate each cluster out of PointIndices vector we have to iterate through clusterIndices,
 * we create a new PointCloud for each entry and write all points of the current cluster into this PointCloud object
 */

/* Extraction of centroid & avg. RGB values of found cluster */
void postProcessClusters(PointCloudPtr cloud, PointIndicesVector clusterIndices){
	if(clusterIndices.size() > 0) {
		for(int clusterId = 0 ; clusterId < clusterIndices.size(); clusterId++){
			Indices &id = clusterIndices[clusterId];
			float avgR = 0 ; float avgG = 0; float avgB = 0;
			float avgX = 0 ; float avgY = 0; float avgZ = 0;
			// For calculating center of point cloud
			pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
			for (int pointId = 0; pointId < id.indices.size(); pointId++) {
				avgR += (*cloud)[pointId].r ;
				avgG += (*cloud)[pointId].g ;
				avgB += (*cloud)[pointId].b ;
				avgX += (*cloud)[pointId].x ;
				avgY += (*cloud)[pointId].y ;
				avgZ += (*cloud)[pointId].z ;
				centroid.add((*cloud)[pointId]);
			}

			// Fetch centroid using `get()`
			centroid.get(centerPoint);
			cout << "[CLUSTER]\tCenter point coordinates [X: " << centerPoint.x << " Y: " 
				<< centerPoint.y << " Z: " << centerPoint.z << "]" << endl;
			avgR /= float(id.indices.size());
			avgG /= float(id.indices.size()) ;
			avgB /= float(id.indices.size());
			avgX /= float(id.indices.size());
			avgY /= float(id.indices.size());
			avgZ /= float(id.indices.size());
			cout << "[CLUSTER]\tID: " << clusterId << " Average RGB: " << (avgR+avgG+avgB)/3. << endl ;
			/* Check if avg. RGB value over threshold to detect a color of clustered object
			if((((avgR+avgG+avgB)/3.) > 130) && (fabs(avgX) < 0.1)) {
				cout << "White!!!" << endl ;
				looking_for_white = false;
			}
			*/
		}
	}
}
/* Create a VoxelGrid filter and perform down sampling on the input cloud */
void downSampler(PointCloudPtr cloud, PointCloudPtr cloudSmall){
	pcl::VoxelGrid <pcl::PointXYZRGB> downSampler;
	downSampler.setInputCloud(cloud);
	downSampler.setLeafSize(0.04, 0.04, 0.04);
	downSampler.filter(*cloudSmall); // result is saved in cloudSmall
	std::cerr << "[POINT CLOUD]\tPoint cloud after filtering [size: "<< cloudSmall->points.size() << " height: " << cloudSmall->width*cloudSmall->height
					<< " data points (" << pcl::getFieldsList (*cloudSmall) << ")]" << std::endl;
}

/* Setup model and method type for segmentation.
 * Distance threshold, determines how close a point must be to the model in order to be considered an inlier.
 * We use RANSAC as robust estimator of choice here with the goal of finding the ground plane and removing it.
 * The algorithm assumes that all of the data in a dataset is composed of both inliers and outliers.
 * 		Inliers: can be defined by a particular model with a specific set of parameters.
 *   	Outliers: if that model does not fit then it gets discarded.
 */
void segmentCloud(PointCloudPtr cloud, PointIndicesPtr input, PointIndicesPtr inliers, PointIndicesPtr indices){
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	while(true) {
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		seg.setOptimizeCoefficients(true); // Optional
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		// maybe tweak numbers here
		seg.setDistanceThreshold(0.05);
		seg.setMaxIterations(100);
		seg.setInputCloud(cloudPtr);
		seg.setIndices(input); // indices that represent the input cloud (e.g. outliers without NANs)
		seg.segment(*inliers, *coefficients);
		cout << "[CLUSTER]\tGround plane points: " << inliers->indices.size() << endl;

		PointIndicesPtr outliers = invertIndices(inliers, 640*480); // Get outliers by swapping indices
		cout << "[CLUSTER]\tNon ground plane points: " << outliers->indices.size() << endl;
		input = intersectIndices(outliers, input);
		// inliers: all indices on ground plane that are not NAN
		if(((fabs(coefficients->values[1]) > 0.9)) && (inliers->indices.size()>30000)) {
			input = intersectIndices(indices, outliers);
			break;
		}
	}
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
	// Twist object for moving the robot
    geometry_msgs::Twist moveCommand;
	Point centerPoint; // cluster object centroid
	bool searchObject = true; // init state search object when program starts
	bool visited[8]; // bool array for equally distributing push attempts on each object

    while(ros::ok()) {

        if(count==0) { // Start motor
            kobuki_msgs::MotorPower mPower;
            mPower.state = mPower.ON;
            controlMotor.publish(mPower); 
			for(int i = 0; i < 8; i++) visited[i] = false;
        }

		/* Routine after colission/bump with an object */
		if(receivedBump) {
            std::cout << "[INFO]\tCOLISSION - DETECTED A BUMP!" << std::endl;
			moveCommand = moveBot(0, 0, 0, 0, 0 ,0); // STOP ROBOT	
			controlRobot.publish(moveCommand);
		    // COLLISION ROUTINE HERE
			// TO-DO
            receivedBump = false;	
        } 

		/* Routine if robot is looking for a pointcloud to target */
		if(isSpinning){	
			// MAYBE if(count%(10*16)==0){ ... } if sleep does not work
			cout << "[INFO]\tSEARCHING FOR OBJECT..." << endl;		
			// TO-DO: 8 objects * 2 seconds = 16 seconds for a full turn, stop after 1 second if we got that magic number pulled
     		// float angularZ = 2. * PI / 10
			int magic = rand()%7;
			visited[magic] = true;
			double speed = 22.5; // degrees/sec
			double angle = 45*magic; // degrees

			// conversion from angle to radians
			double angularSpeed = speed * 2. * PI / 360;
			double relativeAngle = angle * 2. * PI / 360;

			double t0 = ros::Time::now().toSec(); // set current time
			double currentAngle = 0;
			
			while(currentAngle < relativeAngle){
				moveCommand = moveBot(0, 0, 0, 0, 0, abs(angularSpeed));
				controlRobot.publish(moveCommand);
				double t1 = ros::Time::now().toSec();
				currentAngle = angularSpeed*(t1-t0);
			}
			moveCommand.angular.z = 0;
			controlRobot.publish(moveCommand);
			ros::Duration(16).sleep();
			isSpinning = false;	
    	} 

		/* Routine for found object we want to navigate to */
        if(foundObject && !receivedBump) {
			geometry_msgs::Twist moveCommand;
			// TO-DO  cloud still small, so probably not near / in front of object, do turning
			// check cloud size and if certain threshold is reached, just go with a full linear speed
			if(centerPoint.x > 0.2){	
				moveCommand = moveBot(0.1, 0, 0, 0, 0, -0.25);
			} else if(centerPoint.x < -0.2){
				moveCommand = moveBot(0.1, 0, 0, 0, 0, 0.25);
			} else {
				moveCommand = moveBot(0.5, 0, 0, 0, 0, 0); // pedal to the metal
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
				cout << "\n" << endl;
				/* START FILTERING HERE */
				std::cerr << "[POINT CLOUD]\tUnprocessed point cloud [size: "<< cloudPtr->size() <<  " height: " << cloudPtr->height 
					<< " width: " << cloudPtr->width << "dimension: " << cloudPtr->width*cloudPtr->height << "]" << std::endl;
				/* NAN FILTERING */
				vector<int> indices(0);
				pcl::removeNaNFromPointCloud(*cloudPtr, indices);
				PointIndicesPtr indicesPtr (new Indices());
				indicesPtr->indices = indices; // indices after NaN filtering
				//cout << "[POINT CLOUD]\tPoint cloud has " << indices.size() << " points after NAN elimination" << endl;
				//cout << "[POINT CLOUD]\tORGANIZED: " << cloudPtr->isOrganized() << endl;

				/* SEGMENTATION */
				PointIndicesPtr outliersNotNaN = indicesPtr;
				PointIndicesPtr inliers (new Indices);
				segmentCloud(cloudPtr, outliersNotNaN, inliers, indicesPtr);

				// outliersNotNaN corresponds to all points except ground plane points
				// all other planes are still there, only ground removed so far
				PointIndicesPtr outliersNotNaNSubsample = subsampleIndices(outliersNotNaN, 4);
				// remove ground plane and store in cloudNoPlane
				PointCloudPtr cloudNoPlane (new PointCloud);
				PointCloudPtr cloudGroundPlane (new PointCloud); // ground plane cloud
				extractCluster(cloudPtr, cloudNoPlane, outliersNotNaNSubsample, false, false);

				/* EUCLIDEAN CLUSTER EXTRACTION 
				 * Vector contains actual index info, indices of each detected cluster are saved here, 
				 * contains one instance of PointIndices for each detected cluster
				 * clusterIndices[0] for first cluster in point cloud, and so on
				 */
				PointIndicesVector clusterIndices;
				euclideanClusterExt(cloudPtr, clusterIndices, 0.1, 100, 1000000);

				/* PCL TO ROS CONVERSION FOR PUBLISHING */
				pcl::PCLPointCloud2 conv1;
				pcl::PCLPointCloud2 conv2;
				sensor_msgs::PointCloud2 toROS1;
				sensor_msgs::PointCloud2 toROS2;
				// plane points to PCL2 format
				pcl::toPCLPointCloud2(*cloudNoPlane, conv1);
				pcl::toPCLPointCloud2(*cloudGroundPlane, conv2);
				// conversion to ROS sensor_msg
				pcl_conversions::fromPCL(conv1, toROS1);
				pcl_conversions::fromPCL(conv2, toROS2);		
				// publishing results here
				cloudPubPlane.publish(toROS1);
				cloudPubGPlane.publish(toROS2);
				newData = false; // done processing point clouds
        }

        ros::spinOnce(); // process single round of callbacks
        loopRate.sleep();
        ++count;

    }
    return 0;
}
