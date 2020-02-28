// ROS
#include "ros/ros.h" 
#include "std_msgs/String.h"
#include "kobuki_msgs/BumperEvent.h"
#include "kobuki_msgs/MotorPower.h"
#include "geometry_msgs/Twist.h"

// PCL
#include <pcl/io/pcd_io.h>
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
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

// C Libraries
#include <cmath>
#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>


using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointXYZRGB PointXYZ;
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
PointXYZ centerPoint; // cluster centroid

cv_bridge::CvImagePtr cvPtr; // cv bridge for communication
cv_bridge::CvImagePtr cvPtrTop;

Mat receivedImg; //received img from camera / callback
Mat receivedTopImg;
Mat referenceTopImg; // this saves a reference to the image when the robot was stopped after collission

// global, to see if new data to process arrived
bool newCloudData = false;
bool newImgData = false;
bool newTopImgData = false;
// This is a signal that the robot is in search mode (looking for a object in a circular motion)
bool searchObject = false;
// This is a signal that we have found an object we want to collide with it
bool foundObject = false;
// This is an indicator if we should just move forward because we are directly in front of the object
bool nearObject = false;
// This is a signal that we are returning to center location after a collision attempt
bool isReturning = false;
bool returning = false; // need this bool for returning to marker
// for incoming bumper events, tells us if we collided with an object to start detecting if we can shove smth
bool receivedBump = false;
bool stopped = false; // stopping robot after bump triggered
// calculated angles for turning
bool gotAngle = false; 

kobuki_msgs::BumperEvent bumperEvent;

/* CALLBACKS */
void bumpCallback(const kobuki_msgs::BumperEvent &e){
    bumperEvent = e; receivedBump = true;
}

/* callback gets called if new data is available so we can process it,
 * we copy the point cloud to a global var (cloudPtr) and set newData to true
 */
void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg (*input, *cloudPtr); newCloudData = true;	
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
   	try { 
	   cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
	}
   	catch (cv_bridge::Exception& e){
     	ROS_ERROR("cv_bridge exception: %s", e.what());
     	return;
   	}
   	receivedImg = cvPtr->image;
   	newImgData = true;
}

void imageTopCallback(const sensor_msgs::ImageConstPtr& msg){
   	try { 
	   cvPtrTop = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
	}
   	catch (cv_bridge::Exception& e){
     	ROS_ERROR("cv_bridge exception: %s", e.what());
     	return;
   	}
   	receivedTopImg = cvPtrTop->image;
   	newTopImgData = true;
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

void IndicesDeleter(pcl::PointIndices * p){}

/* get closest cluster index from given cluster indices and an input point cloud, we iterate over found clusters and choose closest returning the index */
int getClosestCluster(IndicesVector clusterIndices, PointCloudPtr cloud, ExtractIndices ex){
	int closestClusterId = -1;
	double distance = 9999999;
	double centerP;
	PointXYZ cp;
	PointCloudPtr closestObj;
	IndicesPtr ptr;
	float avgR = 0 ; float avgG = 0; float avgB = 0;
	float avgX = 0 ; float avgY = 0; float avgZ = 0;
	Centroid centroid;
	double closestZ = 9999999;

	if(cloud->size() != 0){
		for (int i=0; i<clusterIndices.size(); i++) {
			cout << "[CLUSTER]\tCluster "<< i << " has " << clusterIndices[i].indices.size() << " elements" << endl;
			PointCloudPtr temp (new PointCloud);
			ptr = IndicesPtr(&(clusterIndices[i]), IndicesDeleter);
			ex.setInputCloud(cloud) ;
    		ex.setIndices(ptr);
    		ex.setNegative(false) ;
    		ex.filter(*temp);
    		pcl::computeCentroid(*temp, cp);
    		double distToCenter = sqrt(cp.x*cp.x + cp.y*cp.y*0 + cp.z*cp.z) ;
    		if (distToCenter < distance) {
				if(closestZ >= cp.y){
					closestClusterId = i;
					distance = distToCenter;
					closestObj = temp;
				}
    		}
		}	
	}
	cout << "[CLUSTER]\tClosest cluster has index " << closestClusterId << endl;
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
		cout << "[CLUSTER]\tAverage RGB: " << (avgR+avgG+avgB)/3. << endl << "\n";
	}
}

/* returns a geometry_msgs::Twist move command we can send to our bot */
geometry_msgs::Twist moveBot(double linX, double linY, double linZ, double angX, double angY, double angZ){
	geometry_msgs::Twist moveCommand;
	moveCommand.linear.x = linX; moveCommand.linear.y = linY; moveCommand.linear.z = linZ;
	moveCommand.angular.x = angX; moveCommand.angular.y = angY; moveCommand.angular.z = angZ;
	return moveCommand;
}
			
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "controller");
    ros::NodeHandle nodeHandle; // Main access point for communication with ROS

    /* PUBLISHING TOPICS */
	static const ros::Publisher cloudPubPlane = nodeHandle.advertise<sensor_msgs::PointCloud2>("/plane", 1000);
	static const ros::Publisher cloudPubGPlane = nodeHandle.advertise<sensor_msgs::PointCloud2>("/groundplane", 1000);
    static const ros::Publisher controlMotor = nodeHandle.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1000);
    static const ros::Publisher controlRobot = nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    /* SUBSCRIBED TOPICS */
    static const ros::Subscriber bumpSub = nodeHandle.subscribe("/mobile_base/events/bumper", 1000, bumpCallback);
    static const ros::Subscriber pointsSub = nodeHandle.subscribe("/camera/depth/points", 1000, pointsCallback);
	static const ros::Subscriber imgSub = nodeHandle.subscribe("/camera/rgb/image_raw", 1000, imageCallback);
	static const ros::Subscriber imgTopSub = nodeHandle.subscribe("/camera_top/rgb/image_raw_top", 1000, imageTopCallback);

    ros::Rate loopRate(10); // loop with with x Hz (10 Hz -> 10 main loop calls per sec)
    int count = 0; // simple counter that gets incremented every main loop access
	int turn = 0; // how many turns did our robot do so far!

	pcl::SACSegmentation<pcl::PointXYZRGB> seg; // Segmentation object
	int currentGPpoints;
	int currentNGPpoints;

	double angularSpeed; 
	int turnTimer;
	int pushTimer;

	double t1, t2;
	double nearCount;

    while(ros::ok()) {
        if(count==0) { // Start motor & search routine
            kobuki_msgs::MotorPower mPower;
            mPower.state = mPower.ON;
            controlMotor.publish(mPower); 
			searchObject = true; gotAngle = false;
        }

		/* Routine after colission/bump with an object */
		if(receivedBump && !isReturning) {	
			if(!stopped){
				pushTimer = count + 30;
				cout << "[INFO]\t\tOBJECT HIT...STOPPING ROBOT!" << std::endl;
				geometry_msgs::Twist moveCommand = moveBot(0, 0, 0, 0, 0 ,0);
				controlRobot.publish(moveCommand); // STOP robot
				stopped = true;
				ros::Duration(1).sleep();
				t1 = ros::Time::now().sec;
				if(newTopImgData){
					Mat gauss, hsv, mask;
					// apply guassian filter with kernel = 3
					GaussianBlur(receivedTopImg, gauss, Size(3, 3), 0, 0, BORDER_DEFAULT);
					Size s = receivedTopImg.size();
					int w = s.width; // get width of received img
					int h = s.height; // get height of received img
					double c_x, c_y = 0.0;
					//cvtColor(gauss, hsv, CV_BGR2HSV); // convert to HSV
					// select lower and upper bound of HSV range, filter out and write into mask
					//inRange(hsv, Scalar(110, 60, 60), Scalar(130, 255, 255), mask); 
					referenceTopImg = mask;

					/*
					for(int i = 0; i < gauss.rows; i++){
						const double* Mi = gauss.ptr<double>(i); // get row pointer
						double rowSum = 0;
						for(int j = 0; j < gauss.cols; j++){
							rowSum += gauss.at<cv::Vec3b>(i, j)[0]; // get B value at point (y, x);  
						}
						cout << "row " << i << "sum: " << rowSum << endl;
						
					}
					*/

					// moments describe how image pixel intensities are distributed according to their location
					// weighted avg. of image pixel intensities, we can get the centroid or spatial information from the image moment
					Moments M = moments(mask);
					c_x = M.m10 / M.m00;
					c_y = M.m01 / M.m00;
				}
			}
			// 3 sec push
			if(count < pushTimer){
				geometry_msgs::Twist moveCommand = moveBot(0, 0, 0, 0, 0 ,0);
				moveCommand.linear.x = 0.1;
				controlRobot.publish(moveCommand);
				// COLLISION ROUTINE HERE TO-DO	
			} else { 
				t2 = ros::Time::now().sec;
				cout << "[INFO]\t\tDONE PUSHING... " << t2-t1 << " SEC" << std::endl;
				receivedBump = false; foundObject = false; isReturning = true;
			}
        }

		/* Routine if robot is looking for a pointcloud to target */
		if(searchObject){		
			if(!gotAngle){ // this is for calculating the angle to hit with a pulled magic nr.
				cout << endl << "[INFO]\t\tTURN " << turn+1 << endl;
				cout << "[INFO]\t\tSEARCHING FOR OBJECT..." << endl;			
				double speed = 45; // degrees/sec
				// generate random target number
				std::srand(std::time(NULL)); // random seed gen
				int magic = rand()%6+1;
				double angle = magic * 45;
				// conversion from angle to radians
				angularSpeed = speed * 2. * PI / 360;
				double relativeAngle = angle * 2. * PI / 360;
				turnTimer = count + angle / 45 * 10;
				cout << "[INFO]\t\tTurning to angle: " << angle << "°, with angular speed: " << angularSpeed << endl;
				gotAngle = true;
			}

			/*
			 * double t0 = ros::Time::now().toSec;
			 * while(currentAngle != relativeAngle){
			 * 	double t1 = ros::Time::now().toSec;
			 *	currentAngle = angularSpeed * (t1-t0);
			 * }
			 */

			// STOP after angle hit, can not prevent from overturning here!!!
			if(count > turnTimer){
				geometry_msgs::Twist moveCommand = moveBot(0,0,0,0,0,0);
				controlRobot.publish(moveCommand);
				searchObject = false; receivedBump = false;
				foundObject = true; nearObject = false; nearCount = 0;
			}
			// turning here
			geometry_msgs::Twist moveCommand;
			moveCommand.linear.x = moveCommand.linear.y = moveCommand.linear.z = 0;
			moveCommand.angular.x = moveCommand.angular.y = moveCommand.angular.z = 0;
			moveCommand.angular.z = abs(angularSpeed);   
			controlRobot.publish(moveCommand);
    	} 

		/* PCL part here */
        if(newCloudData && foundObject && !nearObject){ // wait for new data from pointcloud callback!
			/* START FILTERING HERE */
			//std::cerr << "\n[POINT CLOUD]\tUnprocessed point cloud [size: "<< cloudPtr->size() <<  " height: " << cloudPtr->height 
			//	<< " width: " << cloudPtr->width << "]" << std::endl;

			/* DOWN SAMPLING 
			 * Create a VoxelGrid filter and perform down sampling on the input cloud 
			 *
			 * pcl::VoxelGrid <pcl::PointXYZRGB> downSampler;
			 * downSampler.setInputCloud(cloudPtr);
			 * downSampler.setLeafSize(0.04, 0.04, 0.04);
			 * downSampler.filter(*cloudPtrSmall); // result is saved in cloudSmall
			 * std::cerr << "[POINT CLOUD]\tPoint cloud after filtering [size: "<< cloudPtrSmall->points.size() << " height: " << cloudPtrSmall->width*cloudPtrSmall->height
			 *			<< " data points (" << pcl::getFieldsList (*cloudPtrSmall) << ")]" << std::endl;
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
			if(indices.size()>0){
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

					//cout << "[CLUSTER]\tGround plane points: " << inliers->indices.size() << endl;
					pcl::PointIndices::Ptr outliers = invertIndices(inliers, 640*480); // Get outliers by swapping indices
					//cout << "[CLUSTER]\tNon ground plane points: " << outliers->indices.size() << endl;

					currentGPpoints = inliers->indices.size(); currentNGPpoints = outliers->indices.size();

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
				//cout << "[POINT CLOUD]\tSUBSAMPLED OBJECT CLOUD (cloudNoPlane), WITH SIZE: " << cloudNoPlane->points.size() << endl;

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
				ec.setMaxClusterSize(50000);
				ec.setSearchMethod(kdTree);
				ec.setInputCloud(cloudNoPlane);
				ec.extract(clusterIndices);

				if(clusterIndices.size() > 0){
					/* CLUSTER PROCESSING
					* To separate each cluster out of vector<PointIndices> we have to iterate through cluster_indices,
					* create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
					*/

					// heuristically chosen nr. of ground plane points we need to hit for an object to be "near"
					if(currentGPpoints < 60000 || clusterIndices[0].indices.size() >= 50000){
						nearCount = count; nearObject = true;
						cout << "[INFO]\t\tML BOT IS NEAR THE OBJECT...FAST FORWARD" << endl;
					}  
					else {
						int closestClusterId = getClosestCluster(clusterIndices, cloudNoPlane, extract);
						postProcessClusters(cloudNoPlane, clusterIndices[closestClusterId]);

						// SAVE .PCD FILES OF ONLY THE SELECTED CLUSTER
						std::string dest = boost::to_string(turn);
						std::string path(dest);

						if (boost::filesystem::exists(path)) __throw_ios_failure;
						else boost::filesystem::create_directory(path);

						struct timeval tp;
						gettimeofday(&tp, NULL);
						long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
						std::string fileString = path + "/" + boost::to_string(turn) + "_" + boost::to_string(ms) + ".pcd";
						// create temp cloud and filter specific indices (closest cluster)
						PointCloudPtr temp (new PointCloud);
						IndicesPtr ptr = IndicesPtr(&(clusterIndices[closestClusterId]), IndicesDeleter);
						extract.setInputCloud(cloudNoPlane) ;
    					extract.setIndices(ptr);
    					extract.setNegative(false) ;
    					extract.filter(*temp);
						// maybe binary mode here to save some time
						pcl::io::savePCDFileASCII(fileString, *temp);
					}	 
				} else {
					cout << "[INFO]\t\tNO CLUSTER FOUND SEARCHING AGAIN... " << endl;
					foundObject = false; searchObject = true; gotAngle = false; nearObject = false;
				}
				/* PCL TO ROS CONVERSION FOR PUBLISHING */
				pcl::PCLPointCloud2 conv1;
				sensor_msgs::PointCloud2 toROS1;
				// plane points to PCL2 format
				pcl::toPCLPointCloud2(*cloudNoPlane, conv1);
				// conversion to ROS sensor_msg
				pcl_conversions::fromPCL(conv1, toROS1);	
				// publishing results here
				cloudPubPlane.publish(toROS1);
			} else {
				cout << "[INFO]\t\tNO CLUSTER FOUND SEARCHING AGAIN... " << endl;
				foundObject = false; searchObject = true; gotAngle = false; nearObject = false;
			}
			newCloudData = false; // done processing point clouds
        }

		/* Routine for found object we want to navigate to */
        if(foundObject && !receivedBump) {
			geometry_msgs::Twist moveCommand;
			if(!nearObject){
				if(centerPoint.x > 0.1){	
					moveCommand.angular.z = -0.2;
					moveCommand.linear.x = 0.1;
				} else if(centerPoint.x < -0.1){
					moveCommand.angular.z = 0.2;
					moveCommand.linear.x = 0.1;
				} else moveCommand.linear.x = 0.2;
			} else {	
				if(count < nearCount+40) moveCommand.linear.x = 0.2; // 4 sec forward
				else moveCommand.linear.x = 0.0; // stop after 4 sec
			}
			controlRobot.publish(moveCommand);
        }

		if(newImgData && isReturning){
			stopped = false; receivedBump = false; // immediately suppress any bumper events that can happen while turning
     		Mat gauss, hsv, mask;
			// apply guassian filter with kernel = 3
			GaussianBlur(receivedImg, gauss, Size(3, 3), 0, 0, BORDER_DEFAULT);
			Size s = receivedImg.size();
			std::vector<std::vector<Point> > v;
			int w = s.width; // get width of received img
			int h = s.height; // get height of received img
			double c_x, c_y = 0.0;
			cvtColor(gauss, hsv, CV_BGR2HSV); // convert to HSV
			// select lower and upper bound of HSV range, filter out and write into mask
			inRange(hsv, Scalar(20, 100, 100), Scalar(35, 255, 255), mask); 
			// moments describe how image pixel intensities are distributed according to their location
			// weighted avg. of image pixel intensities, we can get the centroid or spatial information from the image moment
			Moments M = moments(mask);
			/*
			if(M.m00>0){
				Point pt(M.m10/M.m00, M.m01/M.m00);
				circle(receivedImg, pt, 20, Scalar(0, 0, 255), -1); // paint circle of centroid
			}
			*/
			c_x = M.m10 / M.m00;
			c_y = M.m01 / M.m00;
			int nonzero = countNonZero(mask);
			//cout << "Moment x center: " << c_x << " ; count: " << nonzero << endl;
			geometry_msgs::Twist moveCommand;
			if(nonzero == 0){
				moveCommand.angular.z = 0.2;
				if(returning) { // exit if we can not see the yellow marker anymore
					cout << "[INFO]\t\tRETURNED TO CENTER MARKER..." << endl;
					isReturning = false; returning = false;
					moveCommand.angular.z = 0; moveCommand.linear.x = 0;
					controlRobot.publish(moveCommand);
					searchObject = true; gotAngle = false; 
					turn++;
				}
			} else {
				// first time yellow is seen we start returning until we see nothing
				if(!returning) cout << "[INFO]\t\tRETURNING TO CENTER MARKER..." << endl;
				returning = true;
				if(c_x < w/2 - 100){
					moveCommand.angular.z = 0.2;
					moveCommand.linear.x = 0.1;
				} else if (c_x > w/2 + 100) {
					moveCommand.angular.z = -0.2;
					moveCommand.linear.x = 0.1;
				} else {
					moveCommand.linear.x = 0.2;
				}
			}
			controlRobot.publish(moveCommand);
			newImgData = false;
		}
        ros::spinOnce(); // process single round of callbacks
        loopRate.sleep();
        ++count;
    }
    return 0;
}
