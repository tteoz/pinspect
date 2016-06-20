/*
 * 		ELABORAZIONE DI DATI TRIDIMENSIONALI - 2014/2015
 *
 * 		Department of Information Engineering (DEI)
 *
 * 						University of Padova
 *
 *
 * This source was developed for the final course project as well as for the
 * competition organized by Loccioni Group.
 *
 *
 *  Created on: Apr 22, 2015
 *      Authors:
 *      	Alessandro Beltramin
 *      	Matteo Sartori
 *
 */

#ifndef INSPECTION_HPP
#define INSPECTION_HPP

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include "Calibration.hpp"


namespace pinspect {


/**
 * List of 2d corners to represent corners found in images.
 */
typedef std::vector<cv::Point2f> CornerList;


/**
 * Represents a pair of images passing through pipeline stages.
 */
struct ImagePair {
	cv::Mat left;
	cv::Mat right;

  /** same as @see CameraParameters */
	void store(std::string dirName, std::string nodeName) {
		std::string fileName = dirName + nodeName;
		cv::imwrite(fileName + "Left.bmp", left);
		cv::imwrite(fileName + "Right.bmp", right);
	}

  /** same as @see CameraParameters */
	void read(std::string dirName, std::string nodeName) {
		std::string fileName = dirName + nodeName;
		left =  cv::imread(fileName + "Left.bmp");
		right = cv::imread(fileName + "Right.bmp");

    if(!left.data || !right.data)
    	throw ReadError("Cannot read image '" + fileName + "'");
	}
};

/**
 * Represent the set of corners for algorithm purpose through pipeline stages.
 */
struct Corners {
	CornerList leftCorners;
	CornerList rightCorners;

	/** same as @see CameraParameters */
	void store(std::string dirName, std::string nodeName) {
		std::string fileName = dirName + nodeName + ".xml";
		cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
		fs << "leftCorners" << leftCorners;
		fs << "rightCorners" << rightCorners;
	}

	/** same as @see CameraParameters */
	void read(std::string dirName, std::string nodeName) {
		std::string fileName = dirName + nodeName + ".xml";
		cv::FileStorage fs(fileName, cv::FileStorage::READ);
		if (!fs.isOpened())
					throw ReadError("Cannot read file '" + fileName + "'");

		fs["leftCorners"] >> leftCorners;
		fs["rightCorners"] >> rightCorners;
	}
};

/**
 * Computed point clouds. No needs to serialize.
 */
struct PointCloudPtr {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud;

	void store(std::string dirName, std::string nodeName) {
	}

	void read(std::string dirName, std::string nodeName) {
	}
};


/**
 * Utility function to read two images and fill @see ImagePair.
 */
ImagePair buildImagePair(std::string left, std::string right);


/**
 * Pipeline stage that rectificate input images.
 */
class Rectification : public ProcessingStage<ImagePair, ImagePair,
		CameraParameters, CameraParameters, RectificationParameters> {

public:

	Rectification(std::string stageName) : ProcessingStage(stageName) {}

private:

	ImagePair process(ImagePair in, CameraParameters leftCamera,
			CameraParameters rightCamera, RectificationParameters rectP) override;
};


/**
 * This stage run cv::goodFeaturesToTrack on images.
 */
class CornerDetection : public ProcessingStage<Corners, ImagePair> {

public:

	CornerDetection(std::string stageName) : ProcessingStage(stageName) {}

private:

	Corners process(ImagePair input) override;
};


/**
 * This stage try to a grid model on the previous detected corners. Two
 * purpose: remove outlier and sort relevant points.
 *
 * IMPORTANT
 * It uses @see CirclesGridFinder from opencv source code.
 */
class GridFitting : public ProcessingStage<Corners, Corners, ImagePair> {

public:

	GridFitting(std::string stageName) : ProcessingStage(stageName) {}

private:

	Corners process(Corners input, ImagePair images) override;
};


/**
 * Run a refinement process to increase the pin location precision.
 */
class Refinement : public ProcessingStage<Corners, Corners, ImagePair> {

public:

	Refinement(std::string stageName) : ProcessingStage(stageName) {}

private:

	Corners process(Corners corners, ImagePair images) override;
};


/**
 * Compute depth information of stereo points. Then produce a pcl::PointCloud.
 *
 * Uses:
 *  cv::triangulatePoints
 *  cv::convertPointsFromHomogeneous
 */
class Triangulation : public ProcessingStage<PointCloudPtr, Corners,
		RectificationParameters> {

public:

	Triangulation(std::string stageName) : ProcessingStage(stageName) {}

private:

	PointCloudPtr process(Corners corners, RectificationParameters rectP) override;
};


/**
 * Run TransformationEstimationSVD from pcl to match an ideal model to the
 * processing cloud in order to compute rms error.
 */
class Registration : public ProcessingStage<PointCloudPtr, PointCloudPtr> {

public:

	Registration(std::string stageName) : ProcessingStage(stageName) {}

private:

	PointCloudPtr process(PointCloudPtr input) override;
};


} //pinspect


#endif //INSPECTION_HPP
