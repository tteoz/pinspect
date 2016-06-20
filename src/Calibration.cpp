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

#include "Calibration.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <bitset>
#include <cassert>
#include <iostream>
#include "Parameters.hpp"


namespace pinspect {


/**
 * Static member for @see ProcessingInterface
 */
std::string ProcessingInterface::outputDirectory;

/**
 * Static member for @see ProcessingInterface
 */
Parameters ProcessingInterface::parameters;



/**
 * To compose the ideal pattern in calibration procedure.
 */
static Pattern createPattern(Parameters params) {

	cv::Size patternSize = params.get<cv::Size>("PatternSize");
	float edge = params.get<float>("PatternEdgeSize");

	int w = patternSize.width, h = patternSize.height;
  assert(w > 0 && h > 0);

  // CAUTION - the order in which we compose the vector is important
  Pattern ptt;
  for (int i = 0; i < h; i++)
    for (int j = 0; j < w; j++)
      ptt.push_back(cv::Point3f(edge * j, edge * i, 0));

  return ptt;
}


/**
 * Access the parameters file and return an object @see SimpleBlobDetector::Params,
 * used to configure @see SimpleBlobDetector algorithm.
 */
static cv::Ptr<cv::FeatureDetector> getBlobDetector(Parameters params) {

	cv::SimpleBlobDetector::Params detectorParams;
	detectorParams.thresholdStep = params.get<float>("thresholdStep");
	detectorParams.minThreshold = params.get<float>("minThreshold");
	detectorParams.maxThreshold = params.get<float>("maxThreshold");

	detectorParams.filterByArea = params.get<bool>("filterByArea");
	detectorParams.minArea = params.get<float>("minArea");
	detectorParams.maxArea = params.get<float>("maxArea");

	detectorParams.filterByCircularity = params.get<bool>("filterByCircularity");
	detectorParams.minCircularity = params.get<float>("minCircularity");
	detectorParams.maxCircularity = params.get<float>("maxCircularity");

	detectorParams.filterByConvexity = params.get<bool>("filterByConvexity");
	detectorParams.minConvexity = params.get<float>("minConvexity");
	detectorParams.maxConvexity = params.get<float>("maxConvexity");

	return cv::SimpleBlobDetector::create(detectorParams);
}


/**
 *	Execute cv::findCirclesGrid with parameters read from file.
 */
static bool findCirclesGrid(Parameters parameters, cv::Mat image,
		ImagePattern &corners) {

	cv::Size patternSize = parameters.get<cv::Size>("PatternSize");

	return cv::findCirclesGrid(image, patternSize, corners,
			cv::CALIB_CB_SYMMETRIC_GRID, getBlobDetector(parameters));
}


FoundPatterns FindGridPattern::process(FilenameList imageNames) {

	//check dataset
	dataset->prepareAndCheck();

	FoundPatterns output;
	for (unsigned int i = 0; i < imageNames.size(); ++i) {
		std::string name = imageNames[i];
		cv::Mat image = cv::imread(name);

		ImagePattern corners;
		bool wasFound = findCirclesGrid(parameters, image, corners);

		std::cout << name;
		if(wasFound) {
			output.foundPatterns.push_back(true),  std::cout << " found";
			output.patterns.push_back(corners);
		}
		else
			output.foundPatterns.push_back(false), std::cout << " NOT found";

		std::cout << std::endl;
	}
	return output;
}


CameraParameters SingleCalibration::process(FoundPatterns input) {

	PatternList patternList(input.patterns.size(), createPattern(parameters));

	CameraParameters cameraP;
	std::vector<cv::Mat> rVecs, tVecs;
	cameraP.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cameraP.distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	cameraP.imageSize = dataset->imageSize();

	std::cout << "Camera calibration...";
	std::cout.flush();

	double error = cv::calibrateCamera(patternList, input.patterns, cameraP.imageSize,
			cameraP.cameraMatrix, cameraP.distCoeffs, rVecs, tVecs);

	std::cout << "re-projection error " << error << std::endl;
	return cameraP;
}


/**
 * This function is used for generic filtering. @param input is the vector to
 * filter, @param indices is a vector of booleans.
 * If indices[i] == true then the input[i] is kept, otherwise dropped.
 *
 * Used by @see StereoCalibration::process.
 */
template<typename T> static std::vector<T> filterByIndex(std::vector<T> input,
		BoolVector indices) {

	if(input.size() != indices.size())
		throw std::invalid_argument("Input vectors of different size");

	std::vector<T> result;
	for(unsigned int i=0; i < input.size(); ++i)
		if(indices[i] == true)
			result.push_back(input[i]);

	return result;
}


StereoParameters StereoCalibration::process(FoundPatterns leftPatterns,
		CameraParameters leftCamera, FoundPatterns rightPatterns,
		CameraParameters rightCamera) {

	//prune left and right pattern vectors
	ImagePatternList leftPruned = filterByIndex(leftPatterns.patterns,
			filterByIndex(rightPatterns.foundPatterns, leftPatterns.foundPatterns));

	ImagePatternList rightPruned = filterByIndex(rightPatterns.patterns,
			filterByIndex(leftPatterns.foundPatterns, rightPatterns.foundPatterns));

	StereoParameters stereoP;
	PatternList patternList(leftPruned.size(), createPattern(parameters));

	std::cout << "Stereo calibration...";
	std::cout.flush();

	double error = cv::stereoCalibrate(patternList,
			leftPruned, rightPruned,
			leftCamera.cameraMatrix, leftCamera.distCoeffs,
			rightCamera.cameraMatrix, rightCamera.distCoeffs,
			leftCamera.imageSize, stereoP.R, stereoP.T, stereoP.E, stereoP.F);

	std::cout << "re-projection error " << error << std::endl;

	return stereoP;
}


RectificationParameters RectifyCalibration::process(CameraParameters leftCamera,
		CameraParameters rightCamera, StereoParameters stereoCamera) {

	RectificationParameters rectifyP;

	std::cout << "Computing rectification map...";
	std::cout.flush();

	cv::stereoRectify(leftCamera.cameraMatrix, leftCamera.distCoeffs,
			rightCamera.cameraMatrix, rightCamera.distCoeffs, leftCamera.imageSize,
			stereoCamera.R, stereoCamera.T, rectifyP.R1, rectifyP.R2, rectifyP.P1,
			rectifyP.P2, rectifyP.Q, cv::CALIB_ZERO_DISPARITY,
			parameters.get<float>("RectifyAlpha"));

	std::cout << "ok ";

	return rectifyP;
}


} //pinspect
