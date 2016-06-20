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

#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/calib3d.hpp>

#include "ProcessingStage.hpp"
#include "CalibrationSet.hpp"


namespace pinspect {


/** Pattern representation type */
typedef std::vector<cv::Point3f> Pattern;
/** List of Pattern */
typedef std::vector<Pattern> PatternList;

/** same as @see Pattern but for pattern found in image (2D) */
typedef std::vector<cv::Point2f> ImagePattern;
/** list of */
typedef std::vector<ImagePattern> ImagePatternList;

/** It serves to memorize missed pattern in some image during searches */
typedef std::vector<bool> BoolVector;


/**
 * To signal errors while reading content on disk.
 */
struct ReadError : std::runtime_error {
	ReadError(std::string s) : std::runtime_error(s) {}
};


/**
 * Pattern found in images, by findCirclesGrid procedure.
 */
struct FoundPatterns {
	ImagePatternList patterns;
	BoolVector foundPatterns;

	void store(std::string dirName, std::string nodeName) {}

	void read(std::string dirName, std::string nodeName) {}
};

/**
 * Represents a camera model.
 */
struct CameraParameters {
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  cv::Size imageSize;

  /**
   * To serialize the structure on disk.
   */
  void store(std::string dirName, std::string nodeName) {
  	std::string fileName = dirName + nodeName + ".xml";
		cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
		fs << nodeName << "{";
		fs << "cameraMatrix" << cameraMatrix;
		fs << "distCoeffs" << distCoeffs;
		fs << "imageSize" << imageSize;
		fs << "}";
	}

  /**
   * To read data previously serialized on disk.
   */
  void read(std::string dirName, std::string nodeName) {
  	std::string fileName = dirName + nodeName + ".xml";
  	cv::FileStorage fs(fileName, cv::FileStorage::READ);
  	if(!fs.isOpened())
  		throw ReadError("Cannot read file '" + fileName + "'");

  	cv::FileNode node = fs[nodeName];
  	if(node.isNone())
  		throw ReadError("Cannot parse node '" + nodeName + "'");

  	node["cameraMatrix"] >> cameraMatrix;
  	node["distCoeffs"] >> distCoeffs;
  	node["imageSize"] >> imageSize;
  }
};

/**
 * Represents a stereo camera model.
 */
struct StereoParameters {
  cv::Mat R;
  cv::Mat T;
  cv::Mat E;
  cv::Mat F;

  /** same as @see CameraParameters */
  void store(std::string dirName, std::string nodeName) {
  	std::string fileName = dirName + nodeName + ".xml";
  	cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
  	fs << nodeName << "{";
  	fs << "R" << R;
  	fs << "T" << T;
  	fs << "E" << E;
  	fs << "F" << F;
  	fs << "}";
  }

  /** same as @see CameraParameters */
  void read(std::string dirName, std::string nodeName) {
  	std::string fileName = dirName + nodeName + ".xml";
		cv::FileStorage fs(fileName, cv::FileStorage::READ);
		if (!fs.isOpened())
			throw ReadError("Cannot read file '" + fileName + "'");

		cv::FileNode node = fs[nodeName];
		if (node.isNone())
			throw ReadError("Cannot parse node '" + nodeName + "'");

		node["R"] >> R;
		node["T"] >> T;
		node["E"] >> E;
		node["F"] >> F;
	}
};

/**
 * Parameter used to rectify an image.
 */
struct RectificationParameters {
  cv::Mat R1;
  cv::Mat R2;
  cv::Mat P1;
  cv::Mat P2;
  cv::Mat Q;

  /** same as @see CameraParameters */
  void store(std::string dirName, std::string nodeName) {
  	std::string fileName = dirName + nodeName + ".xml";
  	cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
  	fs << nodeName << "{";
  	fs << "R1" << R1;
  	fs << "R2" << R2;
  	fs << "P1" << P1;
  	fs << "P2" << P2;
  	fs << "Q"  << Q;
  	fs << "}";
  }

  /** same as @see CameraParameters */
  void read(std::string dirName, std::string nodeName) {
  	std::string fileName = dirName + nodeName + ".xml";
		cv::FileStorage fs(fileName, cv::FileStorage::READ);
		if (!fs.isOpened())
			throw ReadError("Cannot read file '" + fileName + "'");

		cv::FileNode node = fs[nodeName];
		if (node.isNone())
			throw ReadError("Cannot parse node '" + nodeName + "'");

		node["R1"] >> R1;
		node["R2"] >> R2;
		node["P1"] >> P1;
		node["P2"] >> P2;
		node["Q"]  >> Q;
	}
};


/**
 * Pipeline stage to find calibration pattern in all dataset images.
 * It wraps cv::findCirclesGrid.
 */
class FindGridPattern : public ProcessingStage<FoundPatterns, FilenameList> {

public:

	FindGridPattern(std::string stageName) : ProcessingStage(stageName),
			dataset(nullptr) {}

	void setDataset(CalibrationSet *x) { dataset = x; }

private:

	FoundPatterns process(FilenameList imageNames) override;

	/**
	 * This particular stage needs a pointer to the dataset structure.
	 */
	CalibrationSet *dataset;
};


/**
 * Pipeline stage to calibrate a single camera.
 * It uses cv::calibrateCamera.
 */
class SingleCalibration : public ProcessingStage<CameraParameters, FoundPatterns> {

public:

	SingleCalibration(std::string stageName) : ProcessingStage(stageName),
			dataset(nullptr) {}

	void setDataset(CalibrationSet *x) { dataset = x; }

private:

	CameraParameters process(FoundPatterns input) override;

	/** same as @see FindGridPatter */
	CalibrationSet *dataset;
};


/**
 * Pipeline stage to calibrate a stereo rig.
 * Uses cv::stereoCalibrate.
 */
class StereoCalibration : public ProcessingStage<StereoParameters,
		FoundPatterns, CameraParameters, FoundPatterns, CameraParameters> {

public:

	StereoCalibration(std::string stageName) : ProcessingStage(stageName) {}

private:

	StereoParameters process(FoundPatterns leftPatts , CameraParameters leftCamera,
			FoundPatterns rightPatts, CameraParameters rightCamera) override;

};


/**
 * Stage to compute rectification maps.
 * Uses cv::stereoRectify.
 */
class RectifyCalibration : public ProcessingStage<RectificationParameters,
		CameraParameters, CameraParameters, StereoParameters> {

public:

	RectifyCalibration(std::string stageName) : ProcessingStage(stageName) {}

private:

	RectificationParameters process(CameraParameters leftCamera,
			CameraParameters rightCamera, StereoParameters stereoCamera) override;

};


} //pinspect

#endif //CALIBRATION_HPP
