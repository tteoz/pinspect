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

#ifndef CALIBRATION_SET_HPP
#define CALIBRATION_SET_HPP

#include <opencv2/core/types.hpp>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>


namespace pinspect {

/**
 * Represent a list of file names.
 */
typedef std::vector<std::string> FilenameList;


/**
 * To signal missed consistency among various images.
 */
struct ConsistencyError : std::runtime_error {
	ConsistencyError(std::string s) : std::runtime_error(s) {}
};

/**
 * To signal missed consistency among various images.
 */
struct InvalidImageError : std::runtime_error {
	InvalidImageError(std::string s) : std::runtime_error(s) {}
};

/**
 * To signal problem with file access.
 */
struct FileAccessError : std::runtime_error {
	FileAccessError(std::string s) : std::runtime_error(s) {}
};


/**
 * Represent the set of images used for the calibration of a stereo rig.
 * It perform consistency check among images, that's all images must be of the
 * same type and size.
 */
class CalibrationSet {


public:

	CalibrationSet() : typeOfImages(), checked(false) {}

	/**
	 * Construct an instance from two lists of vector, containing images file name.
	 * It checks that two vectors are of the same size and that all images have
	 * the same characteristics.
	 */
	CalibrationSet(FilenameList leftNames, FilenameList rightNames);

	/**
	 * Scan all left and right images to see if size and type are the same.
	 */
	void prepareAndCheck();

	/**
	 * @return the number of images in the dataset
	 */
	int size() const;

	/**
	 * @return the size of the image used for this dataset
	 */
	cv::Size imageSize() const;

	/**
	 * @return image opencv indicator of the type
	 */
	int imageType() const;

	/**
	 * @return an opencv image obj, built with the file name in the n-th position
	 * of left images.
	 */
	cv::Mat getLeftImage(size_t n) const;

	/**
	 * Same as @see getLeftImage but for right images.
	 */
	cv::Mat getRightImage(size_t n) const;

	/**
	 * @return the name of the n-th image.
	 */
	std::string getLeftName(size_t n) const;

	/**
	 * Same as @see getLeftName but for right images.
	 */
	std::string getRightName(size_t n) const;

	/**
	 * @return the whole left list of names
	 */
	FilenameList getLeftList() const;

	/**
	 * @return the whole right list of names
	 */
	FilenameList getRightList() const;


private:

	/**
	 * Internal routine for bounding check.
	 */
	void checkAccessRange(size_t n) const;

	/**
	 * Left vector of image names.
	 */
	FilenameList leftNames;

	/**
	 * Right vector of image names.
	 */
	FilenameList rightNames;

	/**
	 * Internal opencv format for representing image type;
	 */
	int typeOfImages;

	/**
	 * Size of images in this dataset.
	 */
	cv::Size sizeOfImages;

	/**
	 * State to signal that checking was not already performed.
	 */
	bool checked;
};


/**
 * Overload for printing CalibrationSet's instances
 */
std::ostream& operator<<(std::ostream& os, const CalibrationSet& obj);


/**
 * This function builds the dataset from a file containing a list
 * of images to load.
 */
CalibrationSet buildCalibrationSetFromFile(std::string filename);


/**
 * It's used to build an instance of CalibrationSet from a description
 * of the input dataset like this:
 *
 * 	left:  <PATH-TO-LEFT>/left-image#.jpg
 * 	right: <PATH-TO RIGHT>/right-image#.jpg
 *
 * Where the '#' matches the id of any file (some-image1234.png).
 */
CalibrationSet buildCalibrationSetFromRegex(std::string leftPathForm,
		std::string rightPathForm);


} //namespace pinspect


#endif // CALIBRATION_SET_H
