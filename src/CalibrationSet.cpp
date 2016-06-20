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

#include "CalibrationSet.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdlib>
#include <iostream>
#include <regex>


namespace pinspect {


CalibrationSet::CalibrationSet(FilenameList leftList, FilenameList rightList)
	: leftNames(leftList), rightNames(rightList), typeOfImages(), checked(false) {

	if(leftNames.empty() || leftNames.size() != rightNames.size())
		throw ConsistencyError("Need to provide two consistent vectors of string");
}

void CalibrationSet::prepareAndCheck() {

	//only if this was not already checked
	if(checked) return;
	checked = true;

	//load a single image for consistency among the entire dataset
	cv::Mat image = cv::imread(leftNames[0], -1); // -1 for no conversions
	if (image.empty())
		throw InvalidImageError("Invalid image found '" + leftNames[0] + "'");

	typeOfImages = image.type();
	sizeOfImages = image.size();

	std::cout << "Found " << leftNames.size() << " images" << std::endl;
	std::cout << "check dataset consistency...";
	std::cout.flush();

	//now load(touch) all images for checking
	for (std::string s : leftNames) {
		image = cv::imread(s, -1);
		if (image.type() != typeOfImages)
			throw ConsistencyError("Image '" + s + "' has different type");

		if (image.size() != sizeOfImages)
			throw ConsistencyError("Image '" + s + "' has different size");
	}

	for (std::string s : rightNames) {
		image = cv::imread(s, -1);
		if (image.type() != typeOfImages)
			throw ConsistencyError("Image '" + s + "' has different type");

		if (image.size() != sizeOfImages)
			throw ConsistencyError("Image '" + s + "' has different size");
	}

	std::cout << "ok" << std::endl;
}

int CalibrationSet::size() const {
	return leftNames.size();
}

cv::Size CalibrationSet::imageSize() const {
	return sizeOfImages;
}

int CalibrationSet::imageType() const {
	return typeOfImages;
}

cv::Mat CalibrationSet::getLeftImage(size_t n) const {
	checkAccessRange(n);
	return cv::imread(leftNames[n], -1);
}

cv::Mat CalibrationSet::getRightImage(size_t n) const {
	checkAccessRange(n);
	return cv::imread(rightNames[n], -1);
}

std::string CalibrationSet::getLeftName(size_t n) const {
	checkAccessRange(n);
	return leftNames[n];
}

std::string CalibrationSet::getRightName(size_t n) const {
	checkAccessRange(n);
	return rightNames[n];
}

FilenameList CalibrationSet::getLeftList() const {
	return leftNames;
}

FilenameList CalibrationSet::getRightList() const {
	return rightNames;
}

void CalibrationSet::checkAccessRange(size_t n) const {
	if(n < leftNames.size())
			throw std::out_of_range("Access with out of range index: " + n);
}

std::ostream& operator<<(std::ostream& os, const CalibrationSet& obj) {
	os << "# of images=" << obj.size();
	os << ", image size="  << obj.imageSize();
	os << ", type=" << obj.imageType();
  return os;
}

/**
 * This class serves to describe a set of files on disk.
 * The format can be like this:
 *
 * 	path-tosome-dir/any-file-with<ID>.ext
 *
 * It provides methods to iterate through the content of a directory and to retrieve
 * id from a given file respecting the format.
 * It uses the boost::filesystem::path class.
 */
class FormattedPath {

	boost::filesystem::path wholePath;

	std::string separator;

public:
	/**
	 * Construct a FormattedPath instance.
	 * @param s string representing the path
	 * @param c string (or char) that serves to identify the id of a file
	 */
	FormattedPath(std::string s, std::string c) : wholePath(s), separator(c) {

		if(!wholePath.has_parent_path())
			wholePath = boost::filesystem::path("./") /= wholePath;
	}

	/**
	 * Check the consistency of the structure. Better here than within the
	 * constructor.
	 */
	bool checkSanity() {
		// TODO better
		return is_directory(wholePath);
	}

	/**
	 * Return the iterator used to inspect the directory hosting the general
	 * file format. We use boost::filesystem::directory_iterator class
	 */
	boost::filesystem::directory_iterator getIterator() {
		return boost::filesystem::directory_iterator(wholePath.parent_path());
	}

	/**
	 * Simple interpolation.
	 * If FormattedPath("dir-path/file#.ext") then the function returns
	 * the string: file<to_string(n)>.ext
	 * @param n the value to interpolate on.
	 */
	std::string interpolate(int n) {
		std::regex idRegex(separator);
		std::string format = wholePath.filename().generic_string();
		return std::regex_replace(format, idRegex, std::to_string(n));
	}

	/**
	 * Return the complete path object.
	 * @param n the value to interpolate on.
	 */
	boost::filesystem::path completePath(int n) {
		return wholePath.parent_path() /= boost::filesystem::path(interpolate(n));
	}

	/**
	 * Like completePath() but return the string representation.
	 */
	std::string interpolateWhole(int n) {
		return completePath(n).generic_string();
	}

	/**
	 * Try to match the given string with the format.
	 * If this is not possbile it returns -1, otherwise the matched id
	 * @param input the string to match
	 * @return the matched id
	 */
	int match(std::string input) {
		std::regex idRegex(separator);
		std::string format = wholePath.filename().generic_string();
		//convert the format with a functional regular expression
		std::regex regex(regex_replace(format, idRegex, "(\\d+)"));

		std::smatch matched;
		if(!std::regex_match(input, matched, regex))
			return -1;

		if(!matched[1].matched)
			return -1;

		return stoi(matched[1]);
	}
};

CalibrationSet
buildCalibrationSetFromRegex(std::string leftPathForm, std::string rightPathForm) {
	FormattedPath left(leftPathForm, "#");
	left.checkSanity();
	FormattedPath right(rightPathForm, "#");
	right.checkSanity();

	std::vector<int> foundIds;
	for(boost::filesystem::directory_entry dentry : left.getIterator()) {
		std::string s = dentry.path().filename().generic_string();

		int x = left.match(s);
		if(x >= 0)
			foundIds.push_back(x);
	}

	//we like to keep things in order
	sort(foundIds.begin(), foundIds.end());

	//check for each left found if there's the right one
	for(int id : foundIds) {
		if(!boost::filesystem::is_regular_file(right.interpolateWhole(id)))
			throw ConsistencyError("Not corresponding right file with id: " + id);
	}

	std::vector<std::string> leftNames, rightNames;
	for(int id : foundIds) {
		leftNames.push_back(left.interpolateWhole(id));
		rightNames.push_back(right.interpolateWhole(id));
	}

	return CalibrationSet(leftNames, rightNames);
}

/**
 * Read the content of the @param file in @param nodeName and fill
 * @param list with the content.
 */
static void readNodeList(cv::FileStorage file, std::string nodeName,
		FilenameList & list) {

  cv::FileNode node = file[nodeName];
  if(node.isNone())
    throw FileAccessError("Could not parse node: " + nodeName);

  for(std::string s : node)
    list.push_back(s);
}

CalibrationSet buildCalibrationSetFromFile(std::string filename) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	  if( !fs.isOpened() )
	    throw FileAccessError("Problems opening file: " + filename);

	  FilenameList leftNames, rightNames;
	  readNodeList(fs, "leftCamera", leftNames);
	  readNodeList(fs, "rightCamera", rightNames);

	  return CalibrationSet(leftNames, rightNames);
}


} //namespace pinspect
