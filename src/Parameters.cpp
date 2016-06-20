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

#include "Parameters.hpp"

#include "ProcessingStage.hpp"


namespace pinspect {


Parameters populateDefaultParameters(std::string fileName) {
	Parameters params(fileName);

	//calibration pattern
	params.add("PatternSize", cv::Size(20, 15));
	params.add("PatternEdgeSize", 0.004f);

	//cv::findCirclesGrid options
	params.add("thresholdStep", 10.f);
	params.add("minThreshold", 50.f);
	params.add("maxThreshold", 220.f);

	params.add("filterByArea", true);
	params.add("minArea", 25.f);
	params.add("maxArea", 5000.f);

	params.add("filterByCircularity", true);
	params.add("minCircularity", 0.800000012f);
	params.add("maxCircularity", 3.40282347e+38f);

	params.add("filterByConvexity", true);
	params.add("minConvexity", 0.949999988f);
	params.add("maxConvexity", 3.40282347e+38f);

	//rectification maps computation parameters
	params.add("RectifyAlpha", -1.f);

	//corner detection parameters
	params.add("qualityLevel", 0.01);
	params.add("maxCorners", 70);
	params.add("minDistance", 40.0);
	params.add("blockSize", 3);
	params.add("useHarrisDetector", false);
	params.add("kCornerParams", 0.04);
	params.add("CornerDetectVisualize", false);

	//grid fitting parameters
	params.add("GridFittingSize", cv::Size(16, 2));
	params.add("GridFittingVisualize", false);

	//refinement parameters
	params.add("RefineThreshold", 200);
	params.add("RefineN", 11);
	params.add("RefineMinEdgeSize", 10.f);
	params.add("RefineMinArea", 100.f);
	params.add("RefineMaxArea", 20.f*20.f);
	params.add("RefineDog1Ker", 0);
	params.add("RefineDog2Ker", 10);
	params.add("RefineWindowSize", cv::Size(30, 30));
	params.add("RefineTentatives", 20);
	params.add("RefineDebug", false);
	params.add("RefinementVisualize", false);

	//triangulation options
	params.add("TriangulationVisualize", false);

	//registration options
	params.add("RegistrationVisualize", false);

	//visualization parameters
	params.add("DisplaySize", cv::Size(1920, 1080));
	params.add("GridFittingColor", 0);

	return params;
}


void Parameters::print() {

	std::cout << "Printing parameters" << std::endl << std::endl;

	std::cout << "CALIBRATION PATTERN" << std::endl;
	std::cout << "PatternSize: " << get<cv::Size>("PatternSize") << std::endl;
	std::cout << "PatternEdgeSize: " << get<float>("PatternEdgeSize") << std::endl << std::endl;

	std::cout << "CV::FINDCIRCLESGRID OPTIONS" << std::endl;
	std::cout << "thresholdStep: " << get<float>("thresholdStep") << std::endl;
	std::cout << "minThreshold: " << get<float>("minThreshold") << std::endl;
	std::cout << "maxThreshold: " << get<float>("maxThreshold") << std::endl;
	std::cout << "filterByArea: " << get<bool>("filterByArea") << std::endl;
	std::cout << "minArea: " << get<float>("minArea") << std::endl;
	std::cout << "maxArea: " << get<float>("maxArea") << std::endl;
	std::cout << "filterByCircularity: " << get<bool>("filterByCircularity") << std::endl;
	std::cout << "minCircularity: " << get<float>("minCircularity") << std::endl;
	std::cout << "maxCircularity: " << get<float>("maxCircularity") << std::endl;
	std::cout << "filterByConvexity: " << get<bool>("filterByConvexity") << std::endl;
	std::cout << "minConvexity: " << get<float>("minConvexity") << std::endl;
	std::cout << "maxConvexity: " << get<float>("maxConvexity") << std::endl << std::endl;

	std::cout << "RECTIFICATION MAPS OPTIONS" << std::endl;
	std::cout << "RectifyAlpha: " << get<float>("RectifyAlpha") << std::endl << std::endl;

	std::cout << "CORNER DETECTION OPTIONS" << std::endl;
	std::cout << "qualityLevel: " << get<double>("qualityLevel") << std::endl;
	std::cout << "maxCorners: " << get<int>("maxCorners") << std::endl;
	std::cout << "minDistance: " << get<double>("minDistance") << std::endl;
	std::cout << "blockSize: " << get<int>("blockSize") << std::endl;
	std::cout << "useHarrisDetector: " << get<bool>("useHarrisDetector") << std::endl;
	std::cout << "kCornerParams: " << get<double>("kCornerParams") << std::endl;
	std::cout << "CornerDetectVisualize: " << get<bool>("CornerDetectVisualize") << std::endl << std::endl;

	std::cout << "GRID FITTING OPTIONS" << std::endl;
	std::cout << "GridFittingSize: " << get<cv::Size>("GridFittingSize") << std::endl;
	std::cout << "GridFittingVisualize: " << get<bool>("GridFittingVisualize") << std::endl << std::endl;

	std::cout << "REFINEMENT OPTIONS" << std::endl;
	std::cout << "RefineThreshold: " << get<int>("RefineThreshold") << std::endl;
	std::cout << "RefineN: " << get<int>("RefineN") << std::endl;
	std::cout << "RefineMinEdgeSize: " << get<float>("RefineMinEdgeSize") << std::endl;
	std::cout << "RefineMinArea: " << get<float>("RefineMinArea") << std::endl;
	std::cout << "RefineMaxArea: " << get<float>("RefineMaxArea") << std::endl;
	std::cout << "RefineDog1Ker: " << get<int>("RefineDog1Ker") << std::endl;
	std::cout << "RefineDog2Ker: " << get<int>("RefineDog2Ker") << std::endl;
	std::cout << "RefineWindowSize: " << get<cv::Size>("RefineWindowSize") << std::endl;
	std::cout << "RefineTentatives: " << get<int>("RefineTentatives") << std::endl;
	std::cout << "RefineDebug: " << get<bool>("RefineDebug") << std::endl;
	std::cout << "RefinementVisualize: " << get<bool>("RefinementVisualize") << std::endl << std::endl;

	std::cout << "VISUALIZATION OPTIONS" << std::endl;
	std::cout << "DisplaySize: " << get<cv::Size>("DisplaySize") << std::endl;
	std::cout << "GridFittingColor: " << get<int>("GridFittingColor") << std::endl;
	std::cout << "TriangulationVisualize: " << get<bool>("TriangulationVisualize") << std::endl;
	std::cout << "RegistrationVisualize: " << get<bool>("RegistrationVisualize") << std::endl << std::endl;
}


} //namespace pinspect
