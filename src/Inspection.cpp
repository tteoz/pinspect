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

#include "Inspection.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/distances.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Circlesgrid.hpp"


namespace pinspect {


ImagePair buildImagePair(std::string left, std::string right) {

	ImagePair pair;
	pair.left = cv::imread(left);
	pair.right = cv::imread(right);
	return pair;
}

/**
 * To draw corners on image.
 */
static cv::Mat drawCorners(CornerList corners, cv::Mat image, int color) {
	int r = 5;
	for(auto p : corners)
		cv::circle(image, p, r, cv::Scalar(color), -1, 8, 0);

	return image;
}

/**
 * Scale image using cv::resize.
 */
static cv::Mat scaleImage(cv::Mat image, Parameters params) {
	cv::resize(image, image, params.get<cv::Size>("DisplaySize"));
	return image;
}

/**
 * Utility to visualize founded corners on images.
 */
static void visualizeImageCorners(Corners corners, ImagePair images,
		int color,	Parameters params) {

	cv::Mat left, right;
	left = scaleImage(drawCorners(corners.leftCorners, images.left, color),params);
	right = scaleImage(drawCorners(corners.rightCorners, images.right, color),params);

	cv::imshow("left", left);
	cv::imshow("right", right);
	cv::waitKey(0);
}

/**
 * Run pcl visualization system.
 */
static void visualizePointCloud(PointCloudPtr cloud) {

	pcl::visualization::PCLVisualizer viewer("PCL visualizer");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
		rgb(cloud.pcloud);

	viewer.addCoordinateSystem(1.0);
	viewer.addPointCloud < pcl::PointXYZRGB > (cloud.pcloud, rgb, "cloud");
	viewer.spin();
}

/**
 * Run pcl visualization system for two pointclouds.
 */
static void visualizeTwoPointCloud(PointCloudPtr first, PointCloudPtr second) {

	pcl::visualization::PCLVisualizer viewer("PCL visualizer");

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
		rgbf(first.pcloud);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
			rgbs(second.pcloud);

	viewer.addCoordinateSystem(1.0);
	viewer.addPointCloud < pcl::PointXYZRGB > (first.pcloud, rgbf, "reconstructed");
	viewer.addPointCloud < pcl::PointXYZRGB > (second.pcloud, rgbs, "ideal fit");
	viewer.spin();
}


ImagePair Rectification::process(ImagePair in, CameraParameters leftCamera,
		CameraParameters rightCamera, RectificationParameters rectP) {

	cv::Size imageSize(in.left.cols, in.left.rows);
	cv::Mat leftMap1;
	cv::Mat leftMap2;
	cv::Mat rightMap1;
	cv::Mat rightMap2;
	ImagePair out;

	std::cout << "Rectification of input images...";
	std::cout.flush();

	cv::initUndistortRectifyMap(leftCamera.cameraMatrix, leftCamera.distCoeffs,
			rectP.R1, rectP.P1, imageSize, CV_32FC1, leftMap1, leftMap2);

	cv::initUndistortRectifyMap(rightCamera.cameraMatrix, rightCamera.distCoeffs,
			rectP.R2, rectP.P2, imageSize, CV_32FC1, rightMap1, rightMap2);

	cv::remap(in.left, out.left, leftMap1, leftMap2, cv::INTER_LINEAR,
			cv::BORDER_CONSTANT, cv::Scalar());
	cv::remap(in.right, out.right, rightMap1, rightMap2, cv::INTER_LINEAR,
			cv::BORDER_CONSTANT, cv::Scalar());

	std::cout << "ok ";
	return out;
}


Corners CornerDetection::process(ImagePair input) {

	double qualityLevel = parameters.get<double>("qualityLevel");
	int maxCorners = parameters.get<int>("maxCorners");
	double minDistance = parameters.get<double>("minDistance");
	int blockSize = parameters.get<int>("blockSize");
	bool useHarrisDetector = parameters.get<bool>("useHarrisDetector");
	double k = parameters.get<double>("kCornerParams");

	std::cout << "Corner detection...";
	std::cout.flush();

	Corners output;
  cv::cvtColor(input.left, input.left, cv::COLOR_BGR2GRAY);
	cv::goodFeaturesToTrack(input.left, output.leftCorners, maxCorners, qualityLevel,
			minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

	cv::cvtColor(input.right, input.right, cv::COLOR_BGR2GRAY);
	cv::goodFeaturesToTrack(input.right, output.rightCorners, maxCorners, qualityLevel,
				minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

	std::cout << "ok ";

	if(parameters.get<bool>("CornerDetectVisualize"))
		visualizeImageCorners(output, input, 255, parameters);

	return output;
}


Corners GridFitting::process(Corners input, ImagePair images) {

	cv::Size gridSize = parameters.get<cv::Size>("GridFittingSize");
	int color = parameters.get<int>("GridFittingColor");
	Corners output;

	std::cout << "Grid fitting...";

	CirclesGridFinderParameters finderParams;
	CirclesGridFinder finderLeft(gridSize, input.leftCorners, finderParams);
	finderLeft.findHoles();
	finderLeft.getHoles(output.leftCorners);

	CirclesGridFinder finderRight(gridSize, input.rightCorners, finderParams);
	finderRight.findHoles();
	finderRight.getHoles(output.rightCorners);

	std::cout << "ok ";

	if(parameters.get<bool>("GridFittingVisualize"))
		visualizeImageCorners(output, images, color, parameters);

	return output;
}

/**
 * @see findSquares
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0 ) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Returns sequence of squares detected on the image. The sequence is stored
 * in the specified memory storage.
 */
static void findSquares(const cv::Mat& image,
		std::vector<std::vector<cv::Point> >& squares, int approxEpsilon,
		Parameters params) {

	//fetch parameters
	int thresh = params.get<int>("RefineThreshold");
	int N = params.get<int>("RefineN");
  float minEdgeSize = params.get<float>("RefineMinEdgeSize");
  float minArea = params.get<float>("RefineMinArea");
  float maxArea = params.get<float>("RefineMaxArea");

	squares.clear();

	cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

	// down-scale and upscale the image to filter out the noise
	cv::pyrDown(image, pyr, cv::Size(image.cols / 2, image.rows / 2));
	cv::pyrUp(pyr, timg, image.size());

	std::vector<std::vector<cv::Point>> contours;

	// find squares in every color plane of the image
	for (int c = 0; c < 3; c++) {
		int ch[] = { c, 0 };
		cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

		// try several threshold levels
		for (int l = 0; l < N; l++) {
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if (l == 0) {
				// apply Canny. Take the upper threshold from slider
				// and set the lower to 0 (which forces edges merging)
				cv::Canny(gray0, gray, 0, thresh, 5);

				// DEBUG
				//imshow("Canny", gray);
				//waitKey(0);

				// dilate canny output to remove potential
				// holes between edge segments
				cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));

				// DEBUG
				//imshow("dilate", gray);
				//waitKey(0);
			} else {
				// apply threshold if l!=0:
				//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
				gray = gray0 >= (l + 1) * 255 / N;
			}

			// find contours and store them all as a list
			cv::findContours(gray, contours, cv::RETR_EXTERNAL /*/RETR_LIST*/,
					cv::CHAIN_APPROX_SIMPLE);

			std::vector<cv::Point> approx;

			// test each contour
			for (size_t i = 0; i < contours.size(); i++) {
				// approximate contour with accuracy proportional
				// to the contour perimeter
				cv::approxPolyDP(cv::Mat(contours[i]), approx,
						cv::arcLength(cv::Mat(contours[i]), true) * approxEpsilon * 0.01/*0.02*/,
						true);

				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) < maxArea
						&& // Maximum area
						fabs(cv::contourArea(cv::Mat(approx))) > minArea
						&& // Minimum area
						norm(approx[0] - approx[1]) > minEdgeSize
						&& // Minimum edge size
						norm(approx[1] - approx[2]) > minEdgeSize
						&& cv::isContourConvex(cv::Mat(approx))) {
					double maxCosine = 0;

					for (int j = 2; j < 5; j++) {
						// find the maximum cosine of the angle between joint edges
						double cosine = fabs(
								angle(approx[j % 4], approx[j - 2], approx[j - 1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					if (maxCosine < 0.3)
						squares.push_back(approx);
				}
			}
		}
	}
}


/**
 * Given approximative pin locations (detected thanks to specularties),
 * this function looks for the square section of each pin
 * in a neighborhood of the given point. Refined positions are returned.
 *
 * @param inPoints Input vector of approximative pin locations
 * @param image Input image
 * @param refinedPoints Output vector of refined pin locations
 * @param windowSize Size() of the search window
 *
 * @throws exception when at least one square section is not found
 *
 */
static void refine(const CornerList &inPoints, const cv::Mat &image,
		CornerList &refinedPoints, cv::Size windowSize, Parameters params) {

	// Check input
	CV_Assert(image.cols > 25 && image.rows > 25);
	CV_Assert(windowSize.width > 5 && windowSize.height > 5);

	int dog1_ker = params.get<int>("RefineDog1Ker");
	int dog2_ker = params.get<int>("RefineDog2Ker");
	int tentatives = params.get<int>("RefineTentatives");

	// Output vector
	CornerList tempRefinedPoints;

	int a = windowSize.width / 2;
	int b = windowSize.height / 2;

	for (cv::Point2f p : inPoints) {

		cv::Point2f offset = cv::Point2f((int) p.x - a, (int) p.y - b);

		// Take a window around each point
		cv::Rect window(offset, cv::Point2f((int) p.x + a, (int) p.y + b));

		cv::Mat img_window = image(window);

		if (params.get<bool>("RefineDebug")) {
			std::cout << "window: [(" << (int) p.x - a << "," << (int) p.y - b
					<< "),(" << (int) p.x + a << "," << (int) p.y + b << ")]\n";
			//imshow("img_window",img_window);
			//waitKey(0);
		}

		// Detect square

		std::vector<std::vector<cv::Point> > squares;

		int kernel1 = dog2_ker;
		int kernel2 = dog1_ker;
		cv::Mat result;
		for (int k = 0; k < tentatives; k++) {
			int tuner = (k % 2) ? ((k + 1) / 2) : -((k + 1) / 2); // Tune +- k/2

			if (squares.size() > 0)
				break;

			cv::Mat dog_2, dog_1;
			//input_image.copyTo(img);

			// -- PASS-BAND Filter (DoG)

			cv::GaussianBlur(img_window, dog_2,
					cv::Size((tuner + kernel1) * 2 + 1, (tuner + kernel1) * 2 + 1), 0);
			cv::GaussianBlur(img_window, dog_1,
					cv::Size(kernel2 * 2 + 1, kernel2 * 2 + 1), 0);

			cv::subtract(dog_2, dog_1, result); //result = dog_2 - dog_1;

			cv::normalize(result, result, 0, 255, cv::NORM_MINMAX);

			int approxEpsilon = 4;

			for (int z = 0; z < 9; z++) {
				findSquares(result, squares, approxEpsilon + z, params);

				if (squares.size() > 0)
					break;
			}

			if (params.get<bool>("RefineDebug")) {
				std::cout << "Squares found: " << squares.size() << std::endl;
				for (size_t i = 0; i < squares.size(); i++) {
					const cv::Point* p = &squares[i][0];
					int n = (int) squares[i].size();
					cv::polylines(result, &p, &n, 1, true, cv::Scalar(0, 255, 0), 1/*3*/,
							cv::LINE_AA);
				}
			}
		}

		// Refine pin tip

		/// Get the moments
		std::vector<cv::Moments> mu(squares.size());
		for (unsigned int i = 0; i < squares.size(); i++) {
			mu[i] = cv::moments(squares[i], false);
		}

		///  Get the mass centers:
		std::vector<cv::Point2f> mc(squares.size());
		for (unsigned int i = 0; i < squares.size(); i++) {
			mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		if (params.get<bool>("RefineDebug")) {
			std::cout << "Mass centers found: " << mc.size() << std::endl;
		}

		cv::Point2f pinCenter;

		if (mc.size() > 0) {
			pinCenter = mc[0];

			for (unsigned int i = 1; i < mc.size(); i++) {
				pinCenter += mc[i];
			}

			pinCenter *= 1. / (mc.size());
			if (params.get<bool>("RefineDebug")) {
				cv::circle(result, pinCenter, 1, cv::Scalar(0, 0, 255));
			}

			tempRefinedPoints.push_back(cv::Point2f(offset + pinCenter));

		} else {

			// Pin center not found
//			throw new std::exception;

			//then keep original point
			tempRefinedPoints.push_back(p);

		}

		if (params.get<bool>("RefineDebug")) {
			cv::Mat output;
			cv::resize(result, output, cv::Size(), 10, 10);
			cv::imshow("square detector", output);
			cv::waitKey(0);
		}

	}

	// Allow in-place output
	refinedPoints.clear();
	refinedPoints = tempRefinedPoints;
}


Corners Refinement::process(Corners input, ImagePair images) {
	std::cout << "Refinement of pin position...";
	std::cout.flush();

	cv::Size wsize = parameters.get<cv::Size>("RefineWindowSize");

	Corners output;
  refine(input.leftCorners, images.left, output.leftCorners, wsize,
  		parameters);
  refine(input.rightCorners, images.right, output.rightCorners, wsize,
  		parameters);

  if(parameters.get<bool>("RefinementVisualize"))
  		visualizeImageCorners(output, images, 255, parameters);

  std::cout << "ok ";
  return output;
}


/**
 * To compactly create a pcl::Point.
 */
static pcl::PointXYZRGB make_point(float x, float y, float z, cv::Vec3i col) {
	pcl::PointXYZRGB point;
	point.x = x;
	point.y = y;
	point.z = z;
	point.r = col[0];
	point.g = col[1];
	point.b = col[2];
	return point;
}


PointCloudPtr Triangulation::process(Corners corners, RectificationParameters rectP) {

	cv::Mat leftPoints(corners.leftCorners);
	cv::Mat rightPoints(corners.rightCorners);
	cv::Mat triangPoints;

	std::cout << "Triangulation...";

	cv::triangulatePoints(rectP.P1, rectP.P2, leftPoints, rightPoints, triangPoints);

	//transposition is necessary because the format used by
	//convertPointsFromHomogeneous
	triangPoints = triangPoints.t();

	cv::Mat euclidPoints;
	cv::convertPointsFromHomogeneous(triangPoints, euclidPoints);


	//build the point cloud
	PointCloudPtr output;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);
	output.pcloud = points;

	for (int i = 0; i < euclidPoints.rows; i++) {
		cv::Point3f point = euclidPoints.at<cv::Point3f>(i, 0);

		output.pcloud->push_back(make_point(point.x, point.y, point.z, cv::Vec3i(255,255,255)));
	}

	std::cout << "ok ";

	if(parameters.get<bool>("TriangulationVisualize"))
		visualizePointCloud(output);

	return output;
}


// Dataset specific constants
float smallSmallDist	= 0.002576f; // 0.0025f

/**
 * This function serves to represent the current type of grid in the dataset.
 * Number of pin and location. It's expected for extending the inspection system
 * to specialize this representation.
 */
static cv::Point2f mappingIndexToPattern(int j, int i) {

  float x = 0.0f;
  float y = 0.0f;

  switch(j) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
      x = j*smallSmallDist;
      break;

    default:
      throw new std::exception;
  }

  switch(i) {
    case 0:
    case 1:
      y = i*smallSmallDist;
      break;

    default:
      throw new std::exception;
  }

  return cv::Point2f( x, y );
}


PointCloudPtr Registration::process(PointCloudPtr input) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr idealPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 16; j++) {

			cv::Point2f v = mappingIndexToPattern(j, i);

			pcl::PointXYZRGB p;
			p.x = v.x;
			p.y = v.y;
			p.z = 0;
			p.r = 0;
			p.g = 0;
			p.b = 255;
			idealPoints->points.push_back(p);
		}
	}

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB> transform;
	Eigen::Matrix4f matrix;
	transform.estimateRigidTransformation(*idealPoints, *input.pcloud, matrix);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredPoints(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	PointCloudPtr registered;
	registered.pcloud = registeredPoints;
	pcl::transformPointCloud(*idealPoints, *registeredPoints, matrix);

	std::cout << "Registration, computing errors: " << std::endl;
	std::cout.flush();

	float meanError = 0;
	for (int i = 0; i < 32; i++) {

		pcl::PointXYZRGB p = input.pcloud->points[i];
		pcl::PointXYZRGB m = registeredPoints->points[i];
		float dist = pcl::euclideanDistance(p, m);
		meanError += dist;
		std::cout << "\tfor pin " << i << " is " << dist << std::endl;

		float threshold = 0.0005f;
		if (dist > threshold) {
			input.pcloud->points[i].r = 255; // Mark red
			input.pcloud->points[i].g = 0;
			input.pcloud->points[i].b = 0;
		} else {
			input.pcloud->points[i].r = 0;
			input.pcloud->points[i].g = 255; // Mark green
			input.pcloud->points[i].b = 0;
		}
	}

	meanError /= 32;
	std::cout << "Mean error is " << meanError << " ";

	if(parameters.get<bool>("RegistrationVisualize"))
			visualizeTwoPointCloud(input, registered);

	return input;
}


} //pinspect
