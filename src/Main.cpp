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

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <algorithm>
#include <cstdio>
#include <map>
#include <memory>

#include "Calibration.hpp"
#include "Inspection.hpp"
#include "Parameters.hpp"
#include "Pipeline.hpp"
#include "ResultConnector.hpp"


using namespace pinspect;

namespace po = boost::program_options;


static Pipeline pipeline;

static CalibrationSet dataset;

/**
 * Map of connector objects created on input to pass to various ProcessingStages.
 */
static std::vector<boost::any> inputConnectorList;

/**
 * Bind a name to each ProcessingStage objects.
 * It's used to set up links during pipeline constructions.
 */
static std::map<std::string, ProcessingInterface*> stageMap;


/**
 * Allocate processing stages associated with names.
 */
static void initializeStageMap() {
	stageMap.insert(
	{
		{"findGridLeft", new FindGridPattern("findGridLeft")},
		{"findGridRight",new FindGridPattern("findGridRight")},
		{"leftCamera", new SingleCalibration("leftCamera")},
		{"rightCamera", new SingleCalibration("rightCamera")},
		{"stereoCamera", new StereoCalibration("stereoCamera")},
		{"rectifyMaps", new RectifyCalibration("rectifyMaps")},
		{"rectification", new Rectification("rectification")},
		{"cornerDetection", new CornerDetection("cornerDetection")},
		{"gridFitting", new GridFitting("gridFitting")},
		{"refinement", new Refinement("refinement")},
		{"triangulation", new Triangulation("triangulation")},
		{"registration", new Registration("registration")}
	});
}

/**
 * Free memory.
 */
static void freeStageMap() {
	for(auto x : stageMap)
		delete x.second;
}

/**
 * Retrieve of ProcessingStages. It has to be generics because they are of
 * different type.
 */
template<typename T> static T* getStage(std::string s) {
	return static_cast<T*>(stageMap[s]);
}

/**
 * Allocate a new InputConnector object and @return a pointer to it.
 */
template<typename Type> static InputConnector<Type>* newInputConnector(Type x) {

	InputConnector<Type> newObj;
	newObj.putValue(x);
	inputConnectorList.push_back(boost::any(newObj));

	// this automatically returns a pointer to the underlying object
	return boost::any_cast<InputConnector<Type>>(&inputConnectorList.back());
}


/**
 * This is for reading specific command arguments and to setup input
 * connections. Then the stage is added to the pipeline object.
 */
static void setCalibration(po::variables_map vm) {

	std::string leftFormat = "";
	std::string rightFormat= "";

	if(!vm.count("left-format") || !vm.count("right-format")) {
		std::cout << "\n\nOptions --left-format and --right-format mandatory" << std::endl;
		std::cout << "\nExample: --left-format=datasetLoc/Leftimg#.jpg" << std::endl;
		std::cout << "Where the '#' stand for the ID of the image" << std::endl;
		std::exit(1);
	}

	leftFormat = vm["left-format"].as<std::string>();
	rightFormat = vm["right-format"].as<std::string>();

	dataset = buildCalibrationSetFromRegex(leftFormat, rightFormat);

	FindGridPattern *findLeft = getStage<FindGridPattern>("findGridLeft");
	findLeft->setInput(newInputConnector(dataset.getLeftList()));
	findLeft->setDataset(&dataset);

	FindGridPattern *findRight = getStage<FindGridPattern>("findGridRight");
	findRight->setInput(newInputConnector(dataset.getRightList()));
	findRight->setDataset(&dataset);

	SingleCalibration *calibLeft = getStage<SingleCalibration>("leftCamera");
	calibLeft->setInput(findLeft->getResultConnector());
	calibLeft->setDataset(&dataset);

	SingleCalibration *calibRight = getStage<SingleCalibration>("rightCamera");
	calibRight->setInput(findRight->getResultConnector());
	calibRight->setDataset(&dataset);

	StereoCalibration *stereo = getStage<StereoCalibration>("stereoCamera");
	stereo->setInput(findLeft->getResultConnector(), calibLeft->getResultConnector(),
			findRight->getResultConnector(), calibRight->getResultConnector());


	pipeline.addStage(findLeft).addStage(calibLeft);
	pipeline.addStage(findRight).addStage(calibRight);
	pipeline.addStage(stereo);
}


/**
 * @see setCalibration
 */
static void setRectifyMaps(po::variables_map vm) {

	RectifyCalibration *stage = getStage<RectifyCalibration>("rectifyMaps");
	stage->setInput(getStage<SingleCalibration>("leftCamera")->getResultConnector(),
			getStage<SingleCalibration>("rightCamera")->getResultConnector(),
			getStage<StereoCalibration>("stereoCamera")->getResultConnector());

	pipeline.addStage(stage);
}

/**
 * @see setCalibration
 */
static void setRectification(po::variables_map vm) {

	std::string leftImage = "";
	std::string rightImage= "";

	if (!vm.count("left-image") || !vm.count("right-image")) {
		std::cout << "\n\nOptions --left-image and --right-image mandatory" << std::endl;
		std::exit(1);
	}

	leftImage = vm["left-image"].as<std::string>();
	rightImage = vm["right-image"].as<std::string>();

	Rectification *stage = getStage<Rectification>("rectification");

	stage->setInput(newInputConnector(buildImagePair(leftImage, rightImage)),
			getStage<SingleCalibration>("leftCamera")->getResultConnector(),
			getStage<SingleCalibration>("rightCamera")->getResultConnector(),
			getStage<RectifyCalibration>("rectifyMaps")->getResultConnector());

	pipeline.addStage(stage);
}

/**
 * @see setCalibration
 */
static void setCornerDetection(po::variables_map vm) {

	CornerDetection *stage = getStage<CornerDetection>("cornerDetection");
	stage->setInput(getStage<Rectification>("rectification")->getResultConnector());
	pipeline.addStage(stage);
}

/**
 * @see setCalibration
 */
static void setGridFitting(po::variables_map vm) {

	GridFitting *stage = getStage<GridFitting>("gridFitting");
	stage->setInput(getStage<CornerDetection>("cornerDetection")->getResultConnector(),
			getStage<Rectification>("rectification")->getResultConnector());
	pipeline.addStage(stage);
}

/**
 * @see setCalibration
 */
static void setRefinement(po::variables_map vm) {

	Refinement *stage = getStage<Refinement>("refinement");
	stage->setInput(getStage<GridFitting>("gridFitting")->getResultConnector(),
			getStage<Rectification>("rectification")->getResultConnector());
	pipeline.addStage(stage);
}

/**
 * @see setCalibration
 */
static void setTriangulation(po::variables_map vm) {

	Triangulation *stage = getStage<Triangulation>("triangulation");
	stage->setInput(getStage<Refinement>("refinement")->getResultConnector(),
			getStage<RectifyCalibration>("rectifyMaps")->getResultConnector());
	pipeline.addStage(stage);
}

/**
 * @see setCalibration
 */
static void setRegistration(po::variables_map vm) {

	Registration *stage = getStage<Registration>("registration");
	stage->setInput(getStage<Triangulation>("triangulation")->getResultConnector());
	pipeline.addStage(stage);
}


typedef void (*commandHandler)(po::variables_map vm);

typedef std::pair<std::string, commandHandler> CommandPair;

typedef std::vector<CommandPair> CommandList;

/**
 * Represent commands with a description of which operation to perform.
 */
static std::vector<std::pair<std::string,std::string>> commandDescription = {
		{"calib", "perform calibration"},
		{"rectmap", "compute rectification maps"},
		{"rectify", "rectify input images"},
		{"cornerdet", "corner detection"},
		{"gridfit", "fitting the grid model"},
		{"refine", "pin position refinement"},
		{"triang", "triangulate and project in 3D"},
		{"regist", "model registration, compute errors"}
};

/**
 * Associate to each command name the static function to call to
 * create the ProcessingStage object specific for that command.
 */
static CommandList commandList = {
		{"calib", setCalibration},
		{"rectmap", setRectifyMaps},
		{"rectify", setRectification},
		{"cornerdet", setCornerDetection},
		{"gridfit", setGridFitting},
		{"refine", setRefinement},
		{"triang", setTriangulation},
		{"regist", setRegistration}
};

static void printCommands() {
	for(auto e : commandDescription)
		std::printf("   %s\t%s\n", e.first.c_str(), e.second.c_str());
}


/**
 * Decide how to build the pipeline while parsing command line arguments.
 * @param from decide where to start executing stage
 * @param to what stage to stop at.
 *
 * Every stage between from and to are added in the order decided statically by
 * @see commandList object.
 */
static void parseCommands(std::string from, std::string to, po::variables_map vm) {

	auto fromLambda = [from](CommandPair p){ return p.first == from; };
	auto toLambda   = [to](CommandPair p){ return p.first == to; };

	auto fromIt = std::find_if(commandList.begin(), commandList.end(), fromLambda);
	auto toIt = std::find_if(commandList.begin(), commandList.end(), toLambda);

	if(from == "start")
		fromIt = commandList.begin();
	if(to == "end")
		toIt = commandList.end() - 1;

	//boundary check
	if(fromIt == commandList.end())
		throw std::runtime_error("Invalid command '" + from + "'");

	if(fromIt > toIt)
		throw std::runtime_error("Invalid commands '" + from + "' '" + to + "'");

	if(toIt == commandList.end() && to.front() != '-')
		throw std::runtime_error("Invalid command '" + to + "'");
	else if(toIt == commandList.end())
		//we assume the user wants only one stage
		toIt = fromIt;


	std::cout << "Commands to execute: ";
	std::for_each(fromIt, toIt+1, [vm](CommandPair p) {
		std::cout << "'" << p.first << "' ";
		p.second(vm);
	});

	std::cout << std::endl;
}


std::string optionDescription = "\nUsage: pinspect [OPTIONS] FROM-CMD [TO-CMD]\n\nPrimary options";


int main(int argc, char *argv[]) {

	std::string paramsFilename = "parameters.xml";
	std::string resultsDirname = "results";
	std::string fromCommand = "";
	std::string toCommand = "";

	//boost object for describe program options.
	po::options_description desc(optionDescription);
	desc.add_options()
	    ("help,h", "produce help message")
			("print-params,p", "print set parameters")
	    ("parameters", po::value<std::string>(&paramsFilename), "set parameters file")
			("results", po::value<std::string>(&resultsDirname), "set results directory")
			("left-format", po::value<std::string>(),"left images form of the dataset")
			("right-format", po::value<std::string>(),"right images form of the dataset")
			("left-image,l", po::value<std::string>(), "left image to inspect")
			("right-image,r", po::value<std::string>(), "right image to inspect")
			("commands", po::value<std::vector<std::string>>(), "from command to command");

	po::positional_options_description p;
	p.add("commands", 2);

	//this actually parses argv
	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).positional(p).options(desc).run(),vm);
	po::notify(vm);

	if(vm.count("help")) {
		std::cout << desc << std::endl;
		std::cout << "Commands available:" << std::endl;
		printCommands();
		std::exit(1);
	}

	if(!vm.count("commands")) {
		std::cout << "Need to specify commands to execute" << std::endl;
		std::exit(1);
	}

	std::vector<std::string> commands = vm["commands"].as<std::vector<std::string>>();
	fromCommand = commands[0];
	toCommand   = commands[1];
	if(toCommand == "")
		toCommand = fromCommand;

	//initialize static members of ProcessignInterface
	//which are configurations common to all stages
	Parameters parameters = populateDefaultParameters(paramsFilename);
	if(vm.count("print-params"))
		parameters.print();
	ProcessingInterface::setParameters(parameters);
	ProcessingInterface::setOutputDirectory(resultsDirname + "/");
	//create if not existing
	boost::filesystem::create_directory(resultsDirname);

	initializeStageMap();

	try {
		parseCommands(fromCommand, toCommand, vm);
	}
	catch(std::runtime_error& e) {
		std::cout << e.what() << std::endl;
		std::cout << "Commands available:" << std::endl;
		printCommands();
		std::exit(1);
	}

	//this checks for missing inputs
	pipeline.checkStagesInput();

	auto start = std::chrono::steady_clock::now();
	pipeline.execute();

	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	std::cout << "Total execution time: ";
	std::cout << std::chrono::duration<double, std::milli>(diff).count() << " ms"
			<< std::endl;

	freeStageMap();
	return 0;
}
