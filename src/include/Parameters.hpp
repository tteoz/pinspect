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

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <opencv2/core.hpp>
#include <boost/any.hpp>
#include <stdexcept>
#include <unordered_map>


namespace pinspect {


/**
 * To signal error in file access.
 */
struct BadFileAccess : std::runtime_error {
	BadFileAccess(std::string s) : std::runtime_error(s) {}
};

/**
 * Thrown if a bad parameter access is made. Like a parameter name that's not
 * define.
 */
struct ParameterNameError : std::logic_error {
	ParameterNameError(std::string s) : std::logic_error(s) {}
};


/**
 * Interface to easily access parameters for a given opencv algorithm. Those can
 * be retrieved from a file on disk by name and if that's not possible a default
 * value specified by @see add is used.
 * In fact we use std::unordered_map to map parameter's name to value and
 * cv::FileStorage to access serialized data on file.
 */
class Parameters {

	using any = boost::any;

public:

	Parameters() {}

	/**
	 * This accept @param fileName that contains each param (name, value) pair.
	 * Since the underlying code is from opencv this works for .xml and .yaml.
	 */
	Parameters(std::string fileName) : parametersFilename(fileName) {

		cv::FileStorage file(parametersFilename, cv::FileStorage::READ);

		if(!file.isOpened())
			throw BadFileAccess("Cannot open file: " + parametersFilename);
	}


	/**
	 * Add a parameter @param name and the corresponding default @param defaultValue.
	 * We check on disk to see if there is the parameter to bound with, otherwise
	 * we store the default provided value.
	 */
	template<typename T> void add(std::string name, T defaultValue) {

		cv::FileStorage file(parametersFilename, cv::FileStorage::READ);
		cv::FileNode node = file[name];

		T toStoreValue;
		if(node.isNone())
			//on disk there is no value to retrieve,
			//then we use the provided default
			toStoreValue = defaultValue;

		else
			node >> toStoreValue;

		//inject the value to be associated with @param name into
		//an object of type hold_any
		any anyH(toStoreValue);

		std::pair<std::string, any> insertP(name, anyH);

		//insert and check what happened
		bool r = parametersMap.insert(insertP).second;
		if(!r)
			throw ParameterNameError("Parameter '" + name + "' was already added");
	}


	/**
	 * Return what there's inside the map bounded with @param name.
	 */
	template<typename T> T get(std::string name) {
		T value;

		try {
			value = boost::any_cast<T>(parametersMap.at(name));

		} catch(std::out_of_range& oe) {
			throw ParameterNameError("Parameter '" + name + "' not found");
		}

		return value;
	}

	/**
	 * Print actual parameter map.
	 */
	void print();


private:

	/**
	 * Map that stores the association. It uses boost::any type for dynamic generics.
	 */
	std::unordered_map<std::string, any> parametersMap;

	/**
	 * File name used to access configuration on disk.
	 */
	std::string parametersFilename;
};


/**
 * Initialize the global instance parameters with default values for every
 * parameter options. And of course set possible association
 *   'parameter name' <-> 'actual value'
 */
Parameters populateDefaultParameters(std::string fileName);


} //namespace pinspect


#endif // PARAMETERS_HPP
