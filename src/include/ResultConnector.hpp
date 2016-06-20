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

#ifndef RESULT_CONNECTOR_HPP
#define RESULT_CONNECTOR_HPP


namespace pinspect {


/**
 * To signal problem fetching a value.
 */
struct ResultConnectorError : std::runtime_error {
	ResultConnectorError(std::string s) : std::runtime_error(s) {}
};


/**
 * Abstract class that represent a connection to a value produced by a
 * ProcessingStage. And we have polymorphism on ResultConnector, which is
 * a template.
 */
template<typename Type> class Connector {

public:

	virtual void putValue(Type x) = 0;

	virtual Type fetchValue() = 0;

	virtual bool checkConnection() = 0;

	virtual ~Connector() {}

};


/**
 * Represent a connection to a value produced by another ProcessingStage.
 * It serves to purpose: the first retrieve the result in a easy way, other is
 * to save the produced value on disk for retrieving the next run of the program.
 */
template<typename Type> class ResultConnector : public Connector<Type> {

public:

	ResultConnector(std::string directoryName, std::string nodeName) :
		outputDirectory(directoryName), nodeName(nodeName), isReady(false),
		futureAvailable(false) {}

	/**
	 * Keep the value and store on disk.
	 */
	void putValue(Type x) override {
		isReady = true;
		value = x;

		//store result
		x.store(outputDirectory, nodeName);
	}

	/**
	 * If the value has not produced maybe it's on disk. This try to fetch it
	 * and if it's not being found then signal an error (@see ResultConnectorError).
	 */
	Type fetchValue() override {

		if (!isReady) {
			try {
				//try to read from disk
				value.read(outputDirectory, nodeName);

			} catch(...) {
				throw ResultConnectorError("Value not already produced nor saved on disk");
			}
		}

		return value;
	}

	/**
	 * This serves to signal that the value even not ready right now, it will be
	 * produced by a previous stage and for the moment of execution everything will
	 * be ok.
	 */
	void setFutureAvailable() {
		futureAvailable = true;
	}

	/**
	 * Check if the value is ready by calling @fetchValue.
	 */
	bool checkConnection() override {
		if(futureAvailable)
			return true;

		bool r = true;
		try {
			fetchValue();
		} catch(...) {
			r = false;
		}
		return r;
	}


private:

	/**
	 * Directory to store and read intermediate results.
	 */
	std::string outputDirectory;

	/**
	 * The actual connector name.
	 */
	std::string nodeName;


	bool isReady;

	/**
	 * @see setFutureAvailable
	 */
	bool futureAvailable;

	/**
	 * The actual value we are interested on.
	 */
	Type value;
};


/**
 * This is a specialization of Connector that is used to store inputs that have
 * to be specified by the user throgh the command line.
 */
template<typename Type> class InputConnector : public Connector<Type> {

public:

	void putValue(Type x) override {
		value = x;
	}

	Type fetchValue() override {
		return value;
	}

	bool checkConnection() override {
		return true;
	}

private:

	Type value;
};


} //namespace pinspect


#endif //RESULT_CONNECTOR_HPP
