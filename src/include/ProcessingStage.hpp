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

#ifndef PROCESSING_STAGE_HPP
#define PROCESSING_STAGE_HPP

#include <boost/fusion/tuple.hpp>
#include <boost/fusion/algorithm/transformation/transform.hpp>
#include <boost/fusion/algorithm/query/all.hpp>
#include <boost/fusion/functional/invocation/invoke.hpp>
#include <chrono>

#include "CalibrationSet.hpp"
#include "Parameters.hpp"
#include "ResultConnector.hpp"


namespace pinspect {


/**
 * Abstract class that generalize @see ProcessingStage which is a template.
 */
class ProcessingInterface {

public:

	virtual std::string name() = 0;

	virtual void doProcess() = 0;

	virtual bool checkInputs() = 0;

	virtual ~ProcessingInterface() {}

	static void setOutputDirectory(std::string name) { outputDirectory = name; }

	static void setParameters(Parameters p) {	parameters = p;	}

protected:

	/**
	 * Store the name of the directory to let all stages serialize and recover
	 * their input and outputs from disk.
	 */
	static std::string outputDirectory;

	/**
	 * Parameter structure accessible by all stages.
	 */
	static Parameters parameters;
};


/**
 * Template class that represents a single stage in a processing pipeline.
 * It is parameterized in input parameter through variadic template
 * parameters @see ArgT, and output with @see RetT.
 *
 * This permit a very easy way to compose a pipeline out of single
 * processing stages, all specializing this class. And gives type safety of the
 * input/output objects.
 *
 * Behavior:
 * 		When 'this' stage has to be performed @see doProcess, an initial operation
 * 		of retrieving	is done	through the use of @see Connector objects.
 * 		That can assure that all previous stages of the pipeline were executed.
 * 		After, a call to the private @see process function let the proper happen.
 * 		And finally all results from @see process function are stored.
 */
template<typename RetT, typename... ArgT> class ProcessingStage : public ProcessingInterface {


	/**
	 * Functor that serves to extract the value contained in the
	 * @see ResultConnector provided as argument. It's used in @see doProcess.
	 */
	struct ExtractFromConnector {
		template<typename T> T operator()(Connector<T>* ref) const {
			return ref->fetchValue();
		}
	};

	/**
	 * Callable object to make possible calling the private
	 * function @see process, which is specialized for every overridden class.
	 * It's to be initialized passing a pointer to an instance of this class.
	 * It's used in @see doProcess.
	 */
	struct ProcessOnThis {
		ProcessOnThis(ProcessingStage *ptr) : instancePtr(ptr) {}

		RetT operator()(ArgT...args) { return instancePtr->process(args...); }

		ProcessingStage *instancePtr;
	};

	/**
	 * Functor to check for input availability.
	 */
	struct InputChecker {
		template<typename T> bool operator()(Connector<T>* ref) const {
			return ref->checkConnection();
		}
	};


public:

	ProcessingStage(std::string stageName) : ProcessingInterface(),
		outputValue(outputDirectory, stageName), stageName(stageName) {}


	/**
	 * Return the actual name for this stage.
	 */
	std::string name() override {
		return stageName;
	}

	/**
	 * Collect all data needed from any previous stage and pass it to the actual
	 * @see process function, which has to be overridden by stage classes.
	 *
	 * Compute the running time for this stage using std::chrono::steady_clock
	 */
	void doProcess() override {
		//acquire input arguments
		boost::fusion::tuple<ArgT...> args;
		args = boost::fusion::transform(inputValues, ExtractFromConnector());

		auto start = std::chrono::steady_clock::now();

		//actual call to process()
		RetT ret = boost::fusion::invoke(ProcessOnThis(this), args);

	  auto end = std::chrono::steady_clock::now();
	  auto diff = end - start;
	  std::cout << "(" << std::chrono::duration <double, std::milli>(diff).count()
	  		<< " ms)" << std::endl;

		//store(give) arguments for another stage
		outputValue.putValue(ret);
	}

	/**
	 * Set up connection with the output of other stages. This way we have
	 * information to collect before the actual processing.
	 *
	 * Note the arguments type, it has to be coherent with every single input
	 * data type.
	 */
	void setInput(Connector<ArgT>*...values) {
		inputValues = boost::fusion::make_tuple(values...);
	}

	/**
	 * Run the check on all inputs Connector.
	 */
	bool checkInputs() override {

		auto vec = boost::fusion::transform(inputValues, InputChecker());
		bool ok = boost::fusion::all(vec, [](bool b){ return b == true; });

		//then we set future availability
		if(ok)
			outputValue.setFutureAvailable();
		return ok;
	}

	/**
	 * @return the connector that can be used to fetch the output value of
	 * this stage.
	 */
	Connector<RetT>* getResultConnector() {
		return &outputValue;
	}


	virtual ~ProcessingStage() {}


private:

	/**
	 * This member function must be overridden from deriving classes.
	 * So you can specialize the stage for your desired processing.
	 *
	 * Note the arguments type, this have to be coherent with parameters used to
	 * specialize this template class.
	 */
	virtual RetT process(ArgT...) = 0;


	/**
	 * Hold pointers to connectors of other(previous) stages. This information
	 * is what we need to recall all inputs before performing the actual computation.
	 */
	boost::fusion::tuple<Connector<ArgT>*...> inputValues;

	/**
	 * Where we store results of this processing stage.
	 */
	ResultConnector<RetT> outputValue;

	std::string stageName;

};


} //namespace pinspect


#endif // PROCESSING_STAGE_HPP
