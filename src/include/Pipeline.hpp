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

#ifndef PIPELINE_HPP
#define PIPELINE_HPP

#include <list>
#include "ProcessingStage.hpp"


namespace pinspect {


/**
 * Represent the chain of computations. They are of type @see ProcessingInterface
 * which is polymorphic.
 */
class Pipeline {

public:

	/**
	 * Execute all stages, thus call @see doProcess for each.
	 */
	void execute();

	/**
	 * This serves to check if the pipeline it's built consistently. Every
	 * stage must have the input wired in the right way.
	 */
	void checkStagesInput();

	/**
	 * Simply add @parma stage to the current instance.
	 */
	Pipeline& addStage(ProcessingInterface* stage);


private:

	/**
	 * Pipeline representation as std::list.
	 */
	std::list<ProcessingInterface*> processingList;

};


} //pinspect


#endif //PIPELINE_HPP
