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

#include "Pipeline.hpp"


namespace pinspect {


void Pipeline::execute() {
	for (auto s : processingList)
		s->doProcess();
}


void Pipeline::checkStagesInput() {
	for (auto s : processingList)
		if (!s->checkInputs()) {
			std::cout << "Not all inputs for stage '" + s->name() + "' are available"
					<< std::endl;
			std::cout << "Maybe you have to execute previous stages" << std::endl;
			std::exit(1);
		}
}


Pipeline& Pipeline::addStage(ProcessingInterface* stage) {
	processingList.push_back(stage);
	return *this;
}


} //pinspect
