#include "CragSolverFactory.h"
#include <util/ProgramOptions.h>

util::ProgramOption optionAssignmentSolver(
		util::_long_name        = "assignmentSolver",
		util::_description_text = "Use the assignment solver to get a solution. This is for CRAGs that model an "
		                          "assignment problem.");

util::ProgramOption optionClosedSetSolver(
		util::_long_name        = "closedSetSolver",
		util::_description_text = "Use the closed set solver to get a solution.");

util::ProgramOption optionDirectedMC(
		util::_long_name        = "directedMulticut",
		util::_description_text = "Use the directed multicut solver to get a solution.");

CragSolver*
CragSolverFactory::createSolver(
		const Crag& crag,
		const CragVolumes& volumes,
		CragSolver::Parameters parameters) {

	if (optionAssignmentSolver) {

		return new AssignmentSolver(crag, volumes, parameters);

	} else if (optionClosedSetSolver) {

		return new ClosedSetSolver(crag, parameters);

	} else if (optionDirectedMC) {
		auto solver = new DirectedMultiCutSolver(crag, parameters);
		solver->initializeILP();
		return solver;

	} else {
		auto solver = new MultiCutSolver(crag, parameters);
		solver->initializeILP();
		return solver;
	}
}
