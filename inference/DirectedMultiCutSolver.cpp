#include <boost/filesystem.hpp>
#include <lemon/dijkstra.h>
#include <lemon/connectivity.h>
#include <solver/SolverFactory.h>
#include <util/Logger.h>
#include <util/ProgramOptions.h>
#include <util/box.hpp>
#include "DirectedMultiCutSolver.h"

logger::LogChannel directedmulticutlog("directedmulticutlog",
		"[DirectedMultiCutSolver] ");

DirectedMultiCutSolver::DirectedMultiCutSolver(const Crag& crag,
		const Parameters& parameters) :
		MultiCutSolver(crag, parameters) {
	setSpanningTreeVariables();
	setSpanningTreeConstraints();
}

void DirectedMultiCutSolver::setSpanningTreeVariables() {

	LOG_USER(directedmulticutlog) << "setting spanning tree variables..."
			<< std::endl;

	// for each edge, add two variables representing directed edges.

	// for each node that touches the cube border, add an additional variable.

}

void DirectedMultiCutSolver::setSpanningTreeConstraints() {

	LOG_USER(directedmulticutlog)
			<< "setting initial constraints required for spanning tree"
			<< std::endl;

	// For each edge, allow only one directed edge to be valid.

}
