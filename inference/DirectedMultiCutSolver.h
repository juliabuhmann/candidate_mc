#ifndef CANDIDATE_MC_SOLVER_DIRECTED_MULTI_CUT_H__
#define CANDIDATE_MC_SOLVER_DIRECTED_MULTI_CUT_H__

#include <crag/Crag.h>
#include <crag/CragVolumes.h>
#include <solver/LinearSolverBackend.h>
#include <vigra/tinyvector.hxx>
#include "Costs.h"
#include "MultiCutSolver.h"

class DirectedMultiCutSolver : public MultiCutSolver {

public:

	DirectedMultiCutSolver(const Crag& crag, const Parameters& parameters = Parameters());
	void initializeILP();
	MultiCutSolver::Status solve(CragSolution& solution);
private:

	void setSpanningTreeVariables();
	void setSpanningTreeConstraints();
	void prepareSolver();

	// First item of tuple is edge with direction i-->j, and second item is direction j-->i with i<j.
	std::map<int, std::pair<unsigned int, unsigned int>> _edgeIdToDirVarMap;
	std::map<int, unsigned int> _nodeIdToDirVarMap;
	std::map<int, int> _spanningTreeNodeToParNode;
};

#endif // CANDIDATE_MC_SOLVER_DIRECTED_MULTI_CUT_H__

