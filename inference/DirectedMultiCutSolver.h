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

private:

	void setSpanningTreeVariables();
	void setSpanningTreeConstraints();
};

#endif // CANDIDATE_MC_SOLVER_DIRECTED_MULTI_CUT_H__

