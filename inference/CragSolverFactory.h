#ifndef CANDIDATE_MC_INFERENCE_CRAG_SOLVER_FACTORY_H__
#define CANDIDATE_MC_INFERENCE_CRAG_SOLVER_FACTORY_H__

#include "AssignmentSolver.h"
#include "MultiCutSolver.h"

class CragSolverFactory {

public:

	static CragSolver* createSolver(
			const Crag& crag,
			const CragVolumes& volumes,
			CragSolver::Parameters parameters);
};

#endif // CANDIDATE_MC_INFERENCE_CRAG_SOLVER_FACTORY_H__

