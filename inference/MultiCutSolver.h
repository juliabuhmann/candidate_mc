#ifndef CANDIDATE_MC_SOLVER_MULTI_CUT_H__
#define CANDIDATE_MC_SOLVER_MULTI_CUT_H__

#include <crag/Crag.h>
#include <crag/CragVolumes.h>
#include <solver/LinearSolverBackend.h>
#include <vigra/tinyvector.hxx>
#include "Costs.h"
#include "CragSolver.h"

class MultiCutSolver : public CragSolver {
	friend class DirectedMultiCutSolver;

public:

	MultiCutSolver(const Crag& crag, const Parameters& parameters = Parameters());

	~MultiCutSolver();

	/**
	 * Set the costs (or reward, if negative) of accepting a node or an edge.
	 */
	void setCosts(const Costs& costs) override;

	Status solve(CragSolution& solution) override;

	/**
	 * Get the value of the current solution.
	 */
	double getValue() override { return _solution.getValue(); }

private:

	// a property map returning 1 for every entry
	struct One {

		typedef int Value;

		template <typename T>
		int operator[](const T&) const { return 1; }

	};

	void prepareSolver();

	void setVariables();

	void setInitialConstraints();

	int collectTreePathConstraints(Crag::CragNode n, std::vector<int>& pathIds);

	void findCut(CragSolution& solution);

	bool findViolatedConstraints(CragSolution& solution);

	void propagateLabel(Crag::CragNode n, int label);

	inline unsigned int nodeIdToVar(int nodeId) { return nodeId; }
	inline unsigned int edgeIdToVar(int edgeId) { return _edgeIdToVarMap[edgeId]; }

	const Crag& _crag;

	unsigned int _numNodes, _numEdges;

	std::map<int, unsigned int> _edgeIdToVarMap;

	LinearObjective      _objective;
	LinearConstraints    _constraints;
	LinearSolverBackend* _solver;
	Solution             _solution;

	Parameters _parameters;

    std::vector<LinearConstraint> _allTreePathConstraints;

	int _numPositiveCostPinConstraints;

	Crag::NodeMap<int> _labels;
};

#endif // CANDIDATE_MC_SOLVER_MULTI_CUT_H__

