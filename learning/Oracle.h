#ifndef CANDIDATE_MC_LEARNING_ORACLE_H__
#define CANDIDATE_MC_LEARNING_ORACLE_H__

#include <crag/Crag.h>
#include <inference/CragSolverFactory.h>
#include <features/NodeFeatures.h>
#include <features/EdgeFeatures.h>
#include <util/assert.h>
#include "Loss.h"
#include "BestEffort.h"

/**
 * Provides solution for loss-augmented inference problem, given a set of 
 * weights. To be used in a learning optimizer.
 */
class Oracle {

public:

	Oracle(
			const Crag&             crag,
			const CragVolumes&      volumes,
			const NodeFeatures&     nodeFeatures,
			const EdgeFeatures&     edgeFeatures,
			const Loss&             loss,
			const BestEffort&       bestEffort,
			CragSolver::Parameters  parameters = CragSolver::Parameters()) :
		_crag(crag),
		_volumes(volumes),
		_nodeFeatures(nodeFeatures),
		_edgeFeatures(edgeFeatures),
		_loss(loss),
		_bestEffort(bestEffort),
		_costs(_crag),
		_mostViolatedSolver(CragSolverFactory::createSolver(crag, volumes, parameters)),
		_currentBestSolver(CragSolverFactory::createSolver(crag, volumes, parameters)),
		_iteration(0) {}

	void operator()(
			const FeatureWeights& weights,
			double&               value,
			FeatureWeights&       gradient);

private:

	void updateCosts(const FeatureWeights& weights);

	void accumulateGradient(FeatureWeights& gradient);

	inline double nodeCost(Crag::CragNode n, const FeatureWeights& weights) const {

		return dot(weights[_crag.type(n)], _nodeFeatures[n]);
	}

	inline double edgeCost(Crag::CragEdge e, const FeatureWeights& weights) const {

		return dot(weights[_crag.type(e)], _edgeFeatures[e]);
	}

	inline double dot(const std::vector<double>& a, const std::vector<double>& b) const {

		UTIL_ASSERT_REL(a.size(), ==, b.size());

		double sum = 0;
		auto ba = a.begin();
		auto ea = a.end();
		auto bb = b.begin();

		while (ba != ea) {

			sum += (*ba)*(*bb);
			ba++;
			bb++;
		}

		return sum;
	}

	const Crag&         _crag;
	const CragVolumes&  _volumes;
	const NodeFeatures& _nodeFeatures;
	const EdgeFeatures& _edgeFeatures;
	const Loss&         _loss;
	const BestEffort&   _bestEffort;

	Costs _costs;

	// constant to be added to the optimal value of the multi-cut solution
	double _constant;

	// best-effort part of _constant
	double _B_c;

	std::unique_ptr<CragSolver> _mostViolatedSolver;
	std::unique_ptr<CragSolver> _currentBestSolver;

	int _iteration;
};

#endif // CANDIDATE_MC_LEARNING_ORACLE_H__

