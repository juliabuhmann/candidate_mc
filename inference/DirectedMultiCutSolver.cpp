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

}

void DirectedMultiCutSolver::initializeILP() {
	setVariables();
	setSpanningTreeVariables();
	prepareSolver();
	setSpanningTreeConstraints();
	if (!_parameters.noConstraints)
		setInitialConstraints();

}

void DirectedMultiCutSolver::prepareSolver() {
	LOG_DEBUG(directedmulticutlog) << "preparing solver..." << std::endl;

	int numBorderNodes = _nodeIdToDirVarMap.size();

	// one binary indicator per node and edge (multicut) + two indicators for
	// each edge and for each border node (directed multicut extension).
	_objective.resize(_numNodes + _numEdges * 3 + numBorderNodes);
	_objective.setSense(_parameters.minimize ? Minimize : Maximize);

	_solver->initialize(_numNodes + _numEdges * 3 + numBorderNodes, Binary);

}

void DirectedMultiCutSolver::setSpanningTreeVariables() {

	LOG_USER(directedmulticutlog) << "setting spanning tree variables..."
			<< std::endl;

	unsigned int nextVar = 0;
	// Get the last variable number.
	for (Crag::CragEdge e : _crag.edges()) {
		nextVar = _edgeIdToVarMap[_crag.id(e)];
	}
	++nextVar;

	// for each edge, add two variables representing directed edges.
	for (Crag::CragEdge e : _crag.edges()) {
		_edgeIdToDirVarMap[_crag.id(e)] = std::make_pair(nextVar, nextVar + 1);
		nextVar += 2;
	}

	// for each node that touches the cube border, add an additional variable.
	for (Crag::NodeIt n(_crag); n != lemon::INVALID; ++n) {
		if (!_crag.isLeafNode(n))
			continue;
		if (!_crag.isBorder(n))
			continue;

		_nodeIdToDirVarMap[_crag.id(n)] = nextVar;
		++nextVar;
	}

}

void DirectedMultiCutSolver::setSpanningTreeConstraints() {

	LOG_USER(directedmulticutlog)
			<< "setting initial constraints required for spanning tree"
			<< std::endl;

	// For each edge, allow only one directed edge to be valid.
	// ai-->j + aj-->i <= 1
	int numSingleDirectionConstraints = 0;
	for (Crag::CragEdge e : _crag.edges()) {
		LinearConstraint singleDirectionConstraint;

		std::pair<unsigned int, unsigned int> varPair =
				_edgeIdToDirVarMap[_crag.id(e)];

		singleDirectionConstraint.setCoefficient(varPair.first, 1.0);
		singleDirectionConstraint.setCoefficient(varPair.second, 1.0);

		singleDirectionConstraint.setRelation(LessEqual);
		singleDirectionConstraint.setValue(1.0);
		_constraints.add(singleDirectionConstraint);
		++numSingleDirectionConstraints;
	}

	LOG_USER(directedmulticutlog) << "added " << numSingleDirectionConstraints
			<< " one-direction-per-adjacency-edge constraint" << std::endl;

	int incomingEdgesConstraints = 0;
	// Each node must have exactly one incoming edge.
	// Sigma[a-->i]a = 1
	for (Crag::NodeIt n(_crag); n != lemon::INVALID; ++n) {

		if (!_crag.isLeafNode(n))
			continue;
		LinearConstraint singleIncomingEdge;

		if (_crag.isBorder(n)) {
			singleIncomingEdge.setCoefficient(_nodeIdToDirVarMap[_crag.id(n)],
					1.0);
		}

		// Get all adjacent leaf nodes.

		for (Crag::CragEdge e : _crag.adjEdges(n)) {
			Crag::Node oppositeNode = _crag.oppositeNode(n, e);
			if (!_crag.isLeafNode(oppositeNode))
				continue;

			std::pair<unsigned int, unsigned int> varPair =
					_edgeIdToDirVarMap[_crag.id(e)];
			// Choose the variable representing the incoming edge.
			// If current node i < j, choose sec, if i > j, choose first.
			if (_crag.id(n) > _crag.id(oppositeNode)) {
				singleIncomingEdge.setCoefficient(varPair.first, 1.0);
			} else {
				singleIncomingEdge.setCoefficient(varPair.second, 1.0);
			}

		}

		singleIncomingEdge.setRelation(Equal);
		singleIncomingEdge.setValue(1.0);
		_constraints.add(singleIncomingEdge);
		++incomingEdgesConstraints;
	}
	LOG_USER(directedmulticutlog) << "added " << incomingEdgesConstraints
			<< " one-incoming-edge-per-node constraints" << std::endl;

	// Connect merge decision level to the directed spanning tree. If a spanning tree edge is
	// switched on, the two adjacent regions should be merged (part of the same segment).
	// e(i<-->j)+a(i-->j)+ a(j-->i) <= 1
	int numSpanningTreeEdgeToMergeEdge = 0;
	for (Crag::CragEdge e : _crag.edges()) {
		LinearConstraint spanningToMergeEdgeConstraint;

		std::pair<unsigned int, unsigned int> varPair =
				_edgeIdToDirVarMap[_crag.id(e)];

		spanningToMergeEdgeConstraint.setCoefficient(varPair.first, 1.0);
		spanningToMergeEdgeConstraint.setCoefficient(varPair.second, 1.0);
		spanningToMergeEdgeConstraint.setCoefficient(
				_edgeIdToVarMap[_crag.id(e)], -1.0);

		spanningToMergeEdgeConstraint.setRelation(LessEqual);
		spanningToMergeEdgeConstraint.setValue(0.0);
		_constraints.add(spanningToMergeEdgeConstraint);
		++numSpanningTreeEdgeToMergeEdge;
	}

	LOG_USER(directedmulticutlog) << "added " << numSpanningTreeEdgeToMergeEdge
			<< " if spanning tree edge is on, the two adjacent regions have to be merged."
			<< std::endl;
}

MultiCutSolver::Status DirectedMultiCutSolver::solve(CragSolution& solution) {

	_solver->setObjective(_objective);

	for (unsigned int i = 0; i < _parameters.numIterations; i++) {

		LOG_USER(directedmulticutlog) << "------------------------ iteration "
				<< i << std::endl;

		findCut(solution);

		// extract spanning tree.
		int numSelectedSpanningTreeEdges = 0;
		for (Crag::CragEdge e : _crag.edges()) {
			int nodeU = _crag.id(e.u());
			int nodeV = _crag.id(e.v());
			std::pair<unsigned int, unsigned int> varPair =
					_edgeIdToDirVarMap[_crag.id(e)];
			int solutionU = _solution[varPair.first];
			int solutionV = _solution[varPair.second];
			LOG_ALL(directedmulticutlog) << "v: " << varPair.first << " s: "
					<< solutionU << " merge: " << solution.selected(e)
					<< " id: " << _crag.id(e) << std::endl;
			LOG_ALL(directedmulticutlog) << "v: " << varPair.second << " s: "
					<< solutionV << " merge: " << solution.selected(e)
					<< " id: " << _crag.id(e) << std::endl;
			// Not all edges are part of the spanning tree.
			if (solutionU == 1) {
				// This means, direction goes from small node to big node.
				if (nodeU < nodeV) {
					_spanningTreeNodeToParNode[nodeV] = nodeU;
				} else {
					_spanningTreeNodeToParNode[nodeU] = nodeV;
				}
				++numSelectedSpanningTreeEdges;
			}

			if (solutionV == 1) {
				// This means, direction goes from big node to small node.
				if (nodeU < nodeV) {
					_spanningTreeNodeToParNode[nodeU] = nodeV;
				} else {
					_spanningTreeNodeToParNode[nodeV] = nodeU;
				}
				++numSelectedSpanningTreeEdges;

			}

		}

		LOG_ALL(directedmulticutlog) << "number of total spanning edges: "
				<< numSelectedSpanningTreeEdges << std::endl;

		if (!findViolatedConstraints(solution)) {

			LOG_USER(directedmulticutlog) << "optimal solution with value "
					<< _solution.getValue() << " found" << std::endl;

			int numSelected = 0;
			int numMerged = 0;
			double avgDepth = 0;

			for (Crag::CragNode n : _crag.nodes())
				if (solution.selected(n)) {

					numSelected++;
					avgDepth += _crag.getLevel(n);
				}

			avgDepth /= numSelected;

			for (Crag::CragEdge e : _crag.edges())
				if (solution.selected(e))
					numMerged++;

			LOG_USER(directedmulticutlog) << numSelected
					<< " candidates selected, " << numMerged
					<< " adjacent candidates merged" << std::endl;
			LOG_USER(directedmulticutlog)
					<< "average depth of selected candidates is " << avgDepth
					<< std::endl;

			int numRootNodeConnection = 0;
			for (Crag::NodeIt n(_crag); n != lemon::INVALID; ++n) {
				if (_crag.isBorder(n)) {
					LOG_ALL(directedmulticutlog) << "v: "
							<< _nodeIdToDirVarMap[_crag.id(n)] << " s: "
							<< _solution[_nodeIdToDirVarMap[_crag.id(n)]]
							<< std::endl;
					if (_solution[_nodeIdToDirVarMap[_crag.id(n)]] > 0.5) {
						_spanningTreeNodeToParNode[_crag.id(n)] = -1;
						++numRootNodeConnection;
					}
				}

			}

			LOG_USER(directedmulticutlog)
					<< "regions connected to the root node: "
					<< numRootNodeConnection << std::endl;

			for (const auto &p : _spanningTreeNodeToParNode)
				LOG_ALL(directedmulticutlog) << "child: " << p.first
						<< " parent: " << p.second << std::endl;

			return SolutionFound;
		}
	}

	LOG_USER(directedmulticutlog) << "maximum number of iterations reached"
			<< std::endl;
	return MaxIterationsReached;
}
