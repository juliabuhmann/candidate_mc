#ifndef CANDIDATE_MC_CRAG_BORDER_ANNOTATOR_H__
#define CANDIDATE_MC_CRAG_BORDER_ANNOTATOR_H__

#include "Crag.h"
#include "CragVolumes.h"

/**
 * A class that checks for each node of the crag whether the corresponding region touches the cube of the boundary.
 */
class BorderAnnotator {

public:

	BorderAnnotator() {
		_numBorder = 0;
		_numInside = 0;
	};

	void annotate(Crag& crag, const CragVolumes& volumes);

//protected:
//
//	/**
//	 * Propagate adjacency of leaf candidates in a straight forward manner to
//	 * super-candidates: Candidates are adjacent, if any of their sub-candidates
//	 * are adjacent.
//	 */
//	void propagateLeafAdjacencies(Crag& crag);

private:
//
//	/**
//	 * Find propagated edges for node n and below. Returns the set of
//	 * descendants of n (including n).
//	 */
//	std::set<Crag::CragNode> recurseAdjacencies(Crag& crag, Crag::CragNode n);
//
//	/**
//	 * For binary trees, remove adjacency edges between children, since the
//	 * merge represented by those is already performed by selecting the parent
//	 * node.
//	 */
//	void pruneChildEdges(Crag& crag);
//
//	/**
//	 * Is the given edge connecting children of the same node?
//	 */
//	bool isSiblingEdge(Crag& crag, Crag::CragEdge e);
//
	unsigned int _numBorder;
	unsigned int _numInside;
};

#endif // CANDIDATE_MC_CRAG_PLANAR_ADJACENCY_ANNOTATOR_H__

