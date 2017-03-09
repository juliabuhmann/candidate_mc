#include <util/Logger.h>
#include <util/helpers.hpp>
#include <util/ProgramOptions.h>
#include "BorderAnnotator.h"

logger::LogChannel borderannotatorlog("borderannotatorlog",
		"[BorderAnnotator] ");

void BorderAnnotator::annotate(Crag& crag, const CragVolumes& volumes) {

//	if (optionCragType.as<std::string>() == "empty")
//		return;

//	UTIL_TIME_METHOD;

	util::point<float, 3> resolution;
	for (Crag::NodeIt n(crag); n != lemon::INVALID; ++n) {

		if (!crag.isLeafNode(n))
			continue;

		resolution = volumes[n]->getResolution();
		break;
	}

	// no nodes?
	if (resolution.isZero())
		return;

	util::box<float, 3> cragBB = volumes.getBoundingBox() / resolution;
	LOG_USER(borderannotatorlog) << "annotating border nodes." << std::endl;

	for (Crag::NodeIt n(crag); n != lemon::INVALID; ++n) {

		if (!crag.isLeafNode(n))
			continue;


		const util::point<float, 3>& volumeOffset = volumes[n]->getOffset()
				/ resolution;
		const util::box<unsigned int, 3>& volumeDiscreteBB =
				volumes[n]->getDiscreteBoundingBox();

		util::point<unsigned int, 3> begin = (volumeOffset - cragBB.min());

		util::point<unsigned int, 3> end = begin
				+ util::point<unsigned int, 3>(volumeDiscreteBB.width(),
						volumeDiscreteBB.height(), volumeDiscreteBB.depth());

		// Check if any of the bounding boxes of the volumes touch the cube.
		// Bottom.
		bool is_border = false;
		if (begin.x() == cragBB.min().x() || begin.y() == cragBB.min().y()
				|| begin.z() == cragBB.min().z()) {
			is_border = true;
		}
		if (end.x() == cragBB.max().x() || end.y() == cragBB.max().y()
				|| end.z() == cragBB.max().z()) {
			is_border = true;
		}

		if (is_border) {
			crag.setBorderNode(n, true);
			++_numBorder;
		}else {
			crag.setBorderNode(n, false);
			++_numInside;
		}

	}
	crag.setBorderValid(true);
	LOG_USER(borderannotatorlog) << _numBorder << " number of nodes touch the border and "
			  << _numInside << " nodes are fully contained." <<  std::endl;

}

