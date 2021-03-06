/**
 * merge_tree
 *
 * Given a source image or volume, creates initial supervoxels and iteratively 
 * merges them.
 */

#include <iostream>
#include <fstream>
#include <util/ProgramOptions.h>
#include <util/Logger.h>
#include <util/exceptions.h>
#include <util/helpers.hpp>
#include <vigra/impex.hxx>
#include <vigra/multi_array.hxx>
#include <vigra/functorexpression.hxx>
#include <vigra/slic.hxx>
#include <vigra/multi_convolution.hxx>
#include <mergetree/IterativeRegionMerging.h>
#include <mergetree/MedianEdgeIntensity.h>
#include <mergetree/SmallFirst.h>
#include <mergetree/MultiplyMinRegionSize.h>
#include <mergetree/MultiplySizeDifference.h>
#include <mergetree/RandomPerturbation.h>
#include <io/volumes.h>

util::ProgramOption optionSource(
		util::_long_name        = "source",
		util::_short_name       = "s",
		util::_description_text = "An image or directory of images to compute the merge tree for.");

util::ProgramOption optionInitialSuperpixels(
		util::_long_name        = "initialSuperpixels",
		util::_description_text = "Use the given image/volume as inital superpixels. If not given, superpixels will be extracted automatically.");

util::ProgramOption optionSuperpixelImage(
		util::_long_name        = "superpixelImage",
		util::_description_text = "Create an image with the initial superpixels.");

util::ProgramOption optionReportNextSuperpixelId(
		util::_long_name        = "reportNextSuperpixelId",
		util::_description_text = "Report the largest id + 1 after merging.");

util::ProgramOption optionSuperpixelsFirstId(
		util::_long_name        = "superpixelFirstId",
		util::_description_text = "Set the first id to be used for the superpixels created with superpixelImage or superpixelWithBordersImage.",
		util::_default_value    = 0);

util::ProgramOption optionRagFile(
		util::_long_name        = "ragFile",
		util::_description_text = "A file to write the region adjacency graph for the initial superpixels.");

util::ProgramOption optionMergeHistory(
		util::_long_name        = "mergeHistory",
		util::_description_text = "A file to write the region adjacency graph and merge history after merging.",
		util::_default_value    = "merge_history.txt");

util::ProgramOption optionSmooth(
		util::_long_name        = "smooth",
		util::_description_text = "Smooth the input image with a Gaussian kernel of the given stddev.");

util::ProgramOption optionSlicSuperpixels(
		util::_long_name        = "slicSuperpixels",
		util::_description_text = "Use SLIC superpixels instead of watersheds to obtain initial regions.");

util::ProgramOption optionSlicIntensityScaling(
		util::_long_name        = "slicIntensityScaling",
		util::_description_text = "How to scale the image intensity for comparison to spatial distance. Default is 1.0.",
		util::_default_value    = 1.0);

util::ProgramOption optionSliceSize(
		util::_long_name        = "slicSize",
		util::_description_text = "An upper limit on the SLIC superpixel size. Default is 10.",
		util::_default_value    = 10);

util::ProgramOption optionMergeSmallRegionsFirst(
		util::_long_name        = "mergeSmallRegionsFirst",
		util::_description_text = "Merge small regions first. For parameters, see smallRegionThreshold1, smallRegionThreshold2, and intensityThreshold.");

util::ProgramOption optionMultiplySizeDifference(
		util::_long_name        = "multiplySizeDifference",
		util::_description_text = "Multiply the edge scores with the difference of size of the involved regions.");

util::ProgramOption optionRandomPerturbation(
		util::_long_name        = "randomPerturbation",
		util::_short_name       = "r",
		util::_description_text = "Randomly (normally distributed) perturb the edge scores for merging.");

util::ProgramOption optionDontConsiderRegionSize(
		util::_long_name        = "dontConsiderRegionSize",
		util::_description_text = "By default, the scores are multiplied with the region size to encourage merging of small regions first. This option disables that.");

using namespace logger;

int main(int optionc, char** optionv) {

	using namespace vigra::functor;

	try {

		/********
		 * INIT *
		 ********/

		// init command line parser
		util::ProgramOptions::init(optionc, optionv);

		// init logger
		logger::LogManager::init();

		// read image
		ExplicitVolume<float> source = readVolume<float>(getImageFiles(optionSource.as<std::string>()));

		// smooth
		if (optionSmooth) {

			if (source.depth() > 1) {
				vigra::gaussianSmoothMultiArray(
						source.data(),
						source.data(),
						optionSmooth.as<double>());
			} else {
				vigra::gaussianSmoothMultiArray(
						source.data().bind<2>(0),
						source.data().bind<2>(0),
						optionSmooth.as<double>());
			}
		}

		// generate seeds
		ExplicitVolume<int> initialRegions(source.data().shape()[0], source.data().shape()[1], source.data().shape()[2]);
		if (optionInitialSuperpixels) {

			initialRegions = readVolume<int>(getImageFiles(optionInitialSuperpixels.as<std::string>()));

		} else {

			if (source.depth() > 1)
				vigra::generateWatershedSeeds(
						source.data(),
						initialRegions.data(),
						vigra::IndirectNeighborhood,
						vigra::SeedOptions().extendedMinima());
			else
				vigra::generateWatershedSeeds(
						source.data().bind<2>(0),
						initialRegions.data().bind<2>(0),
						vigra::IndirectNeighborhood,
						vigra::SeedOptions().extendedMinima());

			// perform watersheds or find SLIC superpixels

			if (optionSlicSuperpixels) {

				unsigned int maxLabel;
				if (source.depth() > 1)
						maxLabel = vigra::slicSuperpixels(
								source.data(),
								initialRegions.data(),
								optionSlicIntensityScaling.as<double>(),
								optionSliceSize.as<double>());
				else
						maxLabel = vigra::slicSuperpixels(
								source.data().bind<2>(0),
								initialRegions.data().bind<2>(0),
								optionSlicIntensityScaling.as<double>(),
								optionSliceSize.as<double>());

				LOG_USER(logger::out) << "found " << maxLabel << " SLIC superpixels" << std::endl;

			} else {

				unsigned int maxLabel;
				if (source.depth() > 1)
					maxLabel = vigra::watershedsMultiArray(
							source.data(), /* non-median filtered, possibly smoothed */
							initialRegions.data(),
							vigra::IndirectNeighborhood);
				else
					maxLabel = vigra::watershedsMultiArray(
							source.data().bind<2>(0), /* non-median filtered, possibly smoothed */
							initialRegions.data().bind<2>(0),
							vigra::IndirectNeighborhood);

				LOG_USER(logger::out) << "found " << maxLabel << " watershed regions" << std::endl;

				if (optionReportNextSuperpixelId) {

					// report highest used superpixel id
					LOG_USER(logger::out) << "next superpixel id: ";
					std::cout << (maxLabel + optionSuperpixelsFirstId.as<int>()) << std::endl;
				}
			}

			if (optionSuperpixelImage) {

				ExplicitVolume<int> initialRegionsExport(source.data().shape()[0], source.data().shape()[1], source.data().shape()[2]);
				vigra::transformMultiArray(
						initialRegions.data(),
						initialRegionsExport.data(),
						// vigra starts counting sp with 1
						Arg1() + Param(optionSuperpixelsFirstId.as<int>()));

				if (optionSuperpixelImage)
					saveVolume(
							initialRegionsExport,
							optionSuperpixelImage.as<std::string>());
			}
		}

		// extract merge tree
		IterativeRegionMerging<3> merging(initialRegions.data());

		MedianEdgeIntensity<3> mei(source.data());

		// create the RAG description for the median edge intensities
		if (optionRagFile)
			merging.storeRag(optionRagFile.as<std::string>(), mei);

		if (optionMergeSmallRegionsFirst) {

			SmallFirst<MedianEdgeIntensity<3>> scoringFunction(
					merging.getRag(),
					source.data(),
					initialRegions.data(),
					mei);

			if (optionRandomPerturbation) {

				RandomPerturbation<SmallFirst<MedianEdgeIntensity<3>> > rp(scoringFunction);
				merging.createMergeTree(rp);

			} else {

				merging.createMergeTree(scoringFunction);
			}

		} else if (optionMultiplySizeDifference) {

			MultiplySizeDifference<MedianEdgeIntensity<3>> scoringFunction(
					merging.getRag(),
					initialRegions.data(),
					mei);

			if (optionRandomPerturbation) {

				RandomPerturbation<MultiplySizeDifference<MedianEdgeIntensity<3>> > rp(scoringFunction);
				merging.createMergeTree(rp);

			} else {

				merging.createMergeTree(scoringFunction);
			}

		} else {

			if (optionDontConsiderRegionSize) {

				if (optionRandomPerturbation) {

					RandomPerturbation<MedianEdgeIntensity<3>> rp(mei);
					merging.createMergeTree(rp);

				} else {

					merging.createMergeTree(mei);
				}

			} else {

				MultiplyMinRegionSize<MedianEdgeIntensity<3>> scoringFunction(
						merging.getRag(),
						initialRegions.data(),
						mei);

				if (optionRandomPerturbation) {

					RandomPerturbation<MultiplyMinRegionSize<MedianEdgeIntensity<3>>> rp(scoringFunction);
					merging.createMergeTree(rp);

				} else {

					merging.createMergeTree(scoringFunction);
				}
			}
		}

		LOG_USER(logger::out) << "writing merge history..." << std::endl;

		merging.storeMergeHistory(optionMergeHistory.as<std::string>());

	} catch (Exception& e) {

		handleException(e, std::cerr);
	}
}

