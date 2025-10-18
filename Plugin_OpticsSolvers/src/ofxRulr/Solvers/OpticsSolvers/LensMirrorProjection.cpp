#include "pch_Plugin_OpticsSolvers.h"
#include "LensMirrorProjection.h"

using namespace ofxRulr::Models::OpticsSolvers;
using namespace ofxCeres::Models;

struct LensMirrorProjectionCost {
	//-----------
	LensMirrorProjectionCost(const Array2D<Ray<float>>& gridRays
		, const Array2D<Ray<float>>& incomingRays
		, const Array2D<glm::vec3>& targetPositions)
		: gridRays(gridRays)
		, incomingRays(incomingRays)
		, targetPositions(targetPositions)
	{
	}

	//----------
	template<typename T>
	bool
		operator()(T const* const* parameters, T* residuals)
	{
		auto width = this->gridRays.width();
		auto height = this->gridRays.height();

		// Build the distances Array2D
		Array2D<T> distances;
		distances.allocate(width, height);
		distances.set(parameters[0]);

		// Build the LensMirror object
		LensMirror<T> lensMirror;
		lensMirror.allocate(width, height);
		lensMirror.gridRays = (Array2D<Ray<T>>) this->gridRays;
		lensMirror.setDistances(distances);

		// Build the incomingRays
		auto incomingRays = (Array2D<Ray<T>>) this->incomingRays;

		// Perform reflections
		Array2D<Ray<T>> reflectedRays;
		lensMirror.reflect(incomingRays, reflectedRays);

		// Get the residuals
		{
			auto residualMover = residuals;
			for (auto it : reflectedRays) {
				// Ignore edges
				{
					auto i = it.i();
					auto j = it.j();
					if (i == 0 || j == 0
						|| i == width - 1 || j == height - 1) {
						*residualMover++ = (T) 0.0;
						*residualMover++ = (T)0.0;
						*residualMover++ = (T)0.0;
						continue;
					}
				}

				const auto& reflectedRay = it.value();
				const auto& targetPositionFloat = this->targetPositions[it];
				const auto targetPosition = (glm::tvec3<T>) targetPositionFloat;
				const auto correctTransmissionTowardsTarget = glm::normalize(targetPosition - reflectedRay.s);
				const auto delta = reflectedRay.t - correctTransmissionTowardsTarget;
				
				*residualMover++ = delta[0];
				*residualMover++ = delta[1];
				*residualMover++ = delta[2];
			}
		}

		return true;
	}

	static ceres::CostFunction*
		Create(const Array2D<Ray<float>>& gridRays
			, const Array2D<Ray<float>>& incomingRays
			, const Array2D<glm::vec3>& targetPositions)
	{
		auto costFunction = new ceres::DynamicAutoDiffCostFunction<LensMirrorProjectionCost>(
			new LensMirrorProjectionCost(gridRays, incomingRays, targetPositions)
		);

		// Set size of parameters and residuals
		{
			costFunction->AddParameterBlock(gridRays.size);
			costFunction->SetNumResiduals(3 * gridRays.size);
		}

		return costFunction;
	}

	const Array2D<Ray<float>> gridRays;
	const Array2D<Ray<float>> incomingRays;
	const Array2D<glm::vec3> targetPositions;
};

struct LensMirrorMidDistanceCost {
	//-----------
	LensMirrorMidDistanceCost(float midDistance
		, size_t distanceCount)
		: midDistance(midDistance)
		, distanceCount(distanceCount)
	{
	}

	//----------
	template<typename T>
	bool
		operator()(T const* const* parameters, T* residuals)
	{
		for (size_t i = 0; i < this->distanceCount; i++) {
			residuals[i] = parameters[0][i] - (T) this->midDistance;
		}

		return true;
	}

	static ceres::CostFunction*
		Create(float midDistance
			, size_t distanceCount)
	{
		auto costFunction = new ceres::DynamicAutoDiffCostFunction<LensMirrorMidDistanceCost>(
			new LensMirrorMidDistanceCost(midDistance, distanceCount)
		);

		// Set size of parameters and residuals
		{
			costFunction->AddParameterBlock(distanceCount);
			costFunction->SetNumResiduals(distanceCount);
		}

		return costFunction;
	}

	const float midDistance;
	const size_t distanceCount;
};

namespace ofxRulr {
	namespace Solvers {
		namespace OpticsSolvers {
			//----------
			ofxCeres::SolverSettings
				LensMirrorProjection::getDefaultSolverSettings()
			{
				ofxCeres::SolverSettings solverSettings;
				{
					solverSettings.options.max_num_iterations = 10000;
					solverSettings.options.function_tolerance = 1e-10;
					solverSettings.options.parameter_tolerance = 1e-10;
				}
				return solverSettings;
			}

			//----------
			LensMirrorProjection::Result
				LensMirrorProjection::solve(const LensMirror<float>& initialLensMirror
					, const Array2D<Ray<float>>& incomingRays
					, const Array2D<glm::vec3>& targetPositions
					, float midDistance
					, const ofxCeres::SolverSettings& solverSettings)
			{
				const auto width = incomingRays.width();
				const auto height = incomingRays.height();
				const auto size = incomingRays.size;

				// Parameters
				vector<double> parameters;
				{
					parameters.resize(size);

					auto distances = initialLensMirror.distances;
					for (auto it : distances) {
						const auto& distance = it.value();
						parameters[it.idx()] = (double) distance;
					}
				}

				// Build the problem
				ceres::Problem problem;
				problem.AddResidualBlock(LensMirrorProjectionCost::Create(initialLensMirror.gridRays
					, incomingRays
					, targetPositions)
					, NULL
					, parameters.data());
				problem.AddResidualBlock(LensMirrorMidDistanceCost::Create(midDistance, parameters.size())
					, NULL
					, parameters.data());
				
				// Perform solve
				ceres::Solver::Summary summary;
				{
					ceres::Solve(solverSettings.options
						, &problem
						, &summary);

					if (solverSettings.printReport) {
						cout << summary.FullReport() << endl;
					}
				}

				Result result(summary);

				// Read parameters back into solution
				{
					Array2D<float> distances;
					distances.allocate(width, height);

					for (auto it : distances) {
						it.value() = parameters[it.idx()];
					}

					result.solution.lensMirror.gridRays = initialLensMirror.gridRays;
					result.solution.lensMirror.setDistances(distances);
				}

				return result;
			}
		}
	}
}