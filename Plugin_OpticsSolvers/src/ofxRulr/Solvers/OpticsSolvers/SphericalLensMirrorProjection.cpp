#include "pch_Plugin_OpticsSolvers.h"
#include "SphericalLensMirrorProjection.h"

using namespace ofxRulr::Models::OpticsSolvers;
using namespace ofxCeres::Models;

struct SphericalLensMirrorProjectionCost {
	//-----------
	SphericalLensMirrorProjectionCost(const Array2D<Ray<float>>& gridRays
		, const Array2D<Ray<float>>& incomingRays
		, const Array2D<glm::vec3>& targetPositions
		, const bool isConvex)
		: gridRays(gridRays)
		, incomingRays(incomingRays)
		, targetPositions(targetPositions)
		, isConvex(isConvex)
	{
	}

	//----------
	template<typename T>
	bool
		operator()(T const* const* parameters, T* residuals)
	{
		auto width = this->gridRays.width();
		auto height = this->gridRays.height();

		// Build the spherical lens
		SphericalLensMirror<T> sphericalLensMirror;
		{
			sphericalLensMirror.gridRays = (Array2D<Ray<T>>) this->gridRays;
			sphericalLensMirror.shape = this->isConvex
				? SphericalMirrorShape::Convex
				: SphericalMirrorShape::Concave;
			sphericalLensMirror.sphere.center[0] = parameters[0][0];
			sphericalLensMirror.sphere.center[1] = parameters[0][1];
			sphericalLensMirror.sphere.center[2] = parameters[0][2];
			sphericalLensMirror.sphere.radius = parameters[0][3];
		}

		// Get it as a lensMirror
		auto lensMirror = sphericalLensMirror.getLensMirror();

		// Build the incomingRays
		auto incomingRays = (Array2D<Ray<T>>) this->incomingRays;

		// Perform reflections
		Array2D<Ray<T>> reflectedRays;
		lensMirror.reflect(incomingRays, reflectedRays);

		// Get the residuals
		{
			auto residualMover = residuals;
			for (auto it : reflectedRays) {
				const auto& reflectedRay = it.value();
				const auto& targetPositionFloat = this->targetPositions[it];
				const auto targetPosition = (glm::tvec3<T>) targetPositionFloat;

				auto closestPointOnRay = reflectedRay.closestPointOnRayTo(targetPosition);
				auto delta = targetPosition - closestPointOnRay;

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
			, const Array2D<glm::vec3>& targetPositions
			, const bool isConvex)
	{
		auto costFunction = new ceres::DynamicAutoDiffCostFunction<SphericalLensMirrorProjectionCost>(
			new SphericalLensMirrorProjectionCost(gridRays, incomingRays, targetPositions, isConvex)
		);

		// Set size of parameters and residuals
		{
			costFunction->AddParameterBlock(4);
			costFunction->SetNumResiduals(3 * gridRays.size);
		}

		return costFunction;
	}

	const Array2D<Ray<float>> gridRays;
	const Array2D<Ray<float>> incomingRays;
	const Array2D<glm::vec3> targetPositions;
	const bool isConvex;
};

struct SphericalLensMirrorMidDistanceCost {
	//-----------
	SphericalLensMirrorMidDistanceCost(const float midDistance
		, const Array2D<Ray<float>>& gridRays
		, bool isConvex)
		: gridRays(gridRays)
		, midDistance(midDistance)
		, isConvex(isConvex)
	{
	}

	//----------
	template<typename T>
	bool
		operator()(T const* const* parameters, T* residuals)
	{
		auto width = this->gridRays.width();
		auto height = this->gridRays.height();

		// Build the spherical lens
		SphericalLensMirror<T> sphericalLensMirror;
		{
			sphericalLensMirror.gridRays = (Array2D<Ray<T>>) this->gridRays;
			sphericalLensMirror.shape = this->isConvex
				? SphericalMirrorShape::Convex
				: SphericalMirrorShape::Concave;
			sphericalLensMirror.sphere.center[0] = parameters[0][0];
			sphericalLensMirror.sphere.center[1] = parameters[0][1];
			sphericalLensMirror.sphere.center[2] = parameters[0][2];
			sphericalLensMirror.sphere.radius = parameters[0][3];
		}

		// Get the lensMirror
		auto lensMirror = sphericalLensMirror.getLensMirror();
		const auto& distancesVector = lensMirror.getDistances().data;

		for (size_t i = 0; i < distancesVector.size(); i++) {
			residuals[i] = distancesVector[i] - (T)this->midDistance;
		}

		return true;
	}

	static ceres::CostFunction*
		Create(const float midDistance
			, const Array2D<Ray<float>>& gridRays
			, bool isConvex)
	{
		auto costFunction = new ceres::DynamicAutoDiffCostFunction<SphericalLensMirrorMidDistanceCost>(
			new SphericalLensMirrorMidDistanceCost(midDistance, gridRays, isConvex)
		);

		// Set size of parameters and residuals
		{
			costFunction->AddParameterBlock(4);
			costFunction->SetNumResiduals(gridRays.size);
		}

		return costFunction;
	}

	const float midDistance;
	const Array2D<Ray<float>> gridRays;
	const bool isConvex;
};

namespace ofxRulr {
	namespace Solvers {
		namespace OpticsSolvers {
			//----------
			SphericalLensMirrorProjection::Result
				SphericalLensMirrorProjection::solve(const SphericalLensMirror<float>& initialLensMirror
					, const Array2D<Ray<float>>& incomingRays
					, const Array2D<glm::vec3>& targetPositions
					, float midDistance
					, const ofxCeres::SolverSettings& solverSettings)
			{
				const auto width = incomingRays.width();
				const auto height = incomingRays.height();
				const auto size = incomingRays.size;
				const auto isConvex = initialLensMirror.shape == SphericalMirrorShape::Convex;

				// Parameters
				vector<double> parameters;
				{
					parameters.resize(4);
					parameters[0] = initialLensMirror.sphere.center[0];
					parameters[1] = initialLensMirror.sphere.center[1];
					parameters[2] = initialLensMirror.sphere.center[2];
					parameters[3] = initialLensMirror.sphere.radius;
				}

				// Build the problem
				ceres::Problem problem;
				problem.AddResidualBlock(SphericalLensMirrorProjectionCost::Create(initialLensMirror.gridRays
					, incomingRays
					, targetPositions
					, isConvex)
					, NULL
					, parameters.data());
				problem.AddResidualBlock(SphericalLensMirrorMidDistanceCost::Create(midDistance
					, initialLensMirror.gridRays
					, isConvex)
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
					result.solution.lensMirror.shape = initialLensMirror.shape;
					result.solution.lensMirror.sphere.center[0] = parameters[0];
					result.solution.lensMirror.sphere.center[1] = parameters[1];
					result.solution.lensMirror.sphere.center[2] = parameters[2];
					result.solution.lensMirror.sphere.radius = parameters[3];
				}

				return result;
			}
		}
	}
}