#include "pch_Plugin_OpticsSolvers.h"
#include "LensMirrorProjection2.h"

using namespace ofxRulr::Models::OpticsSolvers;
using namespace ofxCeres::Models;

/// \brief Point on a (normalized) ray at a signed distance from its start.
template<typename T>
static inline glm::tvec3<T> pointOnRay(const Ray<T>& ray, const T& distance) {
	// Rays are assumed to have unit direction (ray.t) by contract.
	return ray.s + ray.t * distance;
}

/// \brief Reflect a unit incident direction about a (possibly non-unit) surface normal.
template<typename T>
static inline glm::tvec3<T> reflectUnitIncident(const glm::tvec3<T>& incidentUnit,
	const glm::tvec3<T>& normal) {
	const auto normalizedNormal = glm::normalize(normal);
	return incidentUnit - (T)2 * glm::dot(incidentUnit, normalizedNormal) * normalizedNormal;
}

/// \brief Projection cost for interior cells: 5 parameters (center, left, right, up, down).
/// Computes surface point at center, normal from symmetric differences, reflects incident ray,
/// and returns 3D delta from target to closest point on reflected ray.
struct LensMirrorProjectionCost2_Center5 {
	LensMirrorProjectionCost2_Center5(const Ray<float>& rayCenter,
		const Ray<float>& rayLeft,
		const Ray<float>& rayRight,
		const Ray<float>& rayUp,
		const Ray<float>& rayDown,
		const Ray<float>& rayIncident,
		const glm::vec3& targetPosition)
		: rayCenter(rayCenter)
		, rayLeft(rayLeft)
		, rayRight(rayRight)
		, rayUp(rayUp)
		, rayDown(rayDown)
		, rayIncident(rayIncident)
		, targetPosition(targetPosition)
	{
	}

	template<typename T>
	bool operator()(const T* const distanceCenter,
		const T* const distanceLeft,
		const T* const distanceRight,
		const T* const distanceUp,
		const T* const distanceDown,
		T* residuals) const
	{
		// Cast stored data to autodiff type.
		const auto rayCenter = (Ray<T>) this->rayCenter;
		const auto rayLeft = (Ray<T>) this->rayLeft;
		const auto rayRight = (Ray<T>) this->rayRight;
		const auto rayUp = (Ray<T>) this->rayUp;
		const auto rayDown = (Ray<T>) this->rayDown;
		const auto rayIncident = (Ray<T>) this->rayIncident;
		const auto target = (glm::tvec3<T>) this->targetPosition;

		// Surface sample points from their distances along normalized Array2D rays.
		const auto pointCenter = pointOnRay(rayCenter, distanceCenter[0]);
		const auto pointLeft = pointOnRay(rayLeft, distanceLeft[0]);
		const auto pointRight = pointOnRay(rayRight, distanceRight[0]);
		const auto pointUp = pointOnRay(rayUp, distanceUp[0]);
		const auto pointDown = pointOnRay(rayDown, distanceDown[0]);

		// Tangents and surface normal (unnormalized).
		const auto tangentX = pointRight - pointLeft;
		const auto tangentY = pointUp - pointDown;
		auto normal = glm::cross(tangentX, tangentY);

		// Handle degenerate cases where neighbors collapse (zero area).
		if (glm::dot(normal, normal) == T(0)) {
			residuals[0] = residuals[1] = residuals[2] = T(0);
			return true;
		}

		// Reflect incident direction about the surface normal at the center point.
		const auto reflectedDirection = reflectUnitIncident(rayIncident.t, normal);

		// Reflected ray originates at surface point and goes along reflectedDirection.
		Ray<T> reflectedRay;
		reflectedRay.s = pointCenter;
		reflectedRay.t = reflectedDirection;

		// Residual = vector from closest point on reflected ray to the target.
		const auto closestPoint = reflectedRay.closestPointOnRayTo(target);
		const auto delta = target - closestPoint;

		residuals[0] = delta.x;
		residuals[1] = delta.y;
		residuals[2] = delta.z;
		return true;
	}

	/// \brief AutoDiff factory: 3 residuals, five 1-scalar parameter blocks.
	static ceres::CostFunction* Create(const Ray<float>& rayCenter,
		const Ray<float>& rayLeft,
		const Ray<float>& rayRight,
		const Ray<float>& rayUp,
		const Ray<float>& rayDown,
		const Ray<float>& rayIncident,
		const glm::vec3& targetPosition)
	{
		return new ceres::AutoDiffCostFunction<LensMirrorProjectionCost2_Center5, 3, 1, 1, 1, 1, 1>(
			new LensMirrorProjectionCost2_Center5(rayCenter, rayLeft, rayRight, rayUp, rayDown, rayIncident, targetPosition));
	}

	const Ray<float> rayCenter, rayLeft, rayRight, rayUp, rayDown, rayIncident;
	const glm::vec3 targetPosition;
};

/// \brief Edge patterns for 4-parameter costs.
/// SingleH: one-sided horizontal (missing L or R), symmetric vertical.
/// SingleV: symmetric horizontal, one-sided vertical (missing U or D).
enum class EdgePattern { SingleH, SingleV };

/// \brief Projection cost for edge cells: 4 parameters (pattern-dependent).
/// Provides a one-sided finite difference on the missing side and symmetric on the other axis.
struct LensMirrorProjectionCost2_Edge4 {
	LensMirrorProjectionCost2_Edge4(EdgePattern pattern,
		const Ray<float>& rayCenter,
		const Ray<float>& rayA,
		const Ray<float>& rayB,
		const Ray<float>& rayC,
		const Ray<float>& rayIncident,
		const glm::vec3& targetPosition)
		: pattern(pattern)
		, rayCenter(rayCenter)
		, rayA(rayA)
		, rayB(rayB)
		, rayC(rayC)
		, rayIncident(rayIncident)
		, targetPosition(targetPosition)
	{
	}

	template<typename T>
	bool operator()(const T* const distance0,
		const T* const distance1,
		const T* const distance2,
		const T* const distance3,
		T* residuals) const
	{
		// Cast stored data to autodiff type.
		const auto rayCenter = (Ray<T>) this->rayCenter;
		const auto rayA = (Ray<T>) this->rayA;
		const auto rayB = (Ray<T>) this->rayB;
		const auto rayC = (Ray<T>) this->rayC;
		const auto rayIncident = (Ray<T>) this->rayIncident;
		const auto target = (glm::tvec3<T>) this->targetPosition;

		// Surface center point.
		const auto pointCenter = pointOnRay(rayCenter, distance0[0]);

		// Build tangents according to the edge pattern.
		glm::tvec3<T> tangentX, tangentY;

		if (this->pattern == EdgePattern::SingleH) {
			// distance1: the single available horizontal neighbor (left or right)
			// distance2, distance3: up, down (both available)
			const auto pointH = pointOnRay(rayA, distance1[0]);
			const auto pointUp = pointOnRay(rayB, distance2[0]);
			const auto pointDown = pointOnRay(rayC, distance3[0]);
			tangentX = pointH - pointCenter;       // one-sided horizontal
			tangentY = pointUp - pointDown;        // symmetric vertical
		}
		else {
			// distance1, distance2: left, right (both available)
			// distance3: the single available vertical neighbor (up or down)
			const auto pointLeft = pointOnRay(rayA, distance1[0]);
			const auto pointRight = pointOnRay(rayB, distance2[0]);
			const auto pointV = pointOnRay(rayC, distance3[0]);
			tangentX = pointRight - pointLeft;     // symmetric horizontal
			tangentY = pointV - pointCenter;       // one-sided vertical
		}

		// Surface normal from tangents.
		auto normal = glm::cross(tangentX, tangentY);
		if (glm::dot(normal, normal) == T(0)) {
			residuals[0] = residuals[1] = residuals[2] = T(0);
			return true;
		}

		// Reflect incident, form reflected ray, compute residual.
		const auto reflectedDirection = reflectUnitIncident(rayIncident.t, normal);
		Ray<T> reflectedRay;
		reflectedRay.s = pointCenter;
		reflectedRay.t = reflectedDirection;

		const auto closestPoint = reflectedRay.closestPointOnRayTo(target);
		const auto delta = target - closestPoint;

		residuals[0] = delta.x;
		residuals[1] = delta.y;
		residuals[2] = delta.z;
		return true;
	}

	/// \brief Factory for SingleH: params = {center, H, U, D}.
	static ceres::CostFunction* Create_SingleH(const Ray<float>& rayCenter,
		const Ray<float>& rayH,
		const Ray<float>& rayUp,
		const Ray<float>& rayDown,
		const Ray<float>& rayIncident,
		const glm::vec3& targetPosition)
	{
		return new ceres::AutoDiffCostFunction<LensMirrorProjectionCost2_Edge4, 3, 1, 1, 1, 1>(
			new LensMirrorProjectionCost2_Edge4(EdgePattern::SingleH, rayCenter, rayH, rayUp, rayDown, rayIncident, targetPosition));
	}

	/// \brief Factory for SingleV: params = {center, L, R, V}.
	static ceres::CostFunction* Create_SingleV(const Ray<float>& rayCenter,
		const Ray<float>& rayLeft,
		const Ray<float>& rayRight,
		const Ray<float>& rayV,
		const Ray<float>& rayIncident,
		const glm::vec3& targetPosition)
	{
		return new ceres::AutoDiffCostFunction<LensMirrorProjectionCost2_Edge4, 3, 1, 1, 1, 1>(
			new LensMirrorProjectionCost2_Edge4(EdgePattern::SingleV, rayCenter, rayLeft, rayRight, rayV, rayIncident, targetPosition));
	}

	const EdgePattern pattern;
	const Ray<float> rayCenter, rayA, rayB, rayC, rayIncident;
	const glm::vec3 targetPosition;
};

/// \brief Projection cost for corner cells: 3 parameters (center, H, V).
/// Uses one-sided finite differences on both axes.
struct LensMirrorProjectionCost2_Corner3 {
	LensMirrorProjectionCost2_Corner3(const Ray<float>& rayCenter,
		const Ray<float>& rayH,
		const Ray<float>& rayV,
		const Ray<float>& rayIncident,
		const glm::vec3& targetPosition)
		: rayCenter(rayCenter)
		, rayH(rayH)
		, rayV(rayV)
		, rayIncident(rayIncident)
		, targetPosition(targetPosition)
	{
	}

	template<typename T>
	bool operator()(const T* const distanceCenter,
		const T* const distanceH,
		const T* const distanceV,
		T* residuals) const
	{
		// Cast stored data to autodiff type.
		const auto rayCenter = (Ray<T>) this->rayCenter;
		const auto rayH = (Ray<T>) this->rayH;
		const auto rayV = (Ray<T>) this->rayV;
		const auto rayIncident = (Ray<T>) this->rayIncident;
		const auto target = (glm::tvec3<T>) this->targetPosition;

		// Surface point and one-sided neighbors.
		const auto pointCenter = pointOnRay(rayCenter, distanceCenter[0]);
		const auto pointH = pointOnRay(rayH, distanceH[0]);
		const auto pointV = pointOnRay(rayV, distanceV[0]);

		// Tangents and normal.
		const auto tangentX = pointH - pointCenter;
		const auto tangentY = pointV - pointCenter;
		auto normal = glm::cross(tangentX, tangentY);

		// Degenerate corner protection.
		if (glm::dot(normal, normal) == T(0)) {
			residuals[0] = residuals[1] = residuals[2] = T(0);
			return true;
		}

		// Reflect and compute projection residual.
		const auto reflectedDirection = reflectUnitIncident(rayIncident.t, normal);
		Ray<T> reflectedRay;
		reflectedRay.s = pointCenter;
		reflectedRay.t = reflectedDirection;

		const auto closestPoint = reflectedRay.closestPointOnRayTo(target);
		const auto delta = target - closestPoint;

		residuals[0] = delta.x;
		residuals[1] = delta.y;
		residuals[2] = delta.z;
		return true;
	}

	/// \brief AutoDiff factory: 3 residuals, three 1-scalar parameter blocks.
	static ceres::CostFunction* Create(const Ray<float>& rayCenter,
		const Ray<float>& rayH,
		const Ray<float>& rayV,
		const Ray<float>& rayIncident,
		const glm::vec3& targetPosition)
	{
		return new ceres::AutoDiffCostFunction<LensMirrorProjectionCost2_Corner3, 3, 1, 1, 1>(
			new LensMirrorProjectionCost2_Corner3(rayCenter, rayH, rayV, rayIncident, targetPosition));
	}

	const Ray<float> rayCenter, rayH, rayV, rayIncident;
	const glm::vec3 targetPosition;
};

/// \brief Mean-regularizer: residual = mean(distances) - mid
struct LensMirrorMidDistanceCostSingle {
	explicit LensMirrorMidDistanceCostSingle(float mid, size_t count)
		: mid(mid), count(count) {
	}

	template<typename T>
	bool operator()(T const* const* parameters, T* residuals) const {
		// parameters[0..count-1] are each 1-scalar blocks
		T sum = T(0);
		for (size_t i = 0; i < this->count; ++i) {
			sum += parameters[i][0];
		}
		const T mean = sum / T(this->count);
		residuals[0] = (mean - T(this->mid)) * 1e3;
		return true;
	}

	static ceres::CostFunction* Create(float mid, size_t count) {
		auto* cost = new ceres::DynamicAutoDiffCostFunction<LensMirrorMidDistanceCostSingle>(
			new LensMirrorMidDistanceCostSingle(mid, count));

		// Add `count` parameter blocks, each of size 1
		for (size_t i = 0; i < count; ++i) {
			cost->AddParameterBlock(1);
		}
		cost->SetNumResiduals(1);
		return cost;
	}

	float mid;
	size_t count;
};


namespace ofxRulr {
	namespace Solvers {
		namespace OpticsSolvers {
			/// \brief Build and solve the per-cell projection with 5/4/3 parameter blocks (center/edge/corner).
			/// Distances are shared variables; we keep the pointer arrays alive until after Solve().
			LensMirrorProjection2::Result
				LensMirrorProjection2::solve(const LensMirror<float>& initialLensMirror,
					const Array2D<Ray<float>>& incomingRays,
					const Array2D<glm::vec3>& targetPositions,
					float midDistance,
					const ofxCeres::SolverSettings& solverSettings)
			{
				// Array2D dimensions and parameter count
				const int width = (int)incomingRays.width();
				const int height = (int)incomingRays.height();
				const int count = (int)incomingRays.size;

				// Shared distance variable for each Array2D element
				std::vector<double> distances(count);
				{
					auto initialDistances = initialLensMirror.distances;
					for (auto it : initialDistances) {
						distances[it.idx()] = (double)it.value();
					}
				}

				ceres::Problem problem;

				// Keep parameter pointer arrays alive through Solve()
				std::vector<std::unique_ptr<double* []>> paramBlocks5; // center cells (5 params)
				std::vector<std::unique_ptr<double* []>> paramBlocks4; // edge cells   (4 params)
				std::vector<std::unique_ptr<double* []>> paramBlocks3; // corner cells (3 params)
				std::vector<std::unique_ptr<double* []>> paramBlocksMean; // single mean-regularizer block
				paramBlocks5.reserve(count);
				paramBlocks4.reserve(2 * width + 2 * height);
				paramBlocks3.reserve(4);
				paramBlocksMean.reserve(1);

				// ---------------------------------------------------------------------
				// Degenerate Array2D guard:
				// If width < 2 or height < 2, neighbor-based normals are undefined.
				// In this case we add ONLY the mean regularizer and solve.
				// ---------------------------------------------------------------------
				if (width < 2 || height < 2) {
					{
						ceres::CostFunction* cost = LensMirrorMidDistanceCostSingle::Create(midDistance, (size_t)count);

						// build array of 1-scalar pointers (one per element)
						auto meanParams = std::unique_ptr<double* []>(new double* [count]);
						for (int i = 0; i < count; ++i) {
							meanParams[i] = &distances[i];
						}
						paramBlocksMean.emplace_back(std::move(meanParams));

						problem.AddResidualBlock(cost, /*loss=*/nullptr,
							paramBlocksMean.back().get(),
							/*num_parameter_blocks=*/count);
					}


					// Solve only the regularization subproblem
					ceres::Solver::Summary summary;
					ceres::Solve(solverSettings.options, &problem, &summary);
					if (solverSettings.printReport) {
						std::cout << summary.FullReport() << std::endl;
					}

					// Package solution
					Result result(summary);
					{
						Array2D<float> outDistances;
						outDistances.allocate(std::max(width, 1), std::max(height, 1));
						for (auto it : outDistances) {
							it.value() = (float)distances[it.idx()];
						}
						result.solution.lensMirror.gridRays = initialLensMirror.gridRays;
						result.solution.lensMirror.setDistances(outDistances);
					}
					return result;
				}

				// ---------------------------------------------------------------------
				// Normal case (width >= 2 and height >= 2):
				// Build projection residuals per cell (5/4/3 params depending on center/edge/corner)
				// and one mean-distance regularizer for all parameters.
				// ---------------------------------------------------------------------

				for (auto it : initialLensMirror.gridRays) {
					const int i = (int)it.i();
					const int j = (int)it.j();
					const int indexCenter = (int)it.idx();

					const bool isAtLeft = (i == 0);
					const bool isAtRight = (i == width - 1);
					const bool isAtTop = (j == 0);
					const bool isAtBottom = (j == height - 1);

					const auto& rayCenter = initialLensMirror.gridRays.at(i, j);
					const auto& rayIncident = incomingRays.at(i, j);
					const auto& target = targetPositions.at(i, j);

					// Interior: 5-parameter cost (left, right, up, down present)
					if (!isAtLeft && !isAtRight && !isAtTop && !isAtBottom) {
						const auto& rayLeft = initialLensMirror.gridRays.at(i - 1, j);
						const auto& rayRight = initialLensMirror.gridRays.at(i + 1, j);
						const auto& rayUp = initialLensMirror.gridRays.at(i, j - 1);
						const auto& rayDown = initialLensMirror.gridRays.at(i, j + 1);

						ceres::CostFunction* cost =
							LensMirrorProjectionCost2_Center5::Create(rayCenter, rayLeft, rayRight, rayUp, rayDown, rayIncident, target);

						auto param = std::unique_ptr<double* []>(new double* [5]);
						param[0] = &distances[indexCenter];           // center
						param[1] = &distances[j * width + (i - 1)];   // left
						param[2] = &distances[j * width + (i + 1)];   // right
						param[3] = &distances[(j - 1) * width + i];   // up
						param[4] = &distances[(j + 1) * width + i];   // down
						paramBlocks5.emplace_back(std::move(param));

						problem.AddResidualBlock(cost, /*loss=*/nullptr, paramBlocks5.back().get(), /*num_parameter_blocks=*/5);
						continue;
					}

					// Corners: 3-parameter cost (one-sided on both axes)
					if (isAtLeft && isAtTop) {
						const auto& rayH = initialLensMirror.gridRays.at(i + 1, j);
						const auto& rayV = initialLensMirror.gridRays.at(i, j + 1);

						ceres::CostFunction* cost =
							LensMirrorProjectionCost2_Corner3::Create(rayCenter, rayH, rayV, rayIncident, target);

						auto param = std::unique_ptr<double* []>(new double* [3]);
						param[0] = &distances[indexCenter];             // center
						param[1] = &distances[j * width + (i + 1)];     // right
						param[2] = &distances[(j + 1) * width + i];     // down
						paramBlocks3.emplace_back(std::move(param));

						problem.AddResidualBlock(cost, nullptr, paramBlocks3.back().get(), 3);
						continue;
					}
					if (isAtRight && isAtTop) {
						const auto& rayH = initialLensMirror.gridRays.at(i - 1, j);
						const auto& rayV = initialLensMirror.gridRays.at(i, j + 1);

						ceres::CostFunction* cost =
							LensMirrorProjectionCost2_Corner3::Create(rayCenter, rayH, rayV, rayIncident, target);

						auto param = std::unique_ptr<double* []>(new double* [3]);
						param[0] = &distances[indexCenter];
						param[1] = &distances[j * width + (i - 1)];     // left
						param[2] = &distances[(j + 1) * width + i];     // down
						paramBlocks3.emplace_back(std::move(param));

						problem.AddResidualBlock(cost, nullptr, paramBlocks3.back().get(), 3);
						continue;
					}
					if (isAtLeft && isAtBottom) {
						const auto& rayH = initialLensMirror.gridRays.at(i + 1, j);
						const auto& rayV = initialLensMirror.gridRays.at(i, j - 1);

						ceres::CostFunction* cost =
							LensMirrorProjectionCost2_Corner3::Create(rayCenter, rayH, rayV, rayIncident, target);

						auto param = std::unique_ptr<double* []>(new double* [3]);
						param[0] = &distances[indexCenter];
						param[1] = &distances[j * width + (i + 1)];     // right
						param[2] = &distances[(j - 1) * width + i];     // up
						paramBlocks3.emplace_back(std::move(param));

						problem.AddResidualBlock(cost, nullptr, paramBlocks3.back().get(), 3);
						continue;
					}
					if (isAtRight && isAtBottom) {
						const auto& rayH = initialLensMirror.gridRays.at(i - 1, j);
						const auto& rayV = initialLensMirror.gridRays.at(i, j - 1);

						ceres::CostFunction* cost =
							LensMirrorProjectionCost2_Corner3::Create(rayCenter, rayH, rayV, rayIncident, target);

						auto param = std::unique_ptr<double* []>(new double* [3]);
						param[0] = &distances[indexCenter];
						param[1] = &distances[j * width + (i - 1)];     // left
						param[2] = &distances[(j - 1) * width + i];     // up
						paramBlocks3.emplace_back(std::move(param));

						problem.AddResidualBlock(cost, nullptr, paramBlocks3.back().get(), 3);
						continue;
					}

					// Left/Right edges (not corners): 4-parameter cost (one-sided horizontal)
					if (isAtLeft || isAtRight) {
						const bool isLeftEdge = isAtLeft;
						const auto& rayH = initialLensMirror.gridRays.at(isLeftEdge ? i + 1 : i - 1, j);
						const auto& rayUp = initialLensMirror.gridRays.at(i, j - 1);
						const auto& rayDown = initialLensMirror.gridRays.at(i, j + 1);

						ceres::CostFunction* cost =
							LensMirrorProjectionCost2_Edge4::Create_SingleH(rayCenter, rayH, rayUp, rayDown, rayIncident, target);

						auto param = std::unique_ptr<double* []>(new double* [4]);
						param[0] = &distances[indexCenter];                                   // center
						param[1] = &distances[j * width + (isLeftEdge ? i + 1 : i - 1)];      // the only horizontal neighbor
						param[2] = &distances[(j - 1) * width + i];                            // up
						param[3] = &distances[(j + 1) * width + i];                            // down
						paramBlocks4.emplace_back(std::move(param));

						problem.AddResidualBlock(cost, nullptr, paramBlocks4.back().get(), 4);
						continue;
					}

					// Top/Bottom edges (not corners): 4-parameter cost (one-sided vertical)
					{
						const bool isTopEdge = isAtTop;
						const auto& rayLeft = initialLensMirror.gridRays.at(i - 1, j);
						const auto& rayRight = initialLensMirror.gridRays.at(i + 1, j);
						const auto& rayV = initialLensMirror.gridRays.at(i, isTopEdge ? j + 1 : j - 1);

						ceres::CostFunction* cost =
							LensMirrorProjectionCost2_Edge4::Create_SingleV(rayCenter, rayLeft, rayRight, rayV, rayIncident, target);

						auto param = std::unique_ptr<double* []>(new double* [4]);
						param[0] = &distances[indexCenter];                      // center
						param[1] = &distances[j * width + (i - 1)];              // left
						param[2] = &distances[j * width + (i + 1)];              // right
						param[3] = &distances[(isTopEdge ? j + 1 : j - 1) * width + i]; // the only vertical neighbor
						paramBlocks4.emplace_back(std::move(param));

						problem.AddResidualBlock(cost, nullptr, paramBlocks4.back().get(), 4);
					}
				}

				// Add one mean-distance regularizer over all parameters
				{
					ceres::CostFunction* cost = LensMirrorMidDistanceCostSingle::Create(midDistance, (size_t)count);

					auto meanParams = std::unique_ptr<double* []>(new double* [count]);
					for (int i = 0; i < count; ++i) {
						meanParams[i] = &distances[i];
					}
					paramBlocksMean.emplace_back(std::move(meanParams));

					problem.AddResidualBlock(cost, /*loss=*/nullptr,
						paramBlocksMean.back().get(),
						/*num_parameter_blocks=*/count);
				}


				// Solve the assembled problem
				ceres::Solver::Summary summary;
				ceres::Solve(solverSettings.options, &problem, &summary);
				if (solverSettings.printReport) {
					std::cout << summary.FullReport() << std::endl;
				}

				// Write solution back
				Result result(summary);
				{
					Array2D<float> outDistances;
					outDistances.allocate(width, height);
					for (auto it : outDistances) {
						it.value() = (float)distances[it.idx()];
					}
					result.solution.lensMirror.gridRays = initialLensMirror.gridRays;
					result.solution.lensMirror.setDistances(outDistances);
				}
				return result;
			}
		} // namespace OpticsSolvers
	} // namespace Solvers
} // namespace ofxRulr
