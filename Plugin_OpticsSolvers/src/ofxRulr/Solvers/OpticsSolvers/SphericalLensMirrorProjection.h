#pragma once

#include "ofxRulr/Models/OpticsSolvers/SphericalLensMirror.h"
#include "LensMirrorProjection.h"

namespace ofxRulr {
	namespace Solvers {
		namespace OpticsSolvers {
			class SphericalLensMirrorProjection {
			public:
				struct Solution {
					Models::OpticsSolvers::SphericalLensMirror<float> lensMirror;
				};

				typedef ofxCeres::Result<Solution> Result;

				static Result solve(const Models::OpticsSolvers::SphericalLensMirror<float>& initialLensMirror
					, const ofxCeres::Models::Array2D<ofxCeres::Models::Ray<float>>& incomingRays
					, const ofxCeres::Models::Array2D<glm::vec3>& targetPositions
					, float midDistance
					, const ofxCeres::SolverSettings& solverSettings = LensMirrorProjection::getDefaultSolverSettings());
			};
		}
	}
}