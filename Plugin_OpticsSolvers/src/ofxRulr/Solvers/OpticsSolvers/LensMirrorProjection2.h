#pragma once

#include "ofxRulr/Models/OpticsSolvers/LensMirror.h"
#include "LensMirrorProjection.h"

namespace ofxRulr {
	namespace Solvers {
		namespace OpticsSolvers {
			class LensMirrorProjection2 {
			public:
				struct Solution {
					Models::OpticsSolvers::LensMirror<float> lensMirror;
				};

				typedef ofxCeres::Result<Solution> Result;

				static Result solve(const Models::OpticsSolvers::LensMirror<float>& initialLensMirror
					, const ofxCeres::Models::Array2D<ofxCeres::Models::Ray<float>>& incomingRays
					, const ofxCeres::Models::Array2D<glm::vec3>& targetPositions
					, float midDistance
					, const ofxCeres::SolverSettings& solverSettings = LensMirrorProjection::getDefaultSolverSettings());
			};
		}
	}
}