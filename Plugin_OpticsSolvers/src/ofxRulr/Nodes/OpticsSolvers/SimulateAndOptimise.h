#pragma once

#include "ofxRulr/Nodes/Base.h"
#include "ofxRulr/Models/OpticsSolvers/LensMirror.h"
#include "ofxRulr/Models/OpticsSolvers/SphericalLensMirror.h"

namespace ofxRulr {
	namespace Nodes {
		namespace OpticsSolvers {
			class SimulateAndOptimise : public Base
			{
			public:
				SimulateAndOptimise();
				string getTypeName() const override;

				void init();
				void update();
				void drawWorldStage();

				void serialize(nlohmann::json&);
				void deserialize(const nlohmann::json&);
				void populateInspector(ofxCvGui::InspectArguments&);

				void simulate();
				void optimise();
			protected:
				struct : ofParameterGroup {
					struct : ofParameterGroup {
						struct : ofParameterGroup {
							ofParameter<WhenActive> primary{ "Primary", WhenActive::Selected };
							ofParameter<WhenActive> secondary{ "Secondary", WhenActive::Selected };
							PARAM_DECLARE("Rays", primary, secondary);
						} rays;

						struct : ofParameterGroup {
							ofParameter<WhenActive> filled{ "Filled", WhenActive::Always };
							ofParameter<WhenActive> wireframe{ "Wireframe", WhenActive::Never };
							PARAM_DECLARE("Projection", filled, wireframe);
						} projection;

						PARAM_DECLARE("Preview", rays, projection);
					} preview;

					struct : ofParameterGroup {
						ofParameter<bool> onLensChange{ "On lens change", true };
						ofParameter<bool> includeEdges{ "Include edges", false };
						PARAM_DECLARE("Simulate", onLensChange, includeEdges);
					} simulate;

					struct : ofParameterGroup {
						ofParameter<bool> useNewSolver{ "Use new solver", true };
						ofParameter<bool> includeEdges{ "Include edges", false };
						ofxCeres::ParameterisedSolverSettings solverSettings;
						PARAM_DECLARE("Optimise", useNewSolver, includeEdges, solverSettings)
					} optimise;

					PARAM_DECLARE("Simulate", preview, simulate, optimise);
				} parameters;
				
				struct {
					ofxCeres::Models::Array2D<ofxCeres::Models::Ray<float>> reflectedRays;
					ofxCeres::Models::Array2D<glm::vec3> screenIntersections;
				} result;

				struct {
					struct {
						ofMesh primary;
						ofMesh secondary;
					} rays; // lines that represent the ray paths

					ofMesh projectionWireframe;
					ofMesh projectionFilled;
				} preview;
			};
		}
	}
}