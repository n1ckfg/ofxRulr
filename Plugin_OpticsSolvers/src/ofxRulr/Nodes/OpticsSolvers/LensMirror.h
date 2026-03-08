#pragma once

#include "ofxRulr/Nodes/Base.h"
#include "ofxRulr/Models/OpticsSolvers/LensMirror.h"
#include "ofxRulr/Models/OpticsSolvers/SphericalLensMirror.h"

namespace ofxRulr {
	namespace Nodes {
		namespace OpticsSolvers {
			class LensMirror : public Base
			{
			public:
				LensMirror();
				string getTypeName() const override;

				void init();
				void update();
				void drawWorldStage();
				ofxCvGui::PanelPtr getPanel() override;

				void serialize(nlohmann::json&);
				void deserialize(const nlohmann::json&);
				void populateInspector(ofxCvGui::InspectArguments&);

				void allocate();
				void calculate();

				const Models::OpticsSolvers::LensMirror<float>& getModel() const;
				void setModel(const Models::OpticsSolvers::LensMirror<float>&);

				bool isSpherical() const;
				Models::OpticsSolvers::SphericalLensMirror<float> getSphericalModel() const;
				void setSphericalModel(const Models::OpticsSolvers::SphericalLensMirror<float>&);

				float getMidDistance() const;

				bool isFrameNew() const;
			protected:
				void rebuildPreview();

				struct : ofParameterGroup {
					ofParameter<int> resolution{ "Resolution", 5, 2, 100 };
					ofParameter<float> midDistance{ "Mid distance", 1.0, 0.0f, 100.0f };

					struct : ofParameterGroup {
						ofParameter<bool> enabled{ "Enabled", false };

						struct : ofParameterGroup {
							ofParameter<float> x{ "X", 0.0, -5.0, 5.0 };
							ofParameter<float> y{ "Y", 0.0, -5.0, 5.0 };
							ofParameter<float> z{ "Z", 0.0, -5.0, 5.0 };
							PARAM_DECLARE("Position", x, y, z);
						} position;

						ofParameter<float> radius{ "Radius", 1.0f };

						ofParameter<Models::OpticsSolvers::SphericalMirrorShape> shape{ "Shape", Models::OpticsSolvers::SphericalMirrorShape::Convex };

						PARAM_DECLARE("Spherical", enabled, position, radius, shape)
					} spherical;

					PARAM_DECLARE("LensMirror", resolution, midDistance, spherical);
				} parameters;

				struct {
					bool needsAllocate = true;
					bool needsCalculate = true;
					Models::OpticsSolvers::LensMirror<float> data;
					Utils::IsFrameNew isFrameNew;
				} lensMirrorModel;

				struct {
					bool needsRebuild = true;
					ofMesh grid; // lines that represent the grid of positions
					ofMesh normals; // lines that represent the normals of the mirror with colours applied
				} preview;

				ofxCvGui::PanelPtr panel;
			};
		}
	}
}