#pragma once

#include "ofxRulr/Nodes/Item/RigidBody.h"

namespace ofxRulr {
	namespace Nodes {
		namespace OpticsSolvers {
			class ScreenTarget : public Item::RigidBody
			{
			public:
				ScreenTarget();
				string getTypeName() const override;

				void init();
				void drawObjectAdvanced(DrawWorldAdvancedArgs&);

				ofxCeres::Models::Plane<float> getPlane() const;
				ofxCeres::Models::Array2D<glm::vec3> getTargetPositions(size_t width, size_t height) const;
			protected:
				struct : ofParameterGroup {
					ofParameter<float> width{ "Width", 1.6f, 0.0f, 100.0f };
					ofParameter<float> height{ "Height", 1.0f, 0.0f, 100.0f };

					PARAM_DECLARE("ScreenTarget", width, height);
				} parameters;

				ofPlanePrimitive plane;
			};
		}
	}
}