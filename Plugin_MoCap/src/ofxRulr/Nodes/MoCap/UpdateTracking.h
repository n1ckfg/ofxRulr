#pragma once

#include "ThreadedProcessNode.h"
#include "MatchMarkers.h"
#include "Body.h"

namespace ofxRulr {
	namespace Nodes {
		namespace MoCap {
			struct UpdateTrackingFrame {
				shared_ptr<MatchMarkersFrame> incomingFrame;

				cv::Mat modelViewRotationVector;
				cv::Mat modelViewTranslation;

				cv::Mat modelRotationVector;
				cv::Mat modelTranslation;
				ofMatrix4x4 transform;
			};

			class UpdateTracking : public ThreadedProcessNode<MatchMarkers
				, MatchMarkersFrame
				, UpdateTrackingFrame> {
			public:
				UpdateTracking();
				string getTypeName() const override;
				void init();
				void update();
			protected:
				void processFrame(shared_ptr<MatchMarkersFrame> incomingFrame) override;

				struct : ofParameterGroup {
					ofParameter<bool> updateCamera{ "Update camera pose", false };
					PARAM_DECLARE("UpdateTracking", updateCamera);
				} parameters;

				shared_ptr<Body> bodyNode;
				mutex bodyNodeMutex;
			};
		}
	}
}