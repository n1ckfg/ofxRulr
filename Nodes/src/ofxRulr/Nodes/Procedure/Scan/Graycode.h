#pragma once

#include "../Base.h"

#include "ofxGraycode.h"
#include "ofxCvGui/Panels/Image.h"
#include "ofxRulr/Utils/VideoOutputListener.h"

namespace ofxRulr {
	namespace Nodes {
		namespace Procedure {
			namespace Scan {
				class Graycode : public Procedure::Base {
				public:
					MAKE_ENUM(VideoOutputMode
						, (None, TestPattern, Data)
						, ("None", "TestPattern", "Data"));
					
					MAKE_ENUM(PreviewMode
						, (CameraInProjector, ProjectorInCamera, Median, MedianInverse, Active)
						, ("CinP", "PinC", "Med", "MedIn", "Active"));

					MAKE_ENUM(ScanMode
						, (Balanced, Unbalanced)
						, ("Balanced", "Unbalanced"));

					Graycode();
					void init();
					string getTypeName() const override;
					ofxCvGui::PanelPtr getPanel() override;
					void update();

					void serialize(nlohmann::json &);
					void deserialize(const nlohmann::json &);

					void throwIfNotReadyForScan() const;
					void runScan();
					void clear();
					bool hasData() const;

					bool hasScanSuite() const;
					ofxGraycode::Decoder & getDecoder() const;
					const ofxGraycode::DataSet & getDataSet() const;
					void setDataSet(const ofxGraycode::DataSet &);

					void importDataSet(const string & filename = "");
					void exportDataSet(const string & filename = "");
				protected:
					struct Suite {
						shared_ptr<ofxGraycode::Payload::Base> payload;
						ofxGraycode::Encoder encoder;
						ofxGraycode::Decoder decoder;
					};

					void invalidateSuite();
					void rebuildSuite();
					void drawPreviewOnVideoOutput(const ofRectangle &);
					void populateInspector(ofxCvGui::InspectArguments &);
					void updatePreview();
					void updateTestPattern();
					ofxGraycode::Payload::Type getIntendedPayloadType();
					
					ofxCvGui::PanelPtr view;

					unique_ptr<Suite> suite;

					ofImage message;
					
					struct : ofParameterGroup {
						struct : ofParameterGroup {
							ofParameter<ScanMode> scanMode{ "Scan mode", ScanMode::Balanced };
							ofParameter<float> captureDelay{ "Capture delay [ms]", 200 };
							ofParameter<int> flushOutputFrames{ "Flush output frames", 2 };
							ofParameter<int> flushInputFrames{ "Flush input frames", 0 };
							ofParameter<float> brightness{ "Brightness [/255]", 255, 0, 255 };

							PARAM_DECLARE("Scan", scanMode, captureDelay, flushOutputFrames, flushInputFrames, brightness);
						} scan;

						struct : ofParameterGroup {
							ofParameter<float> threshold{ "Threshold", 10, 0, 255 };

							PARAM_DECLARE("Processing", threshold)
						} processing;

						struct : ofParameterGroup {
							ofParameter<VideoOutputMode> videoOutputMode{ "Video output mode", VideoOutputMode::Options::TestPattern };
							ofParameter<PreviewMode> previewMode{ "Preview mode", PreviewMode::Options::CameraInProjector };

							PARAM_DECLARE("Preview", videoOutputMode, previewMode);
						} preview;

						PARAM_DECLARE("Graycode", scan, processing, preview);
					} parameters;
					
					ofTexture testPattern;
					ofTexture preview;
					uint8_t testPatternBrightness = 0;
					bool previewDirty = true;
					bool shouldLoadWhenReady = false;

					void callbackChangePreviewMode(PreviewMode &);

					unique_ptr<Utils::VideoOutputListener> videoOutputListener;
				};
			}
		}
	}
}