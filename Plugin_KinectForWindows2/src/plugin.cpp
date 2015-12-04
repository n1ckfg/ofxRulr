#include "ofxPlugin.h"

#include "ofxRulr/Nodes/Item/KinectV2.h"
#include "ofxRulr/Nodes/Procedure/Calibrate/CameraFromKinectV2.h"
#include "ofxRulr/Nodes/Procedure/Calibrate/ProjectorFromKinectV2.h"

OFXPLUGIN_PLUGIN_MODULES_BEGIN(ofxRulr::Nodes::Base)
	OFXPLUGIN_PLUGIN_REGISTER_MODULE(ofxRulr::Nodes::Item::KinectV2);
	OFXPLUGIN_PLUGIN_REGISTER_MODULE(ofxRulr::Nodes::Procedure::Calibrate::CameraFromKinectV2);
	OFXPLUGIN_PLUGIN_REGISTER_MODULE(ofxRulr::Nodes::Procedure::Calibrate::ProjectorFromKinectV2);
OFXPLUGIN_PLUGIN_MODULES_END