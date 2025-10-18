#include "pch_Plugin_OpticsSolvers.h"
#include "ofxPlugin.h"

#include "ofxRulr/Nodes/OpticsSolvers/ScreenTarget.h"
#include "ofxRulr/Nodes/OpticsSolvers/LensMirror.h"
#include "ofxRulr/Nodes/OpticsSolvers/SimulateAndOptimise.h"


OFXPLUGIN_PLUGIN_MODULES_BEGIN(ofxRulr::Nodes::Base)
	OFXPLUGIN_PLUGIN_REGISTER_MODULE(ofxRulr::Nodes::OpticsSolvers::ScreenTarget);
	OFXPLUGIN_PLUGIN_REGISTER_MODULE(ofxRulr::Nodes::OpticsSolvers::LensMirror);
	OFXPLUGIN_PLUGIN_REGISTER_MODULE(ofxRulr::Nodes::OpticsSolvers::SimulateAndOptimise);
OFXPLUGIN_PLUGIN_MODULES_END