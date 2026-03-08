#include "pch_RulrCore.h"
#include "IsFrameNew.h"

namespace ofxRulr {
	namespace Utils {
		//----------
		void
			IsFrameNew::markFrameNew()
		{
			this->nextFrameIsNew = true;
		}

		//----------
		void
			IsFrameNew::update()
		{
			this->currentFrameNew = this->nextFrameIsNew;
			this->nextFrameIsNew = false;
		}

		//----------
		bool
			IsFrameNew::isFrameNew() const
		{
			return this->currentFrameNew;
		}
	}
}