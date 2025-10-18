#pragma once

namespace ofxRulr {
	namespace Utils {
		class IsFrameNew {
		public:
			void markFrameNew();

			void update();
			bool isFrameNew() const;
		protected:
			bool nextFrameIsNew = false;
			bool currentFrameNew = false;
		};
	}
}