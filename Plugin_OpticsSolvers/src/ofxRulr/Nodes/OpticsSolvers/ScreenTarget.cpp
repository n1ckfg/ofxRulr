#include "pch_Plugin_OpticsSolvers.h"
#include "ScreenTarget.h"

namespace ofxRulr {
	namespace Nodes {
		namespace OpticsSolvers {
			//---------
			ScreenTarget::ScreenTarget()
			{
				RULR_NODE_INIT_LISTENER;
			}

			//---------
			string
				ScreenTarget::getTypeName() const
			{
				return "OpticsSolvers::ScreenTarget";
			}

			//---------
			void
				ScreenTarget::init()
			{
				RULR_NODE_UPDATE_LISTENER;
				RULR_RIGIDBODY_DRAW_OBJECT_ADVANCED_LISTENER;

				this->manageParameters(this->parameters);

				// Init the plane and texture
				{
					this->plane.set(1, 1, 2, 2);
					static ofImage& imageTexture{ ofxAssets::image("ofxRulr::ColorGrid") };
					plane.mapTexCoords(0, imageTexture.getHeight(), imageTexture.getWidth(), 0);
				}

			}

			//---------
			void
				ScreenTarget::drawObjectAdvanced(DrawWorldAdvancedArgs& args)
			{
				static ofImage& imageTexture{ ofxAssets::image("ofxRulr::ColorGrid") };
				imageTexture.bind();
				{
					ofPushMatrix();
					{
						ofScale(this->parameters.width.get(), this->parameters.height.get());
						this->plane.draw();
					}
					ofPopMatrix();
				}
				imageTexture.unbind();
			}

			//---------
			ofxCeres::Models::Plane<float>
				ScreenTarget::getPlane() const
			{
				ofxCeres::Models::Plane<float> plane;
				{
					plane.center = this->getPosition();
					plane.normal = (glm::mat3x3)this->getTransform() * glm::vec3(0, 0, 1);
				}
				return plane;
			}

			//---------
			ofxCeres::Models::Array2D<glm::vec3>
				ScreenTarget::getTargetPositions(size_t width, size_t height) const
			{
				ofxCeres::Models::Array2D<glm::vec3> targetPositions;
				targetPositions.allocate(width, height);

				// World-space basis for the screen (centered at this->getPosition())
				const glm::vec3 origin = this->getPosition();
				const glm::mat3 R = (glm::mat3)this->getTransform(); // rotation (ignore translation)
				const glm::vec3 axisX = glm::normalize(R * glm::vec3(1, 0, 0));
				const glm::vec3 axisY = glm::normalize(R * glm::vec3(0, 1, 0));

				// Physical size (meters or your world units)
				const float W = this->parameters.width.get();
				const float H = this->parameters.height.get();

				// Fill grid with world-space points across the rectangle
				for (auto it : targetPositions) {
					const size_t i = it.i();
					const size_t j = it.j();

					// Normalized grid coordinates in [0,1]
					const float u = (width > 1) ? float(i) / float(width - 1) : 0.0f;
					const float v = (height > 1) ? float(j) / float(height - 1) : 0.0f;

					// Map to centered rectangle on the plane
					// u=0 left edge, u=1 right edge along axisX
					// v=0 bottom edge, v=1 top edge along axisY (use your convention)
					const glm::vec3 p =
						origin
						+ (u - 0.5f) * W * axisX
						+ (v - 0.5f) * H * axisY;

					targetPositions.at(i, j) = p;
				}

				return targetPositions;
			}
		}
	}
}