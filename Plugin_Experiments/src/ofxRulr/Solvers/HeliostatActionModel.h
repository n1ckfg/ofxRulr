#pragma once
#include "ofxCeres.h"
#include <glm/glm.hpp>

#include "ofxRulr/Nodes/Experiments/MirrorPlaneCapture/Dispatcher.h"

namespace ofxRulr {
	namespace Solvers {
		class HeliostatActionModel {
		public:
			template<typename T>
			struct Parameters {
				struct Axis {
					glm::tvec3<T> polynomial;
					glm::tvec3<T> rotationAxis;

					struct {
						T minimum;
						T maximum;
					} angleRange;
				};

				glm::tvec3<T> position;

				Axis axis1;
				Axis axis2;

				T mirrorOffset;

				template<typename T2>
				Parameters<T2> castTo() const {
					Parameters<T2> newParameters;
					newParameters.position = glm::tvec3<T2>(this->position);
					newParameters.axis1.polynomial = glm::tvec3<T2>(this->axis1.polynomial);
					newParameters.axis1.rotationAxis = glm::tvec3<T2>(this->axis1.rotationAxis);
					newParameters.axis1.angleRange.minimum = (T2)this->axis1.angleRange.minimum;
					newParameters.axis1.angleRange.maximum= (T2) this->axis1.angleRange.maximum;
					newParameters.axis2.polynomial = glm::tvec3<T2>(this->axis2.polynomial);
					newParameters.axis2.rotationAxis = glm::tvec3<T2>(this->axis2.rotationAxis);
					newParameters.axis2.angleRange.minimum = (T2)this->axis2.angleRange.minimum;
					newParameters.axis2.angleRange.maximum = (T2)this->axis2.angleRange.maximum;
					newParameters.mirrorOffset = (T2)this->mirrorOffset;
					return newParameters;
				}
			};

			template<typename T>
			struct AxisAngles {
				T axis1;
				T axis2;
			};

			template<typename T>
			static void getMirrorCenterAndNormal(const AxisAngles<T>& axisAngles
				, const Parameters<T>& parameters
				, glm::tvec3<T>& center
				, glm::tvec3<T>& normal) {

				auto mirrorTransform = glm::translate<T>(parameters.position)
					* glm::rotate<T>(axisAngles.axis1 * DEG_TO_RAD
						, parameters.axis1.rotationAxis)
					* glm::rotate<T>(axisAngles.axis2 * DEG_TO_RAD
						, parameters.axis2.rotationAxis)
					* glm::translate<T>(glm::tvec3<T>(0, -parameters.mirrorOffset, 0));

				center = ofxCeres::VectorMath::applyTransform<T>(mirrorTransform, { 0, 0, 0 });
				auto centerPlusNormal = ofxCeres::VectorMath::applyTransform<T>(mirrorTransform, { 0, -1, 0 });
				normal = ofxCeres::VectorMath::normalize(centerPlusNormal - center);
			}

			template<typename T>
			static T angleToPosition(const T& angle, const glm::tvec3<T>& polynomial)
			{
				auto correctedAngle =
					angle * polynomial[0]
					+ angle * angle * polynomial[1]
					+ angle * angle * angle * polynomial[2];
				auto goalPosition = (correctedAngle + (T)180.0) / (T)360.0 * (T)4096.0;
				return goalPosition;
			}

			class Navigator {
			public:
				struct Solution {
					AxisAngles<float> axisAngles;
				};
				typedef ofxCeres::Result<Solution> Result;

				static ofxCeres::SolverSettings defaultSolverSettings();

				static Result solveNormal(const Parameters<float> &
					, const glm::vec3 & normal
					, const AxisAngles<float> & initialAngles
					, const ofxCeres::SolverSettings& solverSettings = defaultSolverSettings());

				static Result solvePointToPoint(const Parameters<float>&
					, const glm::vec3& pointA
					, const glm::vec3& pointB
					, const AxisAngles<float>& initialAngles
					, const ofxCeres::SolverSettings& solverSettings = defaultSolverSettings());

				static bool validate(const Parameters<float>&
					, const AxisAngles<float>& axisAngles);

				// returns true if angles were altered
				static bool constrainAngles(const Parameters<float>&
					, AxisAngles<float>& axisAngles
					, const AxisAngles<float>& initialAngles);

				static Result solveConstrained(const Parameters<float>&
					, std::function<Result(const AxisAngles<float>&)>
					, const AxisAngles<float>& initialAngles);
			};

			class SolveAngle {
			public:
				typedef float Solution;
				typedef ofxCeres::Result<Solution> Result;
				static ofxCeres::SolverSettings defaultSolverSettings();

				static Result solveAngle(const Nodes::Experiments::MirrorPlaneCapture::Dispatcher::RegisterValue&
					, const glm::vec3& polynomial
					, const float& priorAngle
					, const ofxCeres::SolverSettings& solverSettings = defaultSolverSettings());
			};
		};
	}
}