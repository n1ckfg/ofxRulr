#pragma once

#include "ofxCeres.h"

namespace ofxRulr {
	namespace Models {
		namespace OpticsSolvers {
			template<typename T>
			struct LensMirror
			{
				// The mirror's vertices all line on these rays which are
				// themselves generally defined in relation to a projector or camera's rays
				ofxCeres::Models::Array2D<ofxCeres::Models::Ray<T>> gridRays;

				ofxCeres::Models::Array2D<T> distances;
				ofxCeres::Models::Array2D<glm::tvec3<T>> positions;
				ofxCeres::Models::Array2D<glm::tvec3<T>> normals;

				void allocate(size_t width, size_t height) {
					this->gridRays.allocate(width, height);

					this->distances.allocate(width, height);
					this->distances.set((T)0.0);

					this->positions.allocate(width, height);
					this->positions.set(glm::tvec3<T>((T)0.0));

					this->normals.allocate(width, height);
					this->normals.set(glm::tvec3<T>((T)0.0));
				}

				void setDistances(const ofxCeres::Models::Array2D<T>& distances) {
					this->distances = distances;
					this->calculateFromDistances();
				}

				ofxCeres::Models::Array2D<T>& getDistances() {
					return this->distances;
				}

				ofxCeres::Models::Array2D<glm::tvec3<T>>& getPositions() {
					return this->positions;
				}

				ofxCeres::Models::Array2D<glm::tvec3<T>>& getNormals() {
					return this->normals;
				}

				void calculateFromDistances() {
					// We have distances and need positions and normals
					const auto width = this->distances.width();
					const auto height = this->distances.height();

					// Calculate positions
					{
						this->positions.allocate(width, height);
						for (auto it : this->distances) {
							const auto& distance = it.value();
							const auto& ray = gridRays[it];
							this->positions[it] = ray.s + ray.t * distance;
						}
					}

					this->calculateFromPositions();
				}

				void calculateFromPositions() {
					// We have distances and positions but need normals
					const auto width = this->distances.width();
					const auto height = this->distances.height();

					this->normals.allocate(width, height);
					auto getP = [&](std::size_t i, std::size_t j) -> glm::tvec3<T> {
						const auto& p = this->positions.at(i, j);
						return glm::tvec3<T>(p.x, p.y, p.z);
						};

					for (std::size_t j = 0; j < height; ++j) {
						for (std::size_t i = 0; i < width; ++i) {
							// dP/di (x-direction)
							glm::tvec3<T> Dx;
							if (i == 0 && width > 1) {
								Dx = getP(i + 1, j) - getP(i, j);            // forward diff at left edge
							}
							else if (i == width - 1 && width > 1) {
								Dx = getP(i, j) - getP(i - 1, j);            // backward diff at right edge
							}
							else if (width > 2) {
								Dx = getP(i + 1, j) - getP(i - 1, j);        // central diff
							}
							else {
								Dx = glm::tvec3<T>(1, 0, 0);                 // degenerate 1-pixel width
							}

							// dP/dj (y-direction)
							glm::tvec3<T> Dy;
							if (j == 0 && height > 1) {
								Dy = getP(i, j + 1) - getP(i, j);            // forward diff at top edge
							}
							else if (j == height - 1 && height > 1) {
								Dy = getP(i, j) - getP(i, j - 1);            // backward diff at bottom edge
							}
							else if (height > 2) {
								Dy = getP(i, j + 1) - getP(i, j - 1);        // central diff
							}
							else {
								Dy = glm::tvec3<T>(0, 1, 0);                 // degenerate 1-pixel height
							}

							// Normal = normalized cross of tangents
							glm::tvec3<T> N = glm::cross(Dy, Dx);            // flip order if orientation is inverted
							T len = glm::length(N);
							if (len > T(0)) {
								N /= len;
							}
							else {
								// Rare degenerate neighborhood; choose a fallback
								N = glm::tvec3<T>(0, 0, 1);
							}

							this->normals.at(i, j) = N;
						}
					}
				}
				
				ofxCeres::Models::Array2D<ofxCeres::Models::Plane<T>>
					getReflectionPlanes() const
				{
					ofxCeres::Models::Array2D<ofxCeres::Models::Plane<T>> reflectionPlanes;
					reflectionPlanes.allocate(this->normals.width(), this->normals.height());

					for (auto it : this->normals) {
						const auto& normal = it.value();
						const auto& position = this->positions[it];
						reflectionPlanes[it].center = position;
						reflectionPlanes[it].normal = normal;
					}

					return reflectionPlanes;
				}

				void
					reflect(const ofxCeres::Models::Array2D<ofxCeres::Models::Ray<T>>& incomingRays,
						ofxCeres::Models::Array2D<ofxCeres::Models::Ray<T>>& reflectedRays) const
				{
					// build per-vertex planes
					auto reflectionPlanes = this->getReflectionPlanes();

					// allocate output
					reflectedRays.allocate(incomingRays.width(), incomingRays.height());

					for (auto it : incomingRays) {
						const auto& incomingRay = it.value();
						const auto& reflectionPlane = reflectionPlanes[it];

						reflectedRays[it] = reflectionPlane.reflect(incomingRay);
					}
				}

				void
					serialize(nlohmann::json& json)
				{
					this->gridRays.serialize(json["gridRays"]);
					this->distances.serialize(json["distances"]);
					this->positions.serialize(json["positions"]);
					this->normals.serialize(json["normals"]);
				}

				void
					deserialize(const nlohmann::json& json)
				{
					if (json.contains("gridRays")) {
						this->gridRays.deserialize(json["gridRays"]);
					}
					if (json.contains("distances")) {
						this->distances.deserialize(json["distances"]);
					}
					if (json.contains("positions")) {
						this->positions.deserialize(json["positions"]);
					}
					if (json.contains("normals")) {
						this->normals.deserialize(json["normals"]);
					}
				}
			};
		} // namespace OpticsSolvers
	} // namespace Models
} // namespace ofxRulr
