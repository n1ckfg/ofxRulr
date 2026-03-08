#pragma once
#include <limits>
#include "LensMirror.h"

namespace ofxRulr {
	namespace Models {
		namespace OpticsSolvers {
			MAKE_ENUM(SphericalMirrorShape
				, (Concave, Convex)
				, ("Concave", "Convex"));

			template<typename T>
			struct SphericalLensMirror
			{
				ofxCeres::Models::Sphere<T> sphere;
				ofxCeres::Models::Array2D<ofxCeres::Models::Ray<T>> gridRays;
				SphericalMirrorShape shape;

				LensMirror<T> getLensMirror() const {
					LensMirror<T> lensMirror;
					lensMirror.gridRays = this->gridRays;
					lensMirror.allocate(this->gridRays.width(), this->gridRays.height());

					const bool isConvex = (this->shape == SphericalMirrorShape::Convex);
					const T radiusSquared = this->sphere.radius * this->sphere.radius;
					const T eps = std::numeric_limits<T>::epsilon() * T(32);

					// Pre-fill distances with NaN so non-intersecting rays are obvious
					{
						const T nan = std::numeric_limits<T>::quiet_NaN();
						lensMirror.distances.set(nan);
					}

					for (auto it : lensMirror.gridRays) {
						auto& gridRay = it.value();

						// Validate direction (should already be normalized, but ensure not degenerate)
						const glm::tvec3<T>& rayStart = gridRay.s;
						const glm::tvec3<T>& rayDir = gridRay.t;
						const T dirDot = glm::dot(rayDir, rayDir);
						if (dirDot <= eps) {
							continue;
						}

						// Intersect with sphere -> returns entry (reversePoint) and exit (forwardsPoint)
						glm::tvec3<T> forwardsPoint; // exit point (larger parameter)
						glm::tvec3<T> reversePoint;  // entry point (smaller parameter)
						const bool intersects = this->sphere.intersect(gridRay, forwardsPoint, reversePoint);
						if (!intersects) {
							continue;
						}

						// Map intersection points to scalar distances along gridRay:
						//   position = rayStart + distance * rayDir
						auto distanceAlongRay = [&](const glm::tvec3<T>& p) -> T {
							return glm::dot(p - rayStart, rayDir) / dirDot;
							};
						const T uEntry = distanceAlongRay(reversePoint);
						const T uExit = distanceAlongRay(forwardsPoint);

						// Determine if the ray origin is inside the sphere
						const bool startsInside =
							glm::dot(rayStart - this->sphere.center, rayStart - this->sphere.center) <= radiusSquared + eps;

						glm::tvec3<T> surfacePoint;
						T chosenDistance = std::numeric_limits<T>::quiet_NaN();
						bool accept = false;

						if (isConvex) {
							// Convex mirror: rays starting inside are not valid; take the ENTRY (front-most) if in front.
							if (!startsInside && uEntry >= T(0)) {
								surfacePoint = reversePoint;
								chosenDistance = uEntry;
								accept = true;
							}
						}
						else {
							// Concave mirror: allow inside/outside; take the EXIT (front-most) if in front.
							if (uExit >= T(0)) {
								surfacePoint = forwardsPoint;
								chosenDistance = uExit;
								accept = true;
							}
						}

						if (!accept) {
							continue;
						}

						// Write the chosen point and normal
						lensMirror.positions[it] = surfacePoint;
						lensMirror.normals[it] = isConvex
							? glm::normalize(surfacePoint - sphere.center)   // outward
							: glm::normalize(sphere.center - surfacePoint); // inward

						// Write distance so that: rayStart + distance * rayDir == surfacePoint
						lensMirror.distances[it] = chosenDistance;
					}

					return lensMirror;
				}
			};
		}
	}
}
