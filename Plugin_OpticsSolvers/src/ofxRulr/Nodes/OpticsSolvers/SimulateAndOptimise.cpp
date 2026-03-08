#include "pch_Plugin_OpticsSolvers.h"
#include "SimulateAndOptimise.h"

#include "ofxRulr/Nodes/Item/View.h"
#include "LensMirror.h"
#include "ScreenTarget.h"

#include "ofxRulr/Solvers/OpticsSolvers/LensMirrorProjection.h"
#include "ofxRulr/Solvers/OpticsSolvers/LensMirrorProjection2.h"
#include "ofxRulr/Solvers/OpticsSolvers/SphericalLensMirrorProjection.h"

namespace ofxRulr {
	namespace Nodes {
		namespace OpticsSolvers {
			//---------
			SimulateAndOptimise::SimulateAndOptimise()
			{
				RULR_NODE_INIT_LISTENER;
			}

			//---------
			string
				SimulateAndOptimise::getTypeName() const
			{
				return "OpticsSolvers::SimulateAndOptimise";
			}

			//---------
			void
				SimulateAndOptimise::init()
			{
				RULR_NODE_UPDATE_LISTENER;
				RULR_NODE_INSPECTOR_LISTENER;
				RULR_NODE_SERIALIZATION_LISTENERS;
				RULR_NODE_DRAW_WORLD_LISTENER;

				this->manageParameters(this->parameters);

				this->addInput<Item::View>();
				this->addInput<LensMirror>();
				this->addInput<ScreenTarget>();
			}

			//---------
			void
				SimulateAndOptimise::update()
			{
				auto lensMirror = this->getInput<LensMirror>();
				if (lensMirror) {
					// Auto simulate if the lens has changed
					if (lensMirror->isFrameNew()) {
						if (this->parameters.simulate.onLensChange) {
							this->simulate();
						}
					}
				}
			}

			//---------
			void
				SimulateAndOptimise::drawWorldStage()
			{
				// rays
				{
					if (ofxRulr::isActive(this, this->parameters.preview.rays.primary)) {
						this->preview.rays.primary.draw();
					}
					if (ofxRulr::isActive(this, this->parameters.preview.rays.secondary)) {
						this->preview.rays.secondary.draw();
					}
				}
				

				// projection
				{
					if (ofxRulr::isActive(this, this->parameters.preview.projection.filled)) {
						// Draw this->preview.projection as a solid mesh using the image imageTexture as the texture
						static ofImage& imageTexture{ ofxAssets::image("ofxRulr::ColorGrid") };
						imageTexture.bind();
						this->preview.projectionFilled.draw();
						imageTexture.unbind();
					}

					if (ofxRulr::isActive(this, this->parameters.preview.projection.wireframe)) {
						// draw this->preview.projection as a wireframe using the vertex colours to represent texture coordinates
						this->preview.projectionWireframe.draw();
					}
				}
			}

			//---------
			void
				SimulateAndOptimise::serialize(nlohmann::json&)
			{

			}

			//---------
			void
				SimulateAndOptimise::deserialize(const nlohmann::json&)
			{

			}

			//---------
			void
				SimulateAndOptimise::populateInspector(ofxCvGui::InspectArguments& args)
			{
				auto inspector = args.inspector;
				inspector->addButton("Simulate", [this]() {
					try {
						this->simulate();
					}
					RULR_CATCH_ALL_TO_ALERT;
					}, ' ');

				inspector->addButton("Optimise", [this]() {
					try {
						this->optimise();
					}
					RULR_CATCH_ALL_TO_ALERT;
					}, OF_KEY_RETURN)->setHeight(100.0f);
			}

			//----------
			void
				SimulateAndOptimise::simulate()
			{
				this->throwIfMissingAnyConnection();
				auto view = this->getInput<Item::View>();
				auto viewModel = view->getViewInWorldSpace();

				auto lensMirror = this->getInput<LensMirror>();
				auto lensMirrorModel = lensMirror->getModel();

				auto screenTarget = this->getInput<ScreenTarget>();
				const auto screenTargetPlane = screenTarget->getPlane();

				auto& normalGrid = lensMirrorModel.normals;

				const std::size_t width = normalGrid.width();
				const std::size_t height = normalGrid.height();
				if (width == 0 || height == 0) {
					// Clear previews if no data
					this->preview.rays.primary.clear();
					this->preview.rays.secondary.clear();
					this->preview.projectionWireframe.clear();
					this->preview.projectionFilled.clear();
					return;
				}

				// Use the gridRays as the incoming rays (bit hacky but saves compute)
				const auto& incomingRays = lensMirrorModel.gridRays;
				
				// Calculate the reflections
				lensMirrorModel.reflect(incomingRays, this->result.reflectedRays);

				// Calculate the screen intersections
				this->result.screenIntersections.allocate(width, height);
				for (auto it : result.reflectedRays) {
					const auto& reflectedRay = it.value();
					this->result.screenIntersections[it] = screenTargetPlane.intersect(reflectedRay);
				}

				// ---------- Create the previews ----------
				static ofImage& imageTexture{ ofxAssets::image("ofxRulr::ColorGrid") };

				// Rays: full path (source -> mirror point -> screen intersection)
				{
					this->preview.rays.primary.clear();
					this->preview.rays.secondary.clear();
					this->preview.rays.primary.setMode(OF_PRIMITIVE_LINES);
					this->preview.rays.secondary.setMode(OF_PRIMITIVE_LINES);

					for (const auto it : normalGrid) {
						const auto& pMirror = lensMirrorModel.positions[it];
						const auto& r0 = lensMirrorModel.gridRays[it];
						const auto& pScreen = this->result.screenIntersections[it];

						auto color = ofFloatColor(it.x(), it.y(), 0, 1.0f);

						// segment 1: source -> mirror
						this->preview.rays.primary.addVertex(r0.s);
						this->preview.rays.primary.addVertex(pMirror);
						this->preview.rays.primary.addColor(color);
						this->preview.rays.primary.addColor(color);

						// segment 2: mirror -> screen
						this->preview.rays.secondary.addVertex(pMirror);
						this->preview.rays.secondary.addVertex(pScreen);
						this->preview.rays.secondary.addColor(color);
						this->preview.rays.secondary.addColor(color);
					}
				}

				// Projection wireframe: grid on screen intersections, vertex color encodes normalized (u,v)
				{
					this->preview.projectionWireframe.clear();
					this->preview.projectionWireframe.setMode(OF_PRIMITIVE_LINES);

					// Pre-create vertex list for easy indexing
					std::vector<glm::vec3> verts;
					verts.reserve(width * height);
					for (std::size_t j = 0; j < height; ++j) {
						for (std::size_t i = 0; i < width; ++i) {
							verts.push_back(this->result.screenIntersections.at(i, j));
						}
					}

					// Add line segments (horizontal and vertical)
					auto idx = [&](std::size_t i, std::size_t j) { return (uint32_t)(i + j * width); };

					for (std::size_t j = 0; j < height; ++j) {
						for (std::size_t i = 0; i < width; ++i) {
							const glm::vec3& v = verts[idx(i, j)];
							const float u = (width > 1) ? float(i) / float(width - 1) : 0.0f;
							const float vcoord = (height > 1) ? float(j) / float(height - 1) : 0.0f;
							const ofFloatColor col(u, vcoord, 0.0f, 1.0f);

							// Horizontal edge
							if (i + 1 < width) {
								this->preview.projectionWireframe.addVertex(v);
								this->preview.projectionWireframe.addColor(col);
								this->preview.projectionWireframe.addVertex(verts[idx(i + 1, j)]);
								this->preview.projectionWireframe.addColor(ofFloatColor(
									(width > 1) ? float(i + 1) / float(width - 1) : 0.0f,
									vcoord, 0.0f, 1.0f));
							}
							// Vertical edge
							if (j + 1 < height) {
								this->preview.projectionWireframe.addVertex(v);
								this->preview.projectionWireframe.addColor(col);
								this->preview.projectionWireframe.addVertex(verts[idx(i, j + 1)]);
								this->preview.projectionWireframe.addColor(ofFloatColor(
									u,
									(height > 1) ? float(j + 1) / float(height - 1) : 0.0f,
									0.0f, 1.0f));
							}
						}
					}
				}

				// Projection filled: triangles with texture coordinates in pixel space
				{
					this->preview.projectionFilled.clear();
					this->preview.projectionFilled.setMode(OF_PRIMITIVE_TRIANGLES);

					const float texW = (float)imageTexture.getWidth();
					const float texH = (float)imageTexture.getHeight();

					// We will add vertices row-major and build triangles via indices
					auto baseIndex = (uint32_t)this->preview.projectionFilled.getNumVertices();

					// Add vertices + texcoords
					for (std::size_t j = 0; j < height; ++j) {
						for (std::size_t i = 0; i < width; ++i) {
							const glm::vec3 p = this->result.screenIntersections.at(i, j);
							const float u = (width > 1) ? float(i) / float(width - 1) : 0.0f;
							const float v = (height > 1) ? float(j) / float(height - 1) : 0.0f;

							// Pixel-space texcoords
							this->preview.projectionFilled.addVertex(p);
							this->preview.projectionFilled.addTexCoord(glm::vec2(u * texW, v * texH));
						}
					}

					// Build triangle indices for the grid
					for (std::size_t j = 0; j + 1 < height; ++j) {
						for (std::size_t i = 0; i + 1 < width; ++i) {
							const uint32_t i0 = baseIndex + (uint32_t)(i + j * width);
							const uint32_t i1 = baseIndex + (uint32_t)(i + 1 + j * width);
							const uint32_t i2 = baseIndex + (uint32_t)(i + (j + 1) * width);
							const uint32_t i3 = baseIndex + (uint32_t)(i + 1 + (j + 1) * width);

							// Two triangles: (i0,i1,i2) and (i1,i3,i2)
							this->preview.projectionFilled.addIndex(i0);
							this->preview.projectionFilled.addIndex(i1);
							this->preview.projectionFilled.addIndex(i2);

							this->preview.projectionFilled.addIndex(i1);
							this->preview.projectionFilled.addIndex(i3);
							this->preview.projectionFilled.addIndex(i2);
						}
					}
				}
			}

			//----------
			void
				SimulateAndOptimise::optimise()
			{
				this->throwIfMissingAnyConnection();
				auto view = this->getInput<Item::View>();
				auto viewModel = view->getViewInWorldSpace();

				auto lensMirror = this->getInput<LensMirror>();
				auto lensMirrorModel = lensMirror->getModel();

				auto width = lensMirrorModel.gridRays.width();
				auto height = lensMirrorModel.gridRays.height();
				auto midDistance = lensMirror->getMidDistance();

				auto screenTarget = this->getInput<ScreenTarget>();
				const auto screenTargetPositions = screenTarget->getTargetPositions(width, height);

				const auto solverSettings = this->parameters.optimise.solverSettings.getSolverSettings();

				// Re-use for efficiency
				auto incomingRays = lensMirrorModel.gridRays;
				{
					// We probably don't need to do this, but just in case
					for (auto& ray : incomingRays) {
						ray.value().t = glm::normalize(ray.value().t);
					}
				}

				if (!lensMirror->isSpherical()) {
					// Apherical
					if (this->parameters.optimise.useNewSolver.get()) {
						auto result = Solvers::OpticsSolvers::LensMirrorProjection2::solve(lensMirrorModel
							, incomingRays
							, screenTargetPositions
							, midDistance
							, solverSettings);

						lensMirror->setModel(result.solution.lensMirror);
					}
					else {
						auto result = Solvers::OpticsSolvers::LensMirrorProjection::solve(lensMirrorModel
							, incomingRays
							, screenTargetPositions
							, midDistance
							, solverSettings);

						lensMirror->setModel(result.solution.lensMirror);
					}
				}
				else {
					// Spherical
					auto sphericalMirrorModel = lensMirror->getSphericalModel();
					auto result = Solvers::OpticsSolvers::SphericalLensMirrorProjection::solve(sphericalMirrorModel
						, incomingRays
						, screenTargetPositions
						, midDistance
						, solverSettings);
					lensMirror->setSphericalModel(result.solution.lensMirror);
				}
			}
		}
	}
}