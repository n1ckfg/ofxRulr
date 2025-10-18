#include "pch_Plugin_OpticsSolvers.h"
#include "LensMirror.h"

namespace ofxRulr {
	namespace Nodes {
		namespace OpticsSolvers {
			//----------
			LensMirror::LensMirror()
			{
				RULR_NODE_INIT_LISTENER;
			}

			//----------
			string
				LensMirror::getTypeName() const
			{
				return "OpticsSolver::LensMirror";
			}

			//----------
			void
				LensMirror::init()
			{
				RULR_NODE_UPDATE_LISTENER;
				RULR_NODE_DRAW_WORLD_LISTENER;
				RULR_NODE_INSPECTOR_LISTENER;
				RULR_NODE_SERIALIZATION_LISTENERS;

				this->addInput<Item::View>();
				this->manageParameters(this->parameters);

				{
					this->panel = ofxCvGui::Panels::makeBlank();
					this->panel->onDraw += [this](ofxCvGui::DrawArguments& args) {
						ofPushStyle();

						const auto& grid = this->lensMirrorModel.data.distances;
						if (grid.size == 0 || grid.width() == 0 || grid.height() == 0) {
							ofxCvGui::Utils::drawText("No distances", args.localBounds, true, false);
							ofPopStyle();
							return;
						}

						const auto cols = (int)grid.width();
						const auto rows = (int)grid.height();

						const float x0 = args.localBounds.x;
						const float y0 = args.localBounds.y;
						const float w = args.localBounds.getWidth();
						const float h = args.localBounds.getHeight();

						const float cellW = w / (float)cols;
						const float cellH = h / (float)rows;

						size_t digits = cellW / 20;
						if (digits < 1) {
							digits = 1;
						}

						// background
						ofSetColor(18, 18, 18);
						ofDrawRectangle(args.localBounds);

						// grid lines
						ofNoFill();
						ofSetColor(80);
						ofSetLineWidth(1.0f);
						for (int c = 0; c <= cols; ++c) {
							const float x = x0 + c * cellW;
							ofDrawLine(x, y0, x, y0 + h);
						}
						for (int r = 0; r <= rows; ++r) {
							const float y = y0 + r * cellH;
							ofDrawLine(x0, y, x0 + w, y);
						}

						// values
						ofFill();
						ofSetColor(255);
						for (int j = 0; j < rows; ++j) {
							for (int i = 0; i < cols; ++i) {
								const float v = grid.at(i, j);

								// cell bounds with a little padding
								ofRectangle cell(x0 + i * cellW, y0 + j * cellH, cellW, cellH);
								ofRectangle padded = cell;
								const float pad = 4.0f;
								padded.x += pad;
								padded.y += pad;
								padded.width = std::max(0.0f, padded.width - 2 * pad);
								padded.height = std::max(0.0f, padded.height - 2 * pad);

								// draw value (4 dp), no background/scissor to keep grid visible
								ofxCvGui::Utils::drawText(ofToString(v, digits), padded, /*background=*/false, /*scissor=*/false);
							}
						}

						ofPopStyle();
						};
				}
			}

			//----------
			void
				LensMirror::update()
			{
				// Check the lens resolution
				{
					auto size = this->parameters.resolution.get();
					if (this->lensMirrorModel.data.getPositions().width() != size
						|| this->lensMirrorModel.data.getPositions().height() != size) {
						this->lensMirrorModel.needsAllocate = true;
					}
				}

				try {
					if (this->lensMirrorModel.needsAllocate) {
						this->allocate();
					}

					if (this->lensMirrorModel.needsCalculate) {
						this->calculate();
					}

					if (this->preview.needsRebuild) {
						this->rebuildPreview();
					}
				}
				RULR_CATCH_ALL_TO_ERROR;

				this->lensMirrorModel.isFrameNew.update();
			}

			//----------
			void
				LensMirror::serialize(nlohmann::json& json)
			{
				this->lensMirrorModel.data.serialize(json["lensMirrorModel"]);
			}

			//----------
			void
				LensMirror::deserialize(const nlohmann::json& json)
			{
				if (json.contains("lensMirrorModel")) {
					this->lensMirrorModel.data.deserialize(json["lensMirrorModel"]);

					// Check if we got data 
					{
						if (this->lensMirrorModel.data.distances.size > 0) {
							this->lensMirrorModel.needsAllocate = false;
						}

						if (this->lensMirrorModel.data.normals.size > 0) {
							this->lensMirrorModel.needsCalculate = false;
						}
					}
				}
				this->preview.needsRebuild = true;
			}

			//----------
			void
				LensMirror::populateInspector(ofxCvGui::InspectArguments& args)
			{
				auto inspector = args.inspector;
				inspector->addButton("Allocate", [this]() {
					try {
						this->allocate();
					}
					RULR_CATCH_ALL_TO_ERROR;
					});
				inspector->addButton("Calculate", [this]() {
					try {
						this->calculate();
					}
					RULR_CATCH_ALL_TO_ERROR;
					}, ' ');
			}

			//----------
			void 
				LensMirror::drawWorldStage()
			{
				this->preview.grid.draw();
				this->preview.normals.draw();
			}

			//----------
			ofxCvGui::PanelPtr
				LensMirror::getPanel()
			{
				return this->panel;
			}

			//----------
			void
				LensMirror::allocate()
			{
				auto size = this->parameters.resolution.get();
				this->lensMirrorModel.data.allocate(size, size);

				auto& distances = this->lensMirrorModel.data.getDistances();
				distances.set(this->parameters.midDistance.get());

				this->lensMirrorModel.needsAllocate = false;
				this->lensMirrorModel.needsCalculate = true;
				this->preview.needsRebuild = true;
			}

			//----------
			void
				LensMirror::calculate()
			{
				this->throwIfMissingAConnection<Item::View>();
				auto viewModel = this->getInput<Item::View>()->getViewInWorldSpace();
				auto size = this->parameters.resolution.get();

				for (auto it : this->lensMirrorModel.data.gridRays) {
					// Calculate the coordinate
					glm::vec2 coordinate{
						ofMap(it.i(), 0, size - 1, -1, 1)
						, ofMap(it.j(), 0, size - 1, 1, -1)
					};

					// Cast the coordinate
					auto ofxRayRay = viewModel.castCoordinate(coordinate);

					// Convert from ofxRay to ofxCeres object and store
					auto& ofxCeresRay = it.value();
					ofxCeresRay.s = ofxRayRay.s;
					ofxCeresRay.t = glm::normalize(ofxRayRay.t);
				}

				if (this->parameters.spherical.enabled) {
					// Spherical
					auto sphericalLensModel = this->getSphericalModel();
					this->lensMirrorModel.data = sphericalLensModel.getLensMirror();
				}
				else {
					this->lensMirrorModel.data.calculateFromDistances();
				}

				this->lensMirrorModel.isFrameNew.markFrameNew();
				this->lensMirrorModel.needsCalculate = false;
				this->preview.needsRebuild = true;
			}

			//----------
			const Models::OpticsSolvers::LensMirror<float>&
				LensMirror::getModel() const
			{
				return this->lensMirrorModel.data;
			}

			//----------
			void
				LensMirror::setModel(const Models::OpticsSolvers::LensMirror<float>& lensMirrorModel)
			{
				this->lensMirrorModel.data = lensMirrorModel;
				if (this->lensMirrorModel.data.distances.size > 0) {
					this->lensMirrorModel.needsAllocate = false;
				}
				if (this->lensMirrorModel.data.normals.size > 0) {
					this->lensMirrorModel.needsCalculate = false;
				}
				this->lensMirrorModel.isFrameNew.markFrameNew();
				this->preview.needsRebuild = true;
			}
	
			//----------
			bool
				LensMirror::isSpherical() const
			{
				return this->parameters.spherical.enabled.get();
			}

			//----------
			Models::OpticsSolvers::SphericalLensMirror<float>
				LensMirror::getSphericalModel() const
			{
				Models::OpticsSolvers::SphericalLensMirror<float> sphericalLensModel;

				sphericalLensModel.gridRays = this->lensMirrorModel.data.gridRays;
				sphericalLensModel.shape = this->parameters.spherical.shape;
				sphericalLensModel.sphere.center.x = this->parameters.spherical.position.x.get();
				sphericalLensModel.sphere.center.y = this->parameters.spherical.position.y.get();
				sphericalLensModel.sphere.center.z = this->parameters.spherical.position.z.get();
				sphericalLensModel.sphere.radius = this->parameters.spherical.radius.get();
				
				return sphericalLensModel;
			}

			//----------
			void
				LensMirror::setSphericalModel(const Models::OpticsSolvers::SphericalLensMirror<float>& sphericalLensModel)
			{
				this->parameters.spherical.enabled.set(true);

				// Copy spherical parameters to this->parameters.spherical
				this->parameters.spherical.shape.set(sphericalLensModel.shape);
				this->parameters.spherical.position.x.set(sphericalLensModel.sphere.center.x);
				this->parameters.spherical.position.y.set(sphericalLensModel.sphere.center.y);
				this->parameters.spherical.position.z.set(sphericalLensModel.sphere.center.z);
				this->parameters.spherical.radius.set(sphericalLensModel.sphere.radius);
				
				// Set our data to match
				this->lensMirrorModel.data = sphericalLensModel.getLensMirror();

				this->lensMirrorModel.needsAllocate = false;
				this->lensMirrorModel.needsCalculate = false;
				this->lensMirrorModel.isFrameNew.markFrameNew();
				this->preview.needsRebuild = true;
			}


			//----------
			float
				LensMirror::getMidDistance() const
			{
				return this->parameters.midDistance.get();
			}

			//----------
			bool
				LensMirror::isFrameNew() const
			{
				return this->lensMirrorModel.isFrameNew.isFrameNew();
			}

			//----------
			void
				LensMirror::rebuildPreview()
			{
				// Clear and set mesh modes
				this->preview.grid.clear();
				this->preview.grid.setMode(OF_PRIMITIVE_LINES);
				this->preview.normals.clear();
				this->preview.normals.setMode(OF_PRIMITIVE_LINES);

				// Access model data
				auto& positions = this->lensMirrorModel.data.getPositions();
				auto& normals = this->lensMirrorModel.data.getNormals();

				const std::size_t W = positions.width();
				const std::size_t H = positions.height();

				if (W == 0 || H == 0) {
					this->preview.needsRebuild = false;
					return;
				}

				// --- Estimate a reasonable normal scale from local spacing (robust but simple)
				float normalScale = 0.01f; // fallback
				{
					// Try to estimate from first available horizontal/vertical edge length
					int samples = 0;
					double acc = 0.0;
					for (std::size_t j = 0; j < H; ++j) {
						for (std::size_t i = 0; i < W; ++i) {
							const auto& p = positions.at(i, j);
							if (i + 1 < W) {
								const auto& px = positions.at(i + 1, j);
								acc += glm::length(px - p);
								++samples;
							}
							if (j + 1 < H) {
								const auto& py = positions.at(i, j + 1);
								acc += glm::length(py - p);
								++samples;
							}
						}
					}
					if (samples > 0) {
						normalScale = static_cast<float>((acc / samples) * 0.25); // normals ~ 1/4 cell size
					}
				}

				const ofFloatColor gridColor(1.0f, 1.0f, 1.0f, 1.0f); // white lines for grid

				// Build grid (wireframe) and normals as line segments
				for (std::size_t j = 0; j < H; ++j) {
					for (std::size_t i = 0; i < W; ++i) {
						const glm::vec3 p = positions.at(i, j);

						// Horizontal segment (i, j) -> (i+1, j)
						if (i + 1 < W) {
							const glm::vec3 px = positions.at(i + 1, j);
							this->preview.grid.addVertex(p);
							this->preview.grid.addVertex(px);
							this->preview.grid.addColor(gridColor);
							this->preview.grid.addColor(gridColor);
						}

						// Vertical segment (i, j) -> (i, j+1)
						if (j + 1 < H) {
							const glm::vec3 py = positions.at(i, j + 1);
							this->preview.grid.addVertex(p);
							this->preview.grid.addVertex(py);
							this->preview.grid.addColor(gridColor);
							this->preview.grid.addColor(gridColor);
						}

						// Normal segment at (i, j)
						if (i < normals.width() && j < normals.height()) {
							const glm::vec3 n = normals.at(i, j);
							const glm::vec3 q = p + n * normalScale;

							// Color normals by direction (map [-1,1] -> [0,1])
							ofFloatColor nColor(
								0.5f * (n.x + 1.0f),
								0.5f * (n.y + 1.0f),
								0.5f * (n.z + 1.0f),
								1.0f
							);

							this->preview.normals.addVertex(p);
							this->preview.normals.addVertex(q);
							this->preview.normals.addColor(nColor);
							this->preview.normals.addColor(nColor);
						}
					}
				}

				this->preview.needsRebuild = false;
			}
		}
	}
}
