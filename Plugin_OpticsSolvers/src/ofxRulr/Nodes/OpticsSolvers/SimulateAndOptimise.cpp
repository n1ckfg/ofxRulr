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

				inspector->addButton("Export system", [this]() {
					try {
						this->exportSystem();
					}
					RULR_CATCH_ALL_TO_ALERT;
					})->setHeight(100.0f);
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

			//----------
			void
				SimulateAndOptimise::exportSystem()
			{
				this->throwIfMissingAnyConnection();

				// Ask for output path
				auto response = ofSystemSaveDialog("system.glb", "Save GLB file");
				if (!response.bSuccess) {
					return;
				}
				const auto outputPath = response.filePath;

				// Validate lens mirror
				auto lensMirror = this->getInput<LensMirror>();
				if (!lensMirror) {
					throw(ofxRulr::Exception("LensMirror node is missing"));
				}
				auto lensMirrorModel = lensMirror->getModel();
				const auto& positionsGrid = lensMirrorModel.getPositions();
				const auto& normalsGrid = lensMirrorModel.getNormals();
				if (positionsGrid.width() == 0 || positionsGrid.height() == 0) {
					throw(ofxRulr::Exception("LensMirror positions are not allocated"));
				}
				if (normalsGrid.width() != positionsGrid.width()
					|| normalsGrid.height() != positionsGrid.height()) {
					throw(ofxRulr::Exception("LensMirror normals and positions dimensions do not match"));
				}

				// Validate screen target
				auto screenTarget = this->getInput<ScreenTarget>();
				if (!screenTarget) {
					throw(ofxRulr::Exception("ScreenTarget node is missing"));
				}
				auto targetGrid = screenTarget->getTargetPositions(positionsGrid.width(), positionsGrid.height());
				if (targetGrid.width() == 0 || targetGrid.height() == 0) {
					throw(ofxRulr::Exception("ScreenTarget target positions not allocated"));
				}

				// Validate camera
				auto viewNode = this->getInput<Item::View>();
				if (!viewNode) {
					throw(ofxRulr::Exception("View node is missing"));
				}
				const auto viewWS = viewNode->getViewInWorldSpace();
				const glm::mat4 V = viewWS.getViewMatrix();
				if (glm::determinant(V) == 0.0f) {
					throw(ofxRulr::Exception("View matrix not invertible"));
				}

				// Sizes
				const std::size_t W = positionsGrid.width();
				const std::size_t H = positionsGrid.height();
				if (W < 2 || H < 2) {
					throw(ofxRulr::Exception("Grid size too small for export (need at least 2x2)"));
				}

				// Flatten lens mirror mesh (pos, nrm, uv, idx)
				std::vector<float> lensPos;    lensPos.reserve(W * H * 3);
				std::vector<float> lensNrm;    lensNrm.reserve(W * H * 3);
				std::vector<float> lensUV;     lensUV.reserve(W * H * 2);
				std::vector<uint32_t> lensIdx; lensIdx.reserve((W - 1) * (H - 1) * 6);

				auto idx2 = [&](std::size_t i, std::size_t j) -> uint32_t {
					return (uint32_t)(i + j * W);
					};

				for (std::size_t j = 0; j < H; ++j) {
					for (std::size_t i = 0; i < W; ++i) {
						const glm::vec3 p = positionsGrid.at(i, j);
						const glm::vec3 n = normalsGrid.at(i, j);
						const float u = (W > 1) ? float(i) / float(W - 1) : 0.0f;
						const float v = (H > 1) ? float(j) / float(H - 1) : 0.0f;
						lensPos.push_back(p.x); lensPos.push_back(p.y); lensPos.push_back(p.z);
						lensNrm.push_back(n.x); lensNrm.push_back(n.y); lensNrm.push_back(n.z);
						lensUV.push_back(u);    lensUV.push_back(v);
					}
				}
				for (std::size_t j = 0; j + 1 < H; ++j) {
					for (std::size_t i = 0; i + 1 < W; ++i) {
						const uint32_t i0 = idx2(i, j);
						const uint32_t i1 = idx2(i + 1, j);
						const uint32_t i2 = idx2(i, j + 1);
						const uint32_t i3 = idx2(i + 1, j + 1);
						lensIdx.push_back(i0); lensIdx.push_back(i1); lensIdx.push_back(i2);
						lensIdx.push_back(i1); lensIdx.push_back(i3); lensIdx.push_back(i2);
					}
				}

				// Flatten screen mesh (pos, uv, idx)
				std::vector<float> scrPos;  scrPos.reserve(W * H * 3);
				std::vector<float> scrUV;   scrUV.reserve(W * H * 2);
				std::vector<uint32_t> scrIdx; scrIdx.reserve((W - 1) * (H - 1) * 6);

				for (std::size_t j = 0; j < H; ++j) {
					for (std::size_t i = 0; i < W; ++i) {
						const glm::vec3 p = targetGrid.at(i, j);
						const float u = (W > 1) ? float(i) / float(W - 1) : 0.0f;
						const float v = (H > 1) ? float(j) / float(H - 1) : 0.0f;
						scrPos.push_back(p.x); scrPos.push_back(p.y); scrPos.push_back(p.z);
						scrUV.push_back(u);    scrUV.push_back(v);
					}
				}
				for (std::size_t j = 0; j + 1 < H; ++j) {
					for (std::size_t i = 0; i + 1 < W; ++i) {
						const uint32_t i0 = idx2(i, j);
						const uint32_t i1 = idx2(i + 1, j);
						const uint32_t i2 = idx2(i, j + 1);
						const uint32_t i3 = idx2(i + 1, j + 1);
						scrIdx.push_back(i0); scrIdx.push_back(i1); scrIdx.push_back(i2);
						scrIdx.push_back(i1); scrIdx.push_back(i3); scrIdx.push_back(i2);
					}
				}

				// Camera parameters
				const int camW = viewWS.getWidth();
				const int camH = viewWS.getHeight();
				if (camW <= 0 || camH <= 0) {
					throw(ofxRulr::Exception("View camera width/height invalid"));
				}
				const glm::mat4 Pm = viewWS.getProjectionMatrix();
				float fy = Pm[1][1];
				if (std::abs(fy) < 1e-6f) {
					throw(ofxRulr::Exception("Projection matrix invalid (fy ~ 0)"));
				}
				const float yfov = 2.0f * atanf(1.0f / std::abs(fy));
				const float aspect = (float)camW / (float)camH;
				const float znear = viewWS.getNearClip();
				float zfar = viewWS.getFarClip();
				if (!(znear > 0.0f)) {
					throw(ofxRulr::Exception("View camera near clip invalid"));
				}
				if (viewWS.isProjectionMatrixInfinite()) {
					if (!(zfar > znear)) zfar = 1000.0f;
				}
				else if (!(zfar > znear)) {
					throw(ofxRulr::Exception("View camera far clip must be greater than near clip"));
				}

				glm::mat4 camMatrixWorld = glm::inverse(V);

				// --- Fix orientation for Blender (camera looks +Z instead of -Z)
				const glm::mat4 flipZ = glm::scale(glm::mat4(1.0f), glm::vec3(1, 1, -1));
				camMatrixWorld = camMatrixWorld * flipZ;

				// Build GLB
				using nlohmann::json;
				std::vector<uint8_t> bin; bin.reserve(1 << 20);
				auto pad4 = [](size_t n) { return (n + 3) & ~size_t(3); };
				auto appendPad = [&](size_t newSize) { while (bin.size() < newSize) bin.push_back(0); };
				auto appendData = [&](const void* src, size_t n) -> uint32_t {
					const uint32_t off = (uint32_t)pad4(bin.size());
					appendPad(off);
					const uint8_t* p = static_cast<const uint8_t*>(src);
					bin.insert(bin.end(), p, p + n);
					appendPad(pad4(bin.size()));
					return off;
					};

				struct BufferView { int buffer = 0; uint32_t offset = 0; uint32_t length = 0; uint32_t stride = 0; };
				std::vector<BufferView> bvs;

				auto addBV = [&](const void* data, size_t bytes, uint32_t stride = 0) {
					BufferView bv; bv.buffer = 0;
					bv.offset = appendData(data, bytes);
					bv.length = (uint32_t)bytes;
					bv.stride = stride;
					bvs.push_back(bv);
					};

				addBV(lensPos.data(), lensPos.size() * sizeof(float), 3 * sizeof(float));
				addBV(lensNrm.data(), lensNrm.size() * sizeof(float), 3 * sizeof(float));
				addBV(lensUV.data(), lensUV.size() * sizeof(float), 2 * sizeof(float));
				addBV(lensIdx.data(), lensIdx.size() * sizeof(uint32_t));
				addBV(scrPos.data(), scrPos.size() * sizeof(float), 3 * sizeof(float));
				addBV(scrUV.data(), scrUV.size() * sizeof(float), 2 * sizeof(float));
				addBV(scrIdx.data(), scrIdx.size() * sizeof(uint32_t));

				struct Accessor { int bufferView; int componentType; const char* type; uint32_t count; };
				std::vector<Accessor> accs = {
					{0, 5126, "VEC3", (uint32_t)(lensPos.size() / 3)},
					{1, 5126, "VEC3", (uint32_t)(lensNrm.size() / 3)},
					{2, 5126, "VEC2", (uint32_t)(lensUV.size() / 2)},
					{3, 5125, "SCALAR", (uint32_t)lensIdx.size()},
					{4, 5126, "VEC3", (uint32_t)(scrPos.size() / 3)},
					{5, 5126, "VEC2", (uint32_t)(scrUV.size() / 2)},
					{6, 5125, "SCALAR", (uint32_t)scrIdx.size()}
				};

				nlohmann::json j;
				j["asset"] = { {"version","2.0"}, {"generator","ofxRulr::SimulateAndOptimise"} };
				j["buffers"] = nlohmann::json::array({ { {"byteLength",(uint32_t)pad4(bin.size())} } });
				j["bufferViews"] = nlohmann::json::array();
				for (auto& bv : bvs) {
					nlohmann::json jb;
					jb["buffer"] = 0;
					jb["byteOffset"] = bv.offset;
					jb["byteLength"] = bv.length;
					if (bv.stride) jb["byteStride"] = bv.stride;
					j["bufferViews"].push_back(jb);
				}
				j["accessors"] = nlohmann::json::array();
				for (size_t i = 0; i < accs.size(); ++i) {
					const auto& a = accs[i];
					nlohmann::json ja;
					ja["bufferView"] = (int)i;
					ja["componentType"] = a.componentType;
					ja["count"] = a.count;
					ja["type"] = a.type;
					j["accessors"].push_back(ja);
				}

				nlohmann::json primLens;
				primLens["attributes"] = { {"POSITION", 0}, {"NORMAL", 1}, {"TEXCOORD_0", 2} };
				primLens["indices"] = 3;

				nlohmann::json primScreen;
				primScreen["attributes"] = { {"POSITION", 4}, {"TEXCOORD_0", 5} };
				primScreen["indices"] = 6;

				j["meshes"] = nlohmann::json::array({
					{ {"name","LensMirror"},   {"primitives", nlohmann::json::array({ primLens })} },
					{ {"name","ScreenTarget"}, {"primitives", nlohmann::json::array({ primScreen })} }
					});

				j["cameras"] = nlohmann::json::array({
					{
						{"name","ViewCamera"},
						{"type","perspective"},
						{"perspective",{
							{"yfov", yfov},
							{"znear", znear},
							{"zfar",  zfar},
							{"aspectRatio", aspect}
						}}
					}
					});

				auto toGltfMatrix = [](const glm::mat4& M) {
					std::array<float, 16> a{};
					int k = 0;
					for (int c = 0; c < 4; ++c)
						for (int r = 0; r < 4; ++r)
							a[k++] = M[c][r];
					return a;
					};

				j["nodes"] = nlohmann::json::array({
					{ {"name","LensMirrorNode"},   {"mesh", 0} },
					{ {"name","ScreenTargetNode"}, {"mesh", 1} },
					{ {"name","ViewCameraNode"},   {"camera", 0}, {"matrix", toGltfMatrix(camMatrixWorld)} }
					});

				j["scenes"] = nlohmann::json::array({ { {"nodes", nlohmann::json::array({0,1,2})} } });
				j["scene"] = 0;

				// Pack GLB
				auto pad4u = [&](uint32_t n) { return (n + 3u) & ~3u; };
				std::string jsonStr = j.dump();
				uint32_t jsonPadded = pad4u((uint32_t)jsonStr.size());
				std::vector<uint8_t> jsonChunk(jsonStr.begin(), jsonStr.end());
				jsonChunk.resize(jsonPadded, 0x20);
				uint32_t binPadded = pad4u((uint32_t)bin.size());
				bin.resize(binPadded, 0);

				struct GLBHeader { uint32_t magic; uint32_t version; uint32_t length; };
				struct GLBChunk { uint32_t length; uint32_t type; };
				const uint32_t JSON_TYPE = 0x4E4F534A; // 'JSON'
				const uint32_t BIN_TYPE = 0x004E4942; // 'BIN\0'

				GLBHeader header;
				header.magic = 0x46546C67; // 'glTF'
				header.version = 2;
				header.length = sizeof(GLBHeader) + sizeof(GLBChunk) + jsonPadded + sizeof(GLBChunk) + binPadded;

				GLBChunk jsonHdr{ jsonPadded, JSON_TYPE };
				GLBChunk binHdr{ binPadded,  BIN_TYPE };

				ofBuffer fileBuf;
				fileBuf.append((const char*)&header, sizeof(header));
				fileBuf.append((const char*)&jsonHdr, sizeof(jsonHdr));
				fileBuf.append((const char*)jsonChunk.data(), jsonChunk.size());
				fileBuf.append((const char*)&binHdr, sizeof(binHdr));
				fileBuf.append((const char*)bin.data(), bin.size());

				if (!ofBufferToFile(outputPath, fileBuf, true)) {
					throw(ofxRulr::Exception("Failed to write GLB to " + outputPath));
				}

				ofLogNotice() << "Exported GLB: " << outputPath;
			}
		}
	}
}