#include "pch_RulrCore.h"
#include "LinkHost.h"

#include "ofPolyline.h"

namespace ofxRulr {
	namespace Graph {
		namespace Editor {
#pragma mark LinkHost
			//---------
			LinkHost::LinkHost() {
				this->onDraw += [this](ofxCvGui::DrawArguments & args) {
					this->draw(args);
				};
				this->onUpdate += [this](ofxCvGui::UpdateArguments & args) {
					this->update();
				};
			}

			//---------
			bool LinkHost::isValid() const {
				return !this->sourceNode.expired() && !this->targetNode.expired() && !this->targetPin.expired();
			}

			//---------
			void LinkHost::update() {
				if (this->getSourcePinPosition() != this->cachedStart
					|| this->getTargetPinPosition() != this->cachedEnd) {
					this->needsRebuild = true;
				}
				if (this->needsRebuild) {
					this->rebuild();
				}
			}
			
			//---------
			ofColor LinkHost::getColor() const {
				auto sourceNode = this->sourceNode.lock();
				if (sourceNode) {
					return sourceNode->getNodeInstance()->getColor();
				}
				else {
					return this->targetPin.lock()->getNodeColor();
				}
			}

			glm::vec3 v3(const glm::vec2& v2) {
				return { v2.x, v2.y, 0.0f };
			}

			//---------
			void LinkHost::rebuild() {
				auto start = this->getSourcePinPosition();
				auto end = this->getTargetPinPosition();

				// Update bounds
				{
					ofRectangle bounds(this->getSourcePinPosition(), this->getTargetPinPosition());
					bounds.x -= 10;
					bounds.y -= 10;
					bounds.width += 20;
					bounds.height += 20;
					this->setBounds(bounds);
				}

				// Create the polyline
				{
					const auto wireRigidity = v3(glm::vec2(100, 0));
					this->polyline.clear();
					this->polyline.addVertex(v3(start));
					this->polyline.bezierTo(v3(start + wireRigidity), v3(end - wireRigidity), v3(end), 30);
				}

				// Upload to vbo mesh
				{
					this->vbo.clear();
					this->vbo.setVertexData(this->polyline.getVertices().data(), this->polyline.size(), GL_DYNAMIC_DRAW);
				}

				this->cachedStart = start;
				this->cachedEnd = end;
				this->needsRebuild = false;
			}

			//---------
			void LinkHost::draw(ofxCvGui::DrawArguments & args) {
				try {
					//move from local to patch space
					ofPushMatrix();
					{
						ofTranslate(-this->getBoundsInParent().getTopLeft());

						ofPushStyle();
						{

							//shadow
							ofPushMatrix();
							{
								ofTranslate(5.0f, 5.0f);
								ofSetLineWidth(2.0f);
								ofSetColor(0, 100);
								this->vbo.draw(GL_LINE_STRIP, 0, this->polyline.size());
							}
							ofPopMatrix();

							//outline
							ofSetLineWidth(3.0f);
							ofSetColor(0);
							this->polyline.draw();

							//line
							ofSetLineWidth(2.0f);
							ofSetColor(this->getColor());
							this->vbo.draw(GL_LINE_STRIP, 0, this->polyline.size());
						}
						ofPopStyle();
					}
					ofPopMatrix();
				}
				catch (ofxRulr::Exception e) {
					ofLogError("ofxRulr::Graph::Editor::LinkHost::draw()") << e.what();
				}
			}

			//---------
			glm::vec2 LinkHost::getSourcePinPosition() const {
				auto sourceNode = this->sourceNode.lock();
				if (!sourceNode) {
					throw(ofxRulr::Exception("LinkHost has no valid source node"));;
				}
				return sourceNode->getOutputPinPositionGlobal();
			}

			//---------
			glm::vec2 LinkHost::getTargetPinPosition() const {
				auto targetNode = this->targetNode.lock();
				auto targetPin = this->targetPin.lock();
				if (!targetNode || !targetPin) {
					throw(ofxRulr::Exception("LinkHost has no valid target node or pin"));;
				}
				return targetNode->getInputPinPosition(targetPin);
			}

#pragma mark TemporaryLinkHost
			//----------
			TemporaryLinkHost::TemporaryLinkHost(shared_ptr<NodeHost> targetNode, shared_ptr<AbstractPin> targetPin) {
				this->targetNode = targetNode;
				this->targetPin = targetPin;
			}

			//---------
			void TemporaryLinkHost::setCursorPosition(const glm::vec2 & cursorPosition) {
				this->cursorPosition = cursorPosition;
				auto targetNode = this->targetNode.lock();
				if (targetNode) {
					this->setBounds(ofRectangle(cursorPosition, targetNode->getOutputPinPositionGlobal()));
				}
			}

			//---------
			void TemporaryLinkHost::setSource(shared_ptr<NodeHost> source) {
				if (source){
					auto targetPin = this->targetPin.lock();
					if (targetPin->checkSupports(source->getNodeInstance())) {
						this->sourceNode = source;
					}
				} else {
					this->sourceNode.reset();
				}
			}

			//---------
			bool TemporaryLinkHost::flushConnection() {
				if (!this->isValid()) {
					//can't make the connection
					return false;
				}
				auto sourceNode = this->sourceNode.lock();
				auto targetPin = this->targetPin.lock();
				targetPin->connect(sourceNode->getNodeInstance());
				return true;
			}

			//---------
			glm::vec2 TemporaryLinkHost::getSourcePinPosition() const {
				if (!this->sourceNode.expired()) {
					auto sourceNode = this->sourceNode.lock();
					return sourceNode->getOutputPinPositionGlobal();
				}
				else {
					return this->cursorPosition;
				}
			}

#pragma mark ObservedLinkHost
			//---------
			ObservedLinkHost::ObservedLinkHost(shared_ptr<NodeHost> sourceNode, shared_ptr<NodeHost> targetNode, shared_ptr<AbstractPin> targetPin) {
				this->sourceNode = sourceNode;
				this->targetNode = targetNode;
				this->targetPin = targetPin;
			}
		}
	}
}
//---------
