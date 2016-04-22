#include "pch_MultiTrack.h"

#include "Utils.h"

#include "World.h"
#include "ofxRulr/Nodes/Data/Channels/Database.h"

using namespace ofxCvGui;

namespace ofxRulr {
	namespace Nodes {
		namespace MultiTrack {
			//----------
			World::World() {
				RULR_NODE_INIT_LISTENER;
			}

			//----------
			string World::getTypeName() const {
				return "MultiTrack::World";
			}

			//----------
			void World::init() {
				RULR_NODE_DRAW_WORLD_LISTENER;
				RULR_NODE_UPDATE_LISTENER;
				RULR_NODE_INSPECTOR_LISTENER;
				//RULR_NODE_SERIALIZATION_LISTENERS;

				auto databasePin = this->addInput<Data::Channels::Database>();

				databasePin->onNewConnection += [this](shared_ptr<Data::Channels::Database> database) {
					database->onPopulateData.addListener([this](Data::Channels::Channel & rootChannel) {
						this->populateDatabase(rootChannel);
					}, this);
				};
				databasePin->onDeleteConnection += [this](shared_ptr<Data::Channels::Database> database) {
					if (database) {
						database->onPopulateData.removeListeners(this);
					}
				};

				for (size_t i = 0; i < NumSubscribers; i++) {
					auto subscriberPin = this->addInput<Subscriber>("Subscriber " + ofToString(i + 1));
					subscriberPin->onNewConnection += [this, i](shared_ptr<Subscriber> subscriber) {
						this->subscribers[i] = subscriber;
					};
					subscriberPin->onDeleteConnection += [this, i](shared_ptr<Subscriber> subscriber) {
						this->subscribers.erase(i);
					};
				}

				this->manageParameters(this->parameters);
			}

			//----------
			void World::update() {
				//perform skeleton fusion
				if(this->parameters.fusion.enabled) {
					this->performFusion();
				}
			}

			//----------
			void World::drawWorld() {
				ofPushStyle();
				{
					for (const auto & combinedBody : this->combinedBodies) {
						ofColor color(200, 100, 100);
						color.setHueAngle((combinedBody.first * 30) % 360);

						//draw combined body
						ofSetColor(color);
						combinedBody.second.mergedBody.drawWorld();

						//draw body perimeter
						if (this->parameters.fusion.drawBodyPerimeter) {
							ofPushStyle();
							{
								auto dimColor = color;
								dimColor.setBrightness(50);
								for (auto & joint : combinedBody.second.mergedBody.joints) {
									ofPushMatrix();
									{
										ofTranslate(joint.second.getPosition() * ofVec3f(1, 0, 1));
										ofRotate(90, 1, 0, 0);
										ofDrawCircle(ofVec3f(), this->parameters.fusion.mergeDistanceThreshold);
									}
									ofPopMatrix();
								}
							}
							ofPopStyle();
						}

						//draw original bodies and line to source kinect
						color.setBrightness(20);
						for (auto originalBody : combinedBody.second.originalBodiesWorldSpace) {
							//original body
							originalBody.second.drawWorld();


							//line to kinect it came from
							auto findSubscriber = this->subscribers.find(originalBody.first);
							if (findSubscriber != this->subscribers.end()) {
								auto subscriber = findSubscriber->second.lock();
								if (subscriber) {
									auto findHead = originalBody.second.joints.find(JointType::JointType_Head);
									if (findHead != originalBody.second.joints.end()) {
										ofDrawLine(findHead->second.getPosition(), subscriber->getPosition());
									}
								}
							}
						}
					}
				}
				ofPopStyle();
			}

			//----------
			void World::populateInspector(ofxCvGui::InspectArguments & args) {
				args.inspector->addLiveValue<size_t>("Connected subscribers", [this]() {
					return this->subscribers.size();
				});
			}


			//----------
			map<size_t, weak_ptr<Subscriber>> & World::getSubscribers() {
				return this->subscribers;
			}

			//----------
			void World::performFusion() {
				auto worldBodiesUnmerged = this->getWorldBodiesUnmerged();
				this->combinedBodies = this->combineWorldBodies(worldBodiesUnmerged);
			}

			//----------
			World::WorldBodiesUnmerged World::getWorldBodiesUnmerged() const {
				//This function accumulates all the bodies from the whole network of sensors.
				//And transforms them using the rigid body transforms from the Subscriber nodes
				WorldBodiesUnmerged worldBodiesUnmerged;

				for (auto subscriberIt : this->subscribers) {
					auto subscriberNode = subscriberIt.second.lock();
					if (subscriberNode) {
						auto subscriber = subscriberNode->getSubscriber();
						if (subscriber) {
							const auto & bodiesInCameraSpace = subscriber->getFrame().getBodies();
							auto & BodiesInWorldSpace = worldBodiesUnmerged[subscriberIt.first];

							for (const auto & body : bodiesInCameraSpace) {
								BodiesInWorldSpace.push_back(body * subscriberNode->getTransform());
							}
						}
					}
				}

				return worldBodiesUnmerged;
			}

			//----------
			CombinedBodySet World::combineWorldBodies(WorldBodiesUnmerged worldBodiesUnmerged) const {
				const auto & mergeDistanceThreshold = this->parameters.fusion.mergeDistanceThreshold.get();
				Bodies mergedWorldBodies;

				auto previousFrameCombinedBodies = this->combinedBodies;
				CombinedBodySet newCombinedBodies;

				// 1. Update any bodies which had been seen previously
				// 2. Add any bodies which weren't seen previously (adding them to existing when distance is low)
				// 3. Calculate the merged position
				// as bodies are handled, we remove them from our WorldBodiesUnmerged set

				//--
				//Update bodies seen previously
				//--
				//
				// 1. Go through existing bodies
				// 2. Go through the sources of that body
				// 3. See if it still has this body index
				for (const auto & oldCombinedBodyIterator : previousFrameCombinedBodies) {
					const auto & bodyIndex = oldCombinedBodyIterator.first;

					for (const auto & originalBodiesIterator : oldCombinedBodyIterator.second.originalBodiesWorldSpace) {
						const auto & originalBodyIndex = originalBodiesIterator.second.bodyId;
						const auto & subscriberID = originalBodiesIterator.first;

						auto findSubscriber = worldBodiesUnmerged.find(subscriberID);
						if (findSubscriber != worldBodiesUnmerged.end()) {
							auto & subscriber = findSubscriber->second;
							//find if we have matching body index
							for (auto subscriberBodyIterator = subscriber.begin(); subscriberBodyIterator != subscriber.end(); ) {
								if (subscriberBodyIterator->bodyId == originalBodyIndex) {
									//we've found a body which we'd tagged before
									auto & combinedBody = newCombinedBodies[oldCombinedBodyIterator.first];

									//assign it the new body
									combinedBody.originalBodiesWorldSpace[subscriberID] = *subscriberBodyIterator;

									//remove it from the unassigned set
									subscriberBodyIterator = subscriber.erase(subscriberBodyIterator);
								}
								else {
									subscriberBodyIterator++;
								}
							}
						}
					}
				}
				//
				//--



				//--
				//Add subscriber body to existing combined bodies
				//--
				//
				for (auto & subscriber : worldBodiesUnmerged) {
					auto & bodies = subscriber.second;
					for (auto bodyIterator = bodies.begin(); bodyIterator != bodies.end(); ) {
						// now we have a body belonging to a subscriber

						bool addedToExistingBody = false;

						//check this body against any built bodies so far (this is often empty)
						for (auto & combinedBody : newCombinedBodies) {
							//find the mean of the combinedBody
							auto meanOfExistingBodies = mean(combinedBody.second.originalBodiesWorldSpace);

							//if the distance between this body and that body is < threshold, add it to the combined body
							if (meanDistance(meanOfExistingBodies, *bodyIterator) < this->parameters.fusion.mergeDistanceThreshold) {
								combinedBody.second.originalBodiesWorldSpace[subscriber.first] = *bodyIterator;
								addedToExistingBody = true;
								break;
							}
						}

						if (!addedToExistingBody) {
							static BodyIndex bodyIndex = 0;
							auto & combinedBody = newCombinedBodies[bodyIndex];
							combinedBody.originalBodiesWorldSpace[subscriber.first] = *bodyIterator;
							bodyIndex++;
						}

						bodyIterator = bodies.erase(bodyIterator);
					}
				}
				//
				//--



				// now newCombinedBodies is populated



				//--
				//Calculate the merged position
				//--
				//
				for (auto & newCombinedBody : newCombinedBodies) {
					newCombinedBody.second.mergedBody = mean(newCombinedBody.second.originalBodiesWorldSpace);
				}
				//
				//--

				return newCombinedBodies;
			}

			//----------
			void World::populateDatabase(Data::Channels::Channel & rootChannel) {
				auto & combined = rootChannel["combined"];
				{
					auto & bodies = combined["bodies"];
					bodies["count"] = (int) this->combinedBodies.size();

					vector<int> indices;
					for (auto body : this->combinedBodies) {
						indices.push_back(body.first);
					}
					bodies["indices"] = indices;

					for (auto & combinedBody : this->combinedBodies) {
						auto & bodyChannel = bodies["body"][ofToString(combinedBody.first)];

						auto & body = combinedBody.second.mergedBody;
						bodyChannel["tracked"] = (int) body.tracked;

						if (!body.tracked) {
							bodyChannel.removeSubChannel("centroid");
							bodyChannel.removeSubChannel("joints");
						}
						else {
							//We use the base of the spine as the centroid
							auto findSpineBase = body.joints.find(JointType::JointType_SpineBase);
							if (findSpineBase != body.joints.end()) {
								bodyChannel["centroid"] = findSpineBase->second.getPosition();
							}
							else {
								bodyChannel.removeSubChannel("centroid");
							}

							auto & jointChannel = bodyChannel["joints"];
							jointChannel["count"] = body.joints.size();

							for (auto & joint : body.joints) {
								auto jointName = ofxKinectForWindows2::toString(joint.first);
								auto & jointChannel = bodyChannel[jointName];
								jointChannel["position"] = joint.second.getPosition();
								jointChannel["orientation"] = joint.second.getOrientation().asVec4();
								jointChannel["trackingState"] = joint.second.getTrackingState();
							}
						}
					}
 				}
			}
		}
	}
}
