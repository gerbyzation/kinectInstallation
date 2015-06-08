#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	kinect.open();
	
	// INITIALIZE THE REQUIRED STREAMS
	kinect.initColorSource();
	kinect.initBodySource();
	kinect.initDepthSource();
	
	// WE NEED TO GRAB A REFERENCE FOR THE COORDINATE 
	// MAPPER FROM THE KINECT DEVICE. THIS WILL BE
	// USED TO UPDATE THE DEPTH TO COLOR MAPPINGS EACH FRAME
	kinect.getSensor()->get_CoordinateMapper(&m_pCoordinateMapper);

	ofSetWindowShape(1920,1080);

}

//--------------------------------------------------------------
void ofApp::update(){

	// UPDATE ALL INITIAZIZED STREAMS
	kinect.update();

	// WE NEED TO GRAB THE PIXEL VALUES FROM THE DEPTH STREAM FOR THE CURRENT FRAME...
	unsigned short * depthPixels = this->kinect.getDepthSource()->getPixels();

	// THEN FEED THEM INTO THE COORDINATE MAPPER WITH OUR ARRAY OF DEPTHSPACEPOINTS.
     HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(
		512 * 424, 
		(const UINT16*)depthPixels, 
		1920 * 1080, 
		m_pDepthCoordinates);

}


//--------------------------------------------------------------
void ofApp::draw(){

	ofBackground(75,95,115);
	
	ofSetColor(255);
	
	ofPushMatrix();
		// WE ARE ONLY DRAWING THE 2D IMAGE WHICH HAS A DEFAULT RESOLUTION OF 1920x1080
		this->kinect.getColorSource()->draw(0,0,1920,1080);
		// USE THE COORDINATE MAPPER ON THE 3D JOINTS TO MAP THEM TO THE 2D COLOR STREAM
		mapJoints2D();
	ofPopMatrix();


}


void ofApp::mapJoints2D()
{
	ofPushStyle();
	// WILL SET A SEMI-TRANSPARENT PURPLE FOR THE JOINTS
	ofSetColor(255,0,255,140);

	ofVec2f pos;
	
	// THERE IS A MAXIMUM OF 6 BODIES TRACKED BY KINECT
	for(int i=0;i<6;i++){
		// IF THE BODY IS BEING TRACKED...
		if(this->kinect.getBodySource()->getBodies()[i].tracked ){

			auto b = this->kinect.getBodySource()->getBodies()[i];

			// ITERATE THROUGH ALL JOINTS IN THE TRACKED BODY...
			 for ( std::map<JointType, ofxKFW2::Data::Joint>::iterator it = b.joints.begin(); it != b.joints.end(); ++it)
			 {
				 if ( it->second.getTrackingState() == TrackingState_Tracked  ) 
				 {
					 // TO GRAB THE JOINT'S 3D POSITION, YOU SIMPLE ACCESS IT LIKE THIS: it->second.getPosition()
					 // BUT IF WE WANT TO CALCULATE WHERE IN THE 2D COLOR STREAM THE 3D JOINTS ARE,
					 // YOU NEED TO TO USE THE COORDINATE MAPPER WE HAVE BEEN UPDATING LIKE SO:
					 pos = it->second.getProjected(m_pCoordinateMapper);

					 // 'pos' IS NOW A VEC2F THAT HAS MAPPED THE 3D POSITION OF THE JOINT IN DEPTH SPACE 
					 // TO IT'S MATCHING COORDINATES IN THE 2D COLOR SPACE OF THE COLOR STREAM
					 ofCircle(pos.x, pos.y, 10);
				 }
			 }
		}
	}

	ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
