/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/constraint/WeldJointConstraint.h"
#include <cmath>

#include <iostream>
 using namespace std;

Controller::Controller(dart::dynamics::Skeleton* bar, dart::dynamics::Skeleton* _skel, dart::constraint::ConstraintSolver* _constrSolver, double _t) {
  mSkel = _skel;
  mConstraintSolver = _constrSolver;
  mTimestep = _t;
  mBar = bar;
  Eigen::Vector3d posBar = mBar->getWorldCOM();
  barXorg=posBar(0);
  barYorg=posBar(1);

  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mTorques.resize(nDof);
  mDefaultPose.resize(nDof);
  mDesiredDofs.resize(nDof);
  
  // Set default pose as the initial pose when the controller is instantiated
  mDefaultPose = mSkel->getPositions();
  mDesiredDofs = mDefaultPose;
  
  mTorques.setZero();

  mTimes = 0;
  isSwing = true;
  isFirst = true;

  isFootGrab = false;


  // Using SPD results in simple spring coefficients
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }
  for (int i = 6; i < 35; i++)
    mKp(i, i) = 400.0;//400
  for (int i = 6; i < 35; i++)
    mKd(i, i) = 40.0;//40

  // Make shoulders and elbows loose
  for (int i = 27; i < 35; i++) {
    mKp(i, i) = 20.0;
    mKd(i, i) = 2.0;
  }

  // Make wrists even looser
  for (int i = 35; i < 39; i++) {
    mKp(i, i) = 1.0;
    mKd(i, i) = 0.1;
  }

  mPreOffset = 0.0;
  mLeftHandHold = NULL;
  mRightHandHold = NULL;
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;

  mLeftFootHold = NULL;
  mRightFootHold = NULL;
  mLeftFootContact = NULL;
  mRightFootContact = NULL;

  mTimer = 300;//300
  mState = "STAND";//STAND
  mArch = 0;
}

Controller::~Controller() {
}


Eigen::VectorXd Controller::getTorques() {
  return mTorques;
}

double Controller::getTorque(int _index) {
  return mTorques[_index];
}

void Controller::setDesiredDof(int _index, double _val) {
  mDesiredDofs[_index] = _val;
}

void Controller::computeTorques(int _currentFrame) {
  mCurrentFrame = _currentFrame;
  mTorques.setZero();
  if (mState == "STAND") {
    stand();
  } else if (mState == "CROUCH") {
    crouch();
  } else if (mState == "JUMP") {
    jump();
  } else if (mState == "REACH") {
    reach();
  } else if (mState == "GRAB") {
    grab();
  } else if (mState == "RELEASE") {
    release();
  } else if (mState == "SWING") {
    swing();
  } else if (mState == "SWINGCROUCH"){
    swingCrouch();
  } else if (mState =="GRABRIGHT"){
    grabRight();
  } else if (mState == "REACHRIGHT"){
    reachRight();
  } else if (mState == "GRABFOOT"){
    grabFoot();
  } else if (mState == "RELEASEFOOT"){
    releaseFoot();
  } else if (mState == "SWINGFOOT"){
    swingFoot();
  } else if (mState =="SWINGFOOTSTRAIGHT"){
    swingFootStraight();
  }

   else {
    std::cout << "Illegal state: " << mState << std::endl;
  }

  // Just to make sure no illegal torque is used. Do not remove this.
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
}

void Controller::checkContactState() {
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;

  mLeftFootContact = NULL;
  mRightFootContact = NULL;

  dart::collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  //cout<<"contact number: "<<nContacts<<endl;
  for (int i = 0; i < nContacts; i++) {
    dart::dynamics::BodyNode* body1 = cd->getContact(i).bodyNode1;
    dart::dynamics::BodyNode* body2 = cd->getContact(i).bodyNode2;
    if (body1 == mSkel->getBodyNode("h_heel_left") || body1 == mSkel->getBodyNode("h_heel_left")
        || body1 == mSkel->getBodyNode("h_heel_right") || body1 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body2;
    if (body2 == mSkel->getBodyNode("h_heel_left") || body2 == mSkel->getBodyNode("h_heel_left")
        || body2 == mSkel->getBodyNode("h_heel_right") || body2 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body1;

    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_toe_left"))
      mLeftFootContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_toe_left"))
      mLeftFootContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_toe_right"))
      mRightFootContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_toe_right"))
      mRightFootContact = body1;
  }
}

void Controller::stand() {
  // Change to default standing pose
  mDesiredDofs = mDefaultPose;
  stablePD();
  ankleStrategy();
  mTimer--;

  // Switch to crouch if time is up
  if (mTimer == 0) {
    mState = "CROUCH";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "STAND -> CROUCH" << std::endl;
  }
}

void Controller::crouch() {
  // Change to crouching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.7;
  mDesiredDofs[9] = 0.7;
  mDesiredDofs[14] = -1.1;
  mDesiredDofs[15] = -1.1;
  mDesiredDofs[17] = 0.6;
  mDesiredDofs[19] = 0.6;
  mDesiredDofs[13] = -0.2;
 

  // After a while, lean forward
  if (mTimer < 200) {
    mDesiredDofs[17] = 1.0;
    mDesiredDofs[19] = 1.0;
  }

  stablePD();
  ankleStrategy();
  mTimer--;

  if (mTimer == 0) {
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "CROUCH -> JUMP" << std::endl;

  }
}

void Controller::jump() {
  // Change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.9;
  mDesiredDofs[9] = 0.9;
  mDesiredDofs[14] = -0.9;

  mDesiredDofs[15] = -0.9;
  mDesiredDofs[17] = -0.9;
  mDesiredDofs[19] = -0.9;
  mDesiredDofs[27] = 0.3;
  mDesiredDofs[28] = -1.0;
  mDesiredDofs[30] = 0.3;
  mDesiredDofs[31] = 1.0;
  mDesiredDofs[33] = 0.5;
  mDesiredDofs[34] = 0.5;

  stablePD();

  // Use Jacobian transpose to compute pushing torques
  
  float h = 1;
  Eigen::Vector3d posBar = mBar->getWorldCOM();

  Eigen:: Vector3d com = mSkel->getWorldCOM();
 
  com = posBar - com;
  com.normalize();
  
  Eigen::Vector3d vf(-1800,-4000, 0.0);//-1100.0  , -2600 , 0.0);
  Eigen::Vector3d offset(0.05, -0.02, 0.0);

  virtualForce(vf, mSkel->getBodyNode("h_heel_left"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_heel_right"), offset);

  checkContactState();
  if (mFootContact == NULL) {
    mState = "REACH";
    std::cout << mCurrentFrame << ": " << "JUMP -> REACH" << std::endl;
  }
}

void Controller::reach() {
  // Change to reaching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[33] = 0.4;
  mDesiredDofs[34] = 0.4;
  stablePD();

  checkContactState();
  if (mFootContact) { // If feet are in contact again, go back to JUMP and continue to push
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "REACH -> JUMP" << std::endl;
  } else if (mLeftHandContact || mRightHandContact) {
    mState = "GRAB";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "REACH -> GRAB" << std::endl;
  } else {
    mState = "REACH";
  }
}

void Controller::grab() {
  leftHandGrab();
  rightHandGrab();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[33] = 0.4;
  mDesiredDofs[34] = 0.4;
  stablePD();
  mTimer--;

  if (mTimer == 0) {
    mState = "SWING";
    //mTimer = 10000;//200
    std::cout << mCurrentFrame << ": " << "GRAB -> SWING" << std::endl;
  }
}  
void Controller::grabRight() {
  // leftHandGrab();
   rightHandGrab();
   leftHandRelease();

   dart::dynamics::BodyNode* _bodyNode = mSkel->getBodyNode("h_hand_left");
  Eigen::Vector3d _target = mBar->getWorldCOM();
  // _target(0) += 0.01;
  mDesiredDofs= ik(_bodyNode,  _target);

  
  mDesiredDofs[9] = 2.2;
  mDesiredDofs[15] = -2.5;
  mDesiredDofs[6] = 1.7;
  mDesiredDofs[14] = -2.5;
 
  mDesiredDofs[30] = 0.7;//right shoulder
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[34] = 3;
  stablePD();
  checkContactState();
 if (mLeftHandContact == mBar->getBodyNode(0)){
    leftHandGrab();
    mState="SWING";
    std::cout << mCurrentFrame << ": " << "GRABRIGHT -> SWING" << std::endl;


 }
  
}  
void Controller::reachRight() {
  rightHandRelease();
  if(mRightHandHold!= NULL)
    cout<<"right hand doesn't release"<<endl;
  dart::dynamics::BodyNode* _bodyNode = mSkel->getBodyNode("h_hand_right");
  Eigen::Vector3d _target = mBar->getWorldCOM();
  // _target(0) += 0.01;
  mDesiredDofs= ik(_bodyNode,  _target);

  mDesiredDofs[6] = 2.2;
  mDesiredDofs[14] = -2.5;
  mDesiredDofs[9] = 1.7;
  mDesiredDofs[15] = -2.5;
  mDesiredDofs[27] = -0.7;//right shoulder
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[33] = -3;
  
  stablePD();
  checkContactState();
  mTimer--;

  if (mRightHandContact == mBar->getBodyNode(0) ) {
    mState = "GRABRIGHT";
    std::cout << mCurrentFrame << ": " << "REACHRIGHT -> GRABRIGHT" << std::endl;
    
  }
  else{
    mState = "REACHRIGHT";

  }
}

void Controller::grabFoot(){
  leftFootGrab();
  rightFootGrab();
  rightHandRelease();
  leftHandRelease();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[17] = -1.5;
  mDesiredDofs[19] = -1.5;
  stablePD();
  mTimer--;
  
  if(mTimer==0){
    mState = "RELEASEFOOT";
    
    std::cout << mCurrentFrame << ": " << "GRABFOOT -> RELEASEFOOT" << std::endl;
     
  }


}
void Controller::swingFootStraight(){
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[17] = -1.5;
  mDesiredDofs[19] = -1.5;

  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  stablePD();
  mTimer--;
  checkContactState();
  float xFoot = mSkel->getBodyNode("h_toe_left")->getWorldCOM()(0);
  float com = mSkel->getWorldCOM()(0);

  if(com >= xFoot && mTimer<=0){
    mState= "SWINGFOOT";
    std::cout << mCurrentFrame << ": " << "SWINGFOOTSTRAIGHT -> SWINGFOOT" << std::endl;
  }
}
void Controller::swingFoot(){
  mDesiredDofs = mDefaultPose;
  
  mDesiredDofs[17] =1.5;
  mDesiredDofs[19] =1.5;

  mDesiredDofs[34] = 0.4;
  mDesiredDofs[33] = 0.4;
  
  mDesiredDofs[13] = 1.5 ;//0.0 
  mDesiredDofs[14] = -0.7 ;//0.0 
  mDesiredDofs[15] = -0.7 ;//0.0 
  stablePD();
  float xFoot = mSkel->getBodyNode("h_toe_left")->getWorldCOM()(0);
  float com = mSkel->getWorldCOM()(0);
  if(com < xFoot){
    mState= "SWINGFOOTSTRAIGHT";
    std::cout << mCurrentFrame << ": " << "SWINGFOOT -> SWINGFOOTSTRAIGHT" << std::endl;
  }

 
}
void Controller::releaseFoot(){
  
  leftFootRelease();
  rightFootRelease();

  mDesiredDofs = mDefaultPose;
  stablePD();


}

void Controller::swing() {
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[27] = 1;
  mDesiredDofs[28] = -2.6;
  mDesiredDofs[30] = 1;
  mDesiredDofs[31] = 2.6;
  mDesiredDofs[33] = 0.4;
  mDesiredDofs[34] = 0.4;
  mDesiredDofs[13] = 0.0 ;//0.0 
  mDesiredDofs[14] = 0.5;
  mDesiredDofs[15] = 0.5;
 
  stablePD();
  checkContactState();

  float v = abs(mSkel->getBodyNode("h_abdomen")->getWorldCOMVelocity()(0)+mSkel->getBodyNode("h_abdomen")->getWorldCOMVelocity()(1));
  Eigen::Vector3d pos = mSkel->getBodyNode("h_toe_right")->getWorldCOM();
  float xHand = mSkel->getBodyNode("h_hand_left")->getWorldCOM()(0);
  float xHead = mSkel->getBodyNode("h_head")->getWorldCOM()(0);
  float xThigh =mSkel->getBodyNode("h_thigh_left")->getWorldCOM()(0);
  float x = mSkel->getWorldCOM()(0);
  float xAbdomen = mSkel->getBodyNode("h_abdomen")->getWorldCOM()(0);

  if( mRightFootContact||mLeftFootContact){
     mState = "GRABFOOT";
     mTimer = 600;
     std::cout << mCurrentFrame << ": " << "SWING -> GRABFOOT" << std::endl;
     return;
  }
  if(x +0.07 < xHand)
  {
   
      mState = "SWINGCROUCH";
      
      std::cout << mCurrentFrame << ": " << "SWING -> SWINGCROUCH" << std::endl;
  }
  

}
void Controller::swingCrouch(){
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[27] = 1;
  mDesiredDofs[28] = -2.6;
  mDesiredDofs[30] = 1;
  mDesiredDofs[31] = 2.6;
  mDesiredDofs[33] = -2.5;
  mDesiredDofs[34] = -2.5;
  mDesiredDofs[13] = -2 ;
  mDesiredDofs[14] = -2.5;
  mDesiredDofs[15] = -2.5;
  
  stablePD();
  checkContactState();
  
  float xHand = mSkel->getBodyNode("h_hand_left")->getWorldCOM()(0);
  float v = abs(mSkel->getBodyNode("h_abdomen")->getWorldCOMVelocity()(0)+mSkel->getBodyNode("h_abdomen")->getWorldCOMVelocity()(1));
 
  if( mRightFootContact||mLeftFootContact){
     mState = "GRABFOOT";
     mTimer = 600;
     std::cout << mCurrentFrame << ": " << "SWING -> GRABFOOT" << std::endl;
     return;
  }
  if(v <= 0.1)
  {
      mState = "SWING";
      mTimes ++;
      isSwing = false;
      std::cout << mCurrentFrame << ": " << "SWINGCROUCH -> SWING" << std::endl;
  }
  else if (v>4){
      mState = "REACHRIGHT";
      mTimer = 300;
     
      std::cout << mCurrentFrame << ": " << "SWINGCROUCH -> REACHRIGHT" << std::endl;
  }

}
void Controller::setRelease(){
  mState = "RELEASE";
}
void Controller::setSwing(){
  mState ="SWING";
}
void Controller::setSwingCrouch(){
  mState="SWINGCROUCH";
}
void Controller::setReachRight(){
  mState="REACHRIGHT";
}
void Controller::setGrabRight(){
  mState = "GRABRIGHT";
}
void Controller::setFootRelease(){
  mState ="RELEASEFOOT";
}
void Controller::setBarPosition(float x,float y){
  deltaX=x;
  deltaY=y;
}

void Controller::release() {
  if(isFirst){
    Eigen::Vector3d com = mSkel->getBodyNode("h_toe_right")->getWorldCOM();
    cout<<com(0)<<" "<<com(1)<<" "<<com(2)<<endl;
    isFirst=false;
  }
  
  leftHandRelease();
  rightHandRelease();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[13] = -1.5;
  mDesiredDofs[14] = -1.5;
  mDesiredDofs[15] = -1.5;
  stablePD();
}
  
void Controller::stablePD() {
  Eigen::VectorXd q = mSkel->getPositions();
  Eigen::VectorXd dq = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * dq;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques += p + d - mKd * qddot * mTimestep;
}

void Controller::ankleStrategy() {
  Eigen::Vector3d com = mSkel->getWorldCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  double offset = com[0] - cop[0];
   if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }  
}

void Controller::virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset) {
  Eigen::MatrixXd jacobian = mSkel->getJacobian(_bodyNode, _offset);
  mTorques += jacobian.transpose() * _force;
}

void Controller::leftHandGrab() {  
  if (mLeftHandHold != NULL)
    return;
  checkContactState();
  if (mLeftHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_left");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mLeftHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mLeftHandHold = hold;
}

void Controller::leftHandRelease() {
  if (mLeftHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mLeftHandHold);
  mSkel->getBodyNode("h_hand_left")->setCollidable(true);
  mLeftHandHold = NULL;
}

void Controller::rightHandGrab() {  
  if (mRightHandHold != NULL)
    return;

  checkContactState();
  if (mRightHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_right");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mRightHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mRightHandHold = hold;
}


void Controller::rightHandRelease() {
  if (mRightHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mRightHandHold);
  mSkel->getBodyNode("h_hand_right")->setCollidable(true);
  mRightHandHold = NULL;
}

//**************************************
  void Controller::leftFootGrab() {  
  if (mLeftFootHold != NULL)
    return;
  checkContactState();
  if (mLeftFootContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_toe_left");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mLeftFootContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mLeftFootHold = hold;
}
  void Controller::leftFootRelease() {
  if (mLeftFootHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mLeftFootHold);
  mSkel->getBodyNode("h_toe_left")->setCollidable(true);
  mLeftFootHold = NULL;
}

void Controller::rightFootGrab() {  
  if (mRightFootHold != NULL)
    return;

  checkContactState();
  if (mRightFootContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_toe_right");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mRightFootContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mRightFootHold = hold;
}

void Controller::rightFootRelease() {
  if (mRightFootHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mRightFootHold);
  mSkel->getBodyNode("h_toe_right")->setCollidable(true);
  mRightFootHold = NULL;
}
//**************************************

dart::dynamics::Skeleton*Controller::getSkel() {
  return mSkel;
}

Eigen::VectorXd Controller::getDesiredDofs() {
  return mDesiredDofs;
}

Eigen::MatrixXd Controller::getKp() {
  return mKp;
}

Eigen::MatrixXd Controller::getKd() {
  return mKd;
}

Eigen::VectorXd Controller::ik(dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _target) {
  Eigen::Vector3d offset = _bodyNode->getLocalCOM();
  Eigen::VectorXd oldPose = mSkel->getPositions();
  Eigen::VectorXd newPose;
  for (int i = 0; i < 200; i++) {
    Eigen::Vector3d diff = _bodyNode->getWorldCOM() - _target;
    Eigen::MatrixXd jacobian = mSkel->getJacobian(_bodyNode, offset);
    jacobian.block(0, 0, 3, 6).setZero();
    newPose = mSkel->getPositions() - 0.1 * 2 * jacobian.transpose() * diff;
    mSkel->setPositions(newPose); 
    mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
  }
  mSkel->setPositions(oldPose);
  mSkel->computeForwardKinematics(true, false, false);
  return newPose;
}  

    
// 0-5 pelvis
// 6-8 left hip
// 9-11 right hip
// 12,13 abdomen
// 14 left knee
// 15 right knee
// 16 spine
// 17,18 left ankle
// 19,20 right ankle
// 21,22 head
// 23 left scapula
// 24 right scapula
// 25 left toe
// 26 right toe
// 27-29 left shoulder
// 30-32 right shoulder
// 33 left elbow
// 34 right elbow
// 35 left wrist
// 36 right wrist
