import time
import math
from datetime import datetime
from time import sleep
import numpy as np
import random
# import pybullet_data
# import cv2
import os
import argparse
# import torch

import sys
sys.path.append('./Eval')
sys.path.append('./')

class Robot:
    def __init__(self,pybullet_api,start_pos=[0, 0, 0],urdf_path=None):
       self.p = pybullet_api

       self.gripperMaxForce = 1000.0
       self.armMaxForce = 800.0
       self.endEffectorIndex = 7
       self.start_pos = start_pos

       #lower limits for null space
       self.ll=[-2.9671, -1.8326 ,-2.9671, -3.1416, -2.9671, -0.0873, -6.3, -0.0001, -0.0001, -0.0001, 0.0, 0.0, -3.14, -3.14, 0.0, 0.0, 0.0, 0.0, -0.0001, -0.0001]
       #upper limits for null space
       self.ul=[2.9671, 1.8326 ,-2.9671, 0.0, 2.9671, 3.8223, 6.3, 0.0001, 0.0001, 0.0001, 0.81, 0.81, 3.14, 3.14, 0.8757, 0.8757, -0.8, -0.8, 0.0001, 0.0001]
       #joint ranges for null space
       self.jr=[(u-l) for (u,l) in zip(self.ul,self.ll)]

       # restposes for null space
       self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
       # joint damping coefficents
       self.jd = [0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
                   0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]

       self.num_controlled_joints = 7
       self.controlled_joints = [0, 1, 2, 3, 4, 5, 6]

       self.activeGripperJointIndexList = [10, 12, 14, 16, 18, 19]

       self.gripper_left_tip_index = 13
       self.gripper_right_tip_index = 17

       model_path = os.path.join(urdf_path,"panda_robot.urdf")
    #    print("model_path in urdf",model_path)
       

       startOrientation = self.p.getQuaternionFromEuler([0, 0, np.pi])
       self.robotId = self.p.loadURDF(model_path, [0, 0, 0], useFixedBase=True) 
       self.p.resetBasePositionAndOrientation(self.robotId, start_pos, startOrientation)

       self.targetVelocities = [0] * self.num_controlled_joints
       self.positionGains = [0.03] * self.num_controlled_joints
       self.velocityGains = [1] * self.num_controlled_joints

       self.numJoint = self.p.getNumJoints(self.robotId)
       self.gripperLowerLimitList = []
       self.gripperUpperLimitList = []
       for jointIndex in range(self.numJoint):
            jointInfo = self.p.getJointInfo(self.robotId,jointIndex)
            # print(self.p.getJointInfo(self.robotId,jointIndex))
            if jointIndex in self.activeGripperJointIndexList:
                self.gripperLowerLimitList.append(jointInfo[8])
                self.gripperUpperLimitList.append(jointInfo[9])
 
    def reset(self): 
        ####### Set Dynamic Parameters for the gripper pad######
        friction_ceof = 1000.0
        self.p.changeDynamics(self.robotId, self.gripper_left_tip_index, lateralFriction=friction_ceof)
        self.p.changeDynamics(self.robotId, self.gripper_left_tip_index, rollingFriction=friction_ceof)
        self.p.changeDynamics(self.robotId, self.gripper_left_tip_index, spinningFriction=friction_ceof)

        self.p.changeDynamics(self.robotId, self.gripper_right_tip_index, lateralFriction=friction_ceof)
        self.p.changeDynamics(self.robotId, self.gripper_right_tip_index, rollingFriction=friction_ceof)
        self.p.changeDynamics(self.robotId, self.gripper_left_tip_index, spinningFriction=friction_ceof)

        self.p.changeDynamics(self.robotId, 16, contactStiffness=300.0, contactDamping=0.9)
        self.p.changeDynamics(self.robotId, 17, contactStiffness=300.0, contactDamping=0.9)
        self.p.changeDynamics(self.robotId, 18, linearDamping=0.1)
        self.p.changeDynamics(self.robotId, 18, angularDamping=0.1)
        self.p.changeDynamics(self.robotId, 18, contactStiffness=300.0, contactDamping=0.9)
        self.p.changeDynamics(self.robotId, 19, linearDamping=0.1)
        self.p.changeDynamics(self.robotId, 19, angularDamping=0.1)
        self.p.changeDynamics(self.robotId, 19, contactStiffness=300.0, contactDamping=0.9)
        self.p.changeDynamics(self.robotId, 20, linearDamping=0.1)
        self.p.changeDynamics(self.robotId, 20, angularDamping=0.1)
        self.p.changeDynamics(self.robotId, 20, contactStiffness=300.0, contactDamping=0.9)


    def jointPositionControl(self,q_list,gripper=None,maxVelocity=None):
        q_list = q_list.tolist()
        if gripper is None:
            self.p.setJointMotorControlArray(bodyUniqueId=self.robotId,jointIndices=self.controlled_joints,controlMode=self.p.POSITION_CONTROL,targetPositions=q_list)
        else:
            self.gripperOpen = 1 - gripper/255.0
            self.gripperPos = np.array(self.gripperUpperLimitList) * (1 - self.gripperOpen) + np.array(self.gripperLowerLimitList) * self.gripperOpen
            self.gripperPos = self.gripperPos.tolist()
            armForce = [self.armMaxForce] * len(self.controlled_joints)
            gripperForce = [self.gripperMaxForce] * len(self.activeGripperJointIndexList)
            
            self.p.setJointMotorControlArray(bodyUniqueId=self.robotId,jointIndices=self.controlled_joints,controlMode=self.p.POSITION_CONTROL,targetPositions=q_list,forces=armForce)
            self.p.setJointMotorControlArray(bodyUniqueId=self.robotId,jointIndices=self.activeGripperJointIndexList,controlMode=self.p.POSITION_CONTROL,targetPositions=self.gripperPos,forces=gripperForce)
        self.p.stepSimulation()

    def setJointValue(self,q,gripper):
        for j in range(len(self.controlled_joints)):
            self.p.resetJointState(self.robotId,j,q[j],0.0)
        self.gripperOpen = 1 - gripper/255.0
        self.gripperPos = np.array(self.gripperUpperLimitList) * (1 - self.gripperOpen) + np.array(self.gripperLowerLimitList) * self.gripperOpen
        for j in range(6):
            index_ = self.activeGripperJointIndexList[j]
            self.p.resetJointState(self.robotId,index_,self.gripperPos[j],0.0)

    def getJointValue(self):
        qlist = []
        for j in range(len(self.controlled_joints)):
            qlist.append(self.p.getJointState(self.robotId,j)[0])
        #print(qlist)
        return qlist

    def getEndEffectorPos(self):
        return self.p.getLinkState(self.robotId, self.endEffectorIndex)[0]

    def getEndEffectorVel(self):
        return self.p.getLinkState(self.robotId, self.endEffectorIndex)[6]

    def getEndEffectorOrn(self):
        return self.p.getLinkState(self.robotId, self.endEffectorIndex)[1]

    def operationSpacePositionControl(self,pos,orn=None,null_pose=None,gripperPos=None):
        if null_pose is None and orn is None:
            jointPoses = self.p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, pos,
                                                      lowerLimits=self.ll,
                                                      upperLimits=self.ul,
                                                      jointRanges=self.jr)[:self.num_controlled_joints]

        elif null_pose is None and orn is not None:
            jointPoses = self.p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, pos, orn,
                                                      lowerLimits=self.ll,
                                                      upperLimits=self.ul,
                                                      jointRanges=self.jr)[:self.num_controlled_joints]

        elif null_pose is not None and orn is None:
            jointPoses = self.p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, pos,
                                                      lowerLimits=self.ll,
                                                      upperLimits=self.ul,
                                                      jointRanges=self.jr,
                                                      restPoses=null_pose)[:self.num_controlled_joints]

        else:
            jointPoses = self.p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, pos, orn,
                                                      lowerLimits=self.ll,
                                                      upperLimits=self.ul,
                                                      jointRanges=self.jr,
                                                      restPoses=null_pose)[:self.num_controlled_joints]

        #print("jointPoses",jointPoses)
        if gripperPos is None:
            self.p.setJointMotorControlArray(bodyIndex=self.robotId,
                                        jointIndices=self.controlled_joints,
                                        controlMode=self.p.POSITION_CONTROL,
                                        targetPositions=jointPoses,
                                        targetVelocities=self.targetVelocities,
                                        forces=[self.armMaxForce] * self.num_controlled_joints)
        else:
            self.gripperOpen = 1.0 - gripperPos/255.0
            self.gripperPos = np.array(self.gripperUpperLimitList) * (1 - self.gripperOpen) + np.array(self.gripperLowerLimitList) * self.gripperOpen
            self.gripperPos = self.gripperPos.tolist()
            gripperForce = [self.gripperMaxForce] * len(self.activeGripperJointIndexList)
            jointPoses = np.array(jointPoses).tolist()
            self.p.setJointMotorControlArray(bodyIndex=self.robotId,
                                        jointIndices=self.controlled_joints + self.activeGripperJointIndexList,
                                        controlMode=self.p.POSITION_CONTROL,
                                        targetPositions=jointPoses + self.gripperPos,
                                        targetVelocities=self.targetVelocities + [0.0] * len(self.activeGripperJointIndexList),
                                        forces=[self.armMaxForce] * self.num_controlled_joints + gripperForce)

        self.p.stepSimulation()

    
    def gripperControl(self,gripperPos):
        self.gripperOpen = 1.0 - gripperPos/255.0 
        self.gripperPos = np.array(self.gripperUpperLimitList) * (1 - self.gripperOpen) + np.array(self.gripperLowerLimitList) * self.gripperOpen
        self.gripperPos = self.gripperPos.tolist()
        gripperForce = [self.gripperMaxForce] * len(self.activeGripperJointIndexList)
        self.p.setJointMotorControlArray(bodyUniqueId=self.robotId,jointIndices=self.activeGripperJointIndexList,controlMode=self.p.POSITION_CONTROL,targetPositions=self.gripperPos,forces=gripperForce)
        self.p.stepSimulation()


    def gripperTipPosControl(self, left_tip_pos, right_tip_pos):
        jointPoses = self.p.calculateInverseKinematics2(self.robotId, [self.gripper_left_tip_index, self.gripper_right_tip_index], 
                                                      [left_tip_pos, right_tip_pos] ,
                                                      lowerLimits=self.ll,
                                                      upperLimits=self.ul,
                                                      jointRanges=self.jr)
        gripperForce = [self.gripperMaxForce] * len(self.activeGripperJointIndexList)
        # print(np.array(jointPoses)[np.array(self.activeGripperJointIndexList).astype(int)])
        self.p.setJointMotorControlArray(bodyIndex=self.robotId,
                                        jointIndices=self.controlled_joints + self.activeGripperJointIndexList,
                                        controlMode=self.p.POSITION_CONTROL,
                                        targetPositions=np.array(jointPoses[:self.num_controlled_joints]).tolist()+ np.array(jointPoses[-len(self.activeGripperJointIndexList):]).tolist(),
                                        targetVelocities=self.targetVelocities+ [0.0] * len(self.activeGripperJointIndexList),
                                        forces=[self.armMaxForce] * self.num_controlled_joints + gripperForce)

    def positionControlWithId(self, id, pos, orn=None, control_gripper=False):
        if orn is None:
            jointPoses = self.p.calculateInverseKinematics(self.robotId, id, 
                                                            pos,
                                                            lowerLimits=self.ll,
                                                            upperLimits=self.ul,
                                                            jointRanges=self.jr)
        else:
            jointPoses = self.p.calculateInverseKinematics(self.robotId, id, 
                                                            pos,
                                                            self.p.getQuaternionFromEuler(orn),
                                                            lowerLimits=self.ll,
                                                            upperLimits=self.ul,
                                                            jointRanges=self.jr)            
        if control_gripper:
            gripperForce = [self.gripperMaxForce] * len(self.activeGripperJointIndexList)
            self.p.setJointMotorControlArray(bodyIndex=self.robotId,
                                        jointIndices=self.controlled_joints + self.activeGripperJointIndexList,
                                        controlMode=self.p.POSITION_CONTROL,
                                        targetPositions=np.array(jointPoses[:self.num_controlled_joints]).tolist()+ np.array(jointPoses[-len(self.activeGripperJointIndexList):]).tolist(),
                                        targetVelocities=self.targetVelocities+ [0.0] * len(self.activeGripperJointIndexList),
                                        forces=[self.armMaxForce] * self.num_controlled_joints + gripperForce)
        else:
            self.p.setJointMotorControlArray(bodyIndex=self.robotId,
                                        jointIndices=self.controlled_joints,
                                        controlMode=self.p.POSITION_CONTROL,
                                        targetPositions=np.array(jointPoses[:self.num_controlled_joints]).tolist(),
                                        targetVelocities=self.targetVelocities,
                                        forces=[self.armMaxForce] * self.num_controlled_joints)
    def positionDeltaControlWithId(self, id, pos_delta, orn=None, control_gripper=False):
        proxy_goal_pos = self.p.getLinkState(self.robotId, id)[0] + pos_delta
        self.positionControlWithId(id, proxy_goal_pos, orn=orn, control_gripper=control_gripper)
        return proxy_goal_pos