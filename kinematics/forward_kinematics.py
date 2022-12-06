'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, cos, sin

from recognize_posture import PostureRecognitionAgent

# for testing
# from keyframes import rightBackToStand


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }
        self.joint_offsets = {'Head': [[0, 0, 126.5], [0, 0, 0]],
                              'LArm': [[0, 98, 100], [0, 0, 0], [105, 15, 0], [0, 0, 0]],
                              'LLeg': [[0, 50, -85], [0, 0, 0], [0, 0, 0], [0, 0, -100], [0, 0, -102.9], [0, 0, 0]],
                              'RArm': [[0, -98, 100], [0, 0, 0], [105, -15, 0], [0, 0, 0]],
                              'RLeg': [[0, -50, -85], [0, 0, 0], [0, 0, 0], [0, 0, -100], [0, 0, -102.9], [0, 0, 0]]
                             }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        
        # Calculate theta values
        cos_theta = cos(joint_angle)
        sin_theta = sin(joint_angle)
        
        # Yaw is z
        # Pitch is y      
        # Roll is x
        
        if 'Yaw' in joint_name:
            T[0:3, 0:3] = matrix([[cos_theta, sin_theta, 0],
                      [-sin_theta, cos_theta, 0],
                      [0, 0, 1]])
        elif 'Pitch' in joint_name:
            T[0:3, 0:3] = matrix([[cos_theta, 0, sin_theta],
                                  [0, 1, 0],
                                  [sin_theta, 0, cos_theta]])
        elif 'Roll' in joint_name:
            T[0:3, 0:3] = matrix([[1, 0, 0],
                      [0, cos_theta, -sin_theta],
                      [0, sin_theta, cos_theta]])
        # print("Joint Name:", joint_name)
        for chain in self.chains.keys():
            if joint_name in self.chains[chain]:
                # print("chains:", self.chains[chain])
                T[3, 0:3] = self.joint_offsets[chain][self.chains[chain].index(joint_name)]
                break

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        
        print("joint:", joints)
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T, Tl)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    # agent.keyframes = rightBackToStand()
    agent.run()
