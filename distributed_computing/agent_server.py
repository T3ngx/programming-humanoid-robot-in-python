'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))


from inverse_kinematics import InverseKinematicsAgent
import threading
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
import pickle #序列化对象，在需要的时候读取

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.posture_classifier = pickle.load(robot_pose_data)
        self.posture = 'unknown'
        robot_pose_data.close()
        
        # create server
        self.server = SimpleXMLRPCServer(('localhost',8888), requestHandler = RequestHandler, allow_none= True )
        self.server.register_introspection_functions()
        self.server.register_multicall_functions()
        self.server.register_instance(self)
        
        
        self.thread = threading.Thread(target = self.server.serve_forever)
        self.thread.start()
        
        print ('Server')
        
        
    def think(self,perception):
        self.posture = self.recognize_postures(perception)
        return super(ServerAgent,self).think(perception)
    
    def recognize_postures(self, perception):
        posture = 'unknown'
        Data = [[perception.joint['LHipYawPitch'],
                 perception.joint['LHipRoll'],
                 perception.joint['LHipPitch'],
                 perception.joint['LKneePitch'],
                 perception.joint['RHipYawPitch'],
                 perception.joint['RHipRoll'],
                 perception.joint['RKneePitch']]]
        
        
        return posture
        
        
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint.get(joint_name)
    
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        if joint_name in self.perception.joint:
            self.target_joints[joint_name] = angle
            

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture
        

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        #set_transforms inverse kinematics
        return self.transform[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        #set_transforms inverse kinematics
        self.transforms[effctor_name] = transform



if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

