#!/usr/bin/env python3

from typing import Mapping
import rospy
import numpy as np
import copy
from rospy.core import loginfo

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion

from the_mainest.srv import GoalPoses, JSPosition, JSPositionResponse, CartPosition, CartPositionResponse
from uhvat_ros_driver.srv import SetGripperState

from the_mainest.srv import Give, ObbArr, GetPNPPoses, AttachObj


radians = np.pi / 180

CANDLE_POSE = np.array([0, -90, 0, -90, 0, 0]) * radians
INITIAL_POSE = np.array([-78, -107, -14, -149, 90, 12]) * radians

MAPPING_POSES = np.array([
    [-70, -86, -60, -120, 145, 0],
    [-59, -75, -71, -154, 39, 46],
    [-81, -91, -113, -72, 176, -4],
    [-63, -64, -100, -137, 28, -3]
]) * radians

TASK = [
    'idle',
    'candle'
    'mapping',
    'initial',
    'pnp'
]


def wait_for(sec):
    rospy.loginfo(f"Wait [{sec}] sec")
    for ti in range(sec):
        rospy.loginfo(f"{ti+1} [sec]")
        rospy.sleep(1)

class TaskInterface(object):

    def __init__(self) -> None:
        super().__init__()
        
        rospy.wait_for_service('cart_position_cmd')
        self.srv_obj = rospy.ServiceProxy('cart_position_cmd', CartPosition)
        
        self.pnp_state = 'idl'
        # [
        #     'idl',
        #     'p1', 
        #     'p1*', 
        #     'gripper_close', 
        #     'attach_obj' 
        #     'p2', 
        #     'p1*', 
        #     'gripper_open', 
        #     'dettach_obj'
        # ]

        p_box = Pose()
        p_box.position.x = 0.4
        p_box.position.y = 0.0
        p_box.position.z = 0.6
        p_box.orientation.x = -np.sqrt(2)/2
        p_box.orientation.y = np.sqrt(2)/2
        p_box.orientation.z = 0
        p_box.orientation.w = 0

        p_mouse = Pose()
        p_mouse.position.x = -0.4
        p_mouse.position.y = 0.0
        p_mouse.position.z = 0.6
        p_mouse.orientation.x = np.sqrt(2)/2
        p_mouse.orientation.y = np.sqrt(2)/2
        p_mouse.orientation.z = 0
        p_mouse.orientation.w = 0

        p_regbi = Pose()
        p_regbi.position.x = 0
        p_regbi.position.y = -0.4
        p_regbi.position.z = 0.6
        p_regbi.orientation.x = 0
        p_regbi.orientation.y = 1
        p_regbi.orientation.z = 0
        p_regbi.orientation.w = 0


        # 'box': Pose(Point(0.54, -0.15, 0.65), Quaternion(-0.6162, 0.7874, -0.0086, 0.0089)),
        # TODO set the points
        self.BOXES = {
            'box': p_box,
            'mouse': p_mouse,
            'regbi': p_regbi
        }



    def pnp(self, obj_name, grasp_pose):
        resp = True

        while resp:
            rospy.loginfo(f"PnP STATE: {self.pnp_state}")

            if self.pnp_state == 'idl' and resp:
                resp = False
                wait_for(5)
                self.pnp_state, resp = 'p1*', True
                # self.pnp_state, resp = 'p2*', True

            elif self.pnp_state == 'p1*' and resp:
                # p1* STATE -- over object
                resp = False
                
                p1 = copy.deepcopy(grasp_pose)
                p1.position.z = p1.position.z + 0.5

                resp = self.go_to_cart(p1)
                self.pnp_state = 'p1'

            elif self.pnp_state == 'p1' and resp:
                resp = False

                p1_ = copy.deepcopy(grasp_pose)
                p1_.position.z = p1_.position.z + 0.3

                resp = self.go_to_cart(p1_)
                self.pnp_state = 'gripper_close'

            elif self.pnp_state == 'gripper_close' and resp:
                resp = False
                self.gripper(6)
                wait_for(2)
                self.pnp_state, resp = 'attach_obj', True

            elif self.pnp_state == 'attach_obj' and resp:
                resp = False
                resp = self.call_attach_obj('obj', True)
                self.pnp_state = 'p2*'

            elif self.pnp_state == 'p2*' and resp:
                resp = False
                
                p2 = self.BOXES[obj_name]
                p2.position.z = p2.position.z

                resp = self.go_to_cart(p2)
                self.pnp_state = 'p2'

            elif self.pnp_state == 'p2' and resp:
                resp = False

                p2 = self.BOXES[obj_name]
                p2.position.z = p2.position.z - 0.1

                resp = self.go_to_cart(p2)
                self.pnp_state = 'gripper_open'

            elif self.pnp_state == 'gripper_open' and resp:
                resp = False
                self.gripper(0)
                wait_for(2)
                self.pnp_state, resp = 'dettach_obj', True

            elif self.pnp_state == 'dettach_obj' and resp:
                resp = False
                resp = self.call_attach_obj('obj', False)
                self.pnp_state = 'exit'
            elif self.pnp_state == 'exit' and resp:
                return True

    def initial(self):
        return self.go_to_js(np.array(CANDLE_POSE) * radians)

    def mapping(self):
        """
        Go throw set of predefined points
        """
        resp = False
        for i in range(len(MAPPING_POSES)):
            print(i)
            resp = self.go_to_js(np.array(MAPPING_POSES[i]) * radians)
            if not resp:
                rospy.logwarn(f"Mapping pose {i} do not feasible")
        return resp

    def go_to_cart(self, pose):
        # return self.call_srv('cart_position_cmd', CartPosition, pose)
        
        try:
            resp = self.srv_obj(pose)
            return resp.status
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False


        # rospy.wait_for_service('cart_position_cmd')
        # try:
        #     srv_obj = rospy.ServiceProxy('cart_position_cmd', CartPosition)
        #     print(srv_obj)
        #     print(type(pose), pose)
        #     resp = srv_obj(pose)
        #     return resp.status
        # except rospy.ServiceException as e:
        #     print(f"Service call failed: {e}")
        #     return False


    def go_to_js(self, q):
        # return self.call_srv('js_position_cmd', JSPosition, q)
        rospy.wait_for_service('js_position_cmd')
        print(type(q), q.tolist())
        try:
            srv_obj = rospy.ServiceProxy('js_position_cmd', JSPosition)
            resp = srv_obj(q.tolist())
            return resp.status
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return -1

    def call_srv(self, srv_name, srv_type, q):
        rospy.wait_for_service(srv_name)
        try:
            srv_obj = rospy.ServiceProxy(srv_name, srv_type)
            resp = srv_obj(q)
            return resp.status
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return -1
    
    def gripper(self, state):
        """
        :state in [0,1,2,3,4,5,6]
        [0,1,2,3] -- positions from open to close
        [4,5,6] -- force closing from low force to high force
        """
        rospy.wait_for_service('gripper_state')
        try:
            srv_obj = rospy.ServiceProxy('gripper_state', SetGripperState)
            srv_obj(state)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def call_give(self, obj):
        """ 
        Sets the object `obj` to `grasping_vision` node for 
        boundingbox estimation.
        """
        rospy.wait_for_service('give')
        try:
            srv_obj = rospy.ServiceProxy('give', Give)
            srv_obj(String(f"give {obj}"))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def call_obb_arr_srv(self):
        """ Returns the 12 numbers from `grasping_vision` node. """
        rospy.wait_for_service('obb_arr_srv')
        try:
            srv_obj = rospy.ServiceProxy('obb_arr_srv', ObbArr)
            resp = srv_obj()
            return resp.data
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def call_get_pnp_poses(self, data):
        """ 
        Returns only p1 !!! (and can return p2 
        (p2 hardcoded for output box position, but not used here) 
        """
        rospy.wait_for_service('get_pnp_poses')
        try:
            srv_obj = rospy.ServiceProxy('get_pnp_poses', GetPNPPoses)
            # f = Float32MultiArray()
            # f.data = data
            resp = srv_obj(data)
            return resp.p1
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def call_attach_obj(self, obj_id, gripper_state):
        rospy.wait_for_service('attach_obj')
        try:
            srv_obj = rospy.ServiceProxy('attach_obj', AttachObj)
            resp = srv_obj(obj_id, gripper_state)
            return resp.status
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


    def call_pnp(self, p1, p2):
        status = False

        # TODO add more logic
        status1 = self.go_to_cart(p1)
        status2 = self.go_to_cart(p2)

        status = status1 and status2
        return status


def main():
   
    task_interface = TaskInterface()
    
    ## Usage:
    # status = task_interface.initial()
    # status = task_interface.mapping()
    # status = task_interface.gripper(0)
    # task_interface.call_give('regbi')
    # obb_arr = task_interface.call_obb_arr_srv()
    # obb_arr = task_interface.call_pnp(p1, p2)

    state = 'idle'
    state_id = 0
    resp = True
    obj_name = ''
    data12numbers = []
    cnt_try_give = 0
    p1 = Pose()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        rospy.loginfo(f"Now: {state}")

        if state == 'idle' and resp:
            "IDLE STATE"
            resp = False
            key = input('Press any key to start or `q` to exit\n')
            if key == 'q':
                break
            state, resp = 'candle', True
        elif state == 'candle' and resp:
            "CANDLE STATE"
            resp = False
            # resp = task_interface.go_to_js(np.array(CANDLE_POSE))
            wait_for(1)
            state, resp = 'initial', True
        elif state == 'initial' and resp:
            "INITIAL STATE"
            resp = False
            resp = task_interface.go_to_js(np.array(INITIAL_POSE))
            state = 'give'
        elif state == 'give' and resp:
            "GIVE STATE"
            resp = False
            
            if cnt_try_give > 5:    # check if try to give 5 times 
                cnt_try_give = 0
                obj_name = ''    
            
            if obj_name == '':      # ask to enter the object name
                obj_name = input('Look at the `/segmented_view` topic and choose the the object name to grasp:\n')

            print(f"Choosed object is: {obj_name}")
            task_interface.call_give(obj_name)
            wait_for(1)
            state, resp = 'get_bbox', True

        elif state == 'get_bbox' and resp:
            "GET BBOX STATE"
            resp = False
            data12numbers = task_interface.call_obb_arr_srv()
            print("DATA 12 numbers: ", data12numbers.data)
            if len(data12numbers.data) == 12:
                state, resp = 'get_pnp_poses', True
            else:
                rospy.logwarn("Data from `add_object` is not recived")
                rospy.logwarn("Trying to get one more times...")
                data12numbers = []
                state, resp = 'give', True
                cnt_try_give = cnt_try_give + 1
        
        elif state == 'get_pnp_poses' and resp:
            "GET PnP POSES STATE"
            resp = False
            p1 = task_interface.call_get_pnp_poses(data12numbers)
            print('**********', p1)

            if type(p1) is Pose:   # check for Pose data validity
                if type(p1.position.x) is float:
                    state, resp = 'pnp', True
                    # state, resp = 'idle', True
                else:
                    rospy.logwarn("PnP STATE: the grasp pose is bad")
            else:
                rospy.logwarn("call the service `get_pnp_poses` one more times...")
                resp = True

        elif state == 'pnp' and resp:
            "PnP STATE"
            resp = False
            resp = task_interface.pnp(obj_name, p1)
            
            p1 = Pose()
            data12numbers = []
            obj_name = ''
            state = 'idle'

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("the_mainest_node")
    main()
