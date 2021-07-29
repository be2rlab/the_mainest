#!/usr/bin/env python3

from typing import Mapping
import rospy
import numpy as np
from rospy.core import loginfo

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

from msdp.srv import GoalPoses, JSPosition, JSPositionResponse, CartPosition, CartPositionResponse
from uhvat_ros_driver.srv import SetGripperState

from the_mainest.srv import Give, ObbArr


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


class TaskInterface(object):

    def __init__(self) -> None:
        super().__init__()
        pass

    def pnp(self):
        resp = False
        cnt = 0
        while cnt < 10:
            print(cnt)
            if cnt % 2 == 0:
                resp = self.go_to_js(np.array(CANDLE_POSE) * radians)
                self.gripper(0)
            else:
                resp = self.go_to_js(np.array(CANDLE_POSE) * radians - 0.1)
                self.gripper(5)
            if not resp:
                rospy.logwarn(f"Mapping pose {i} do not feasible")
            cnt = cnt + 1

        return resp

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
        return self.call_srv('cart_position_cmd', CartPosition, pose)

    def go_to_js(self, q):
        return self.call_srv('js_position_cmd', JSPosition, q)

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

    def call_pnp(self, p1, p2):
        status = False

        # TODO add more logic
        status1 = self.go_to_cart(p1)
        status2 = self.go_to_cart(p2)

        status = status1 and status2
        return status

def wait_for(sec):
    rospy.loginfo(f"Wait [{sec}] sec")
    for ti in range(sec):
        rospy.loginfo(f"{ti+1} [sec]")
        rospy.sleep(1)

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
            resp = True # task_interface.go_to_js(np.array(INITIAL_POSE))
            state = 'give'
        elif state == 'give' and resp:
            "GIVE STATE"
            resp = False
            if obj_name == '':
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
                obj_name = ''
                state, resp = 'pnp', True
            else:
                rospy.logwarn("Data from `add_object` is not recived")
                rospy.logwarn("Trying to get one more times...")
                data12numbers = []
                state, resp = 'give', True
        elif state == 'pnp' and resp:
            "PnP STATE"
            resp = False
            
            state = 'idle'

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("the_mainest_node")
    main()
