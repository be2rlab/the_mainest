#!/usr/bin/env python3

from typing import Mapping
import rospy
import numpy as np

from msdp.srv import GoalPoses, JSPosition, CartPosition
from uhvat_ros_driver.srv import SetGripperState

CANDLE_POSE = [0, -90, 0, -90, 0, 0]
MAPPING_POSES = [
    [-70, -86, -60, -120, 145, 0],
    [-59, -75, -71, -154, 39, 46],
    [-81, -91, -113, -72, 176, -4],
    [-63, -64, -100, -137, 28, -3]
]

radians = np.pi / 180

TASK = [
    'idle',
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
        self.go_to_js(np.array(CANDLE_POSE) * radians)

    def mapping(self):
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
        # return self.call_srv('gripper_state', SetGripperState , state)
        rospy.wait_for_service('gripper_state')
        try:
            srv_obj = rospy.ServiceProxy('gripper_state', SetGripperState)
            resp = srv_obj(state)
            return resp
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return -1


def main():
    # test_js()
    """
     0 -- candle
     1 -- mapping 1

    """
    
    task_interface = TaskInterface()
    
    state = 'idle'
    state_id = 0
    resp = True

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        print(state)

        if state == 'mapping' and resp:
            resp = False
            # resp = task_interface.mapping()
            resp = True
        elif state == 'initial' and resp:
            resp = False
            resp = task_interface.initial()
        elif state == 'pnp' and resp:
            resp = False
            resp = task_interface.pnp()
        else:
            resp = task_interface.go_to_js(np.array(CANDLE_POSE)* radians)
            rospy.loginfo("Return to initial CANDLE pose")
        
        if resp == True:
            state = TASK[state_id]
            state_id = state_id + 1
            if state_id > 3:
                state_id = 0

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("the_mainest_node")
    main()
