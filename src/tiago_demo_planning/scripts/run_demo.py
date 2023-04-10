#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs import msg
from std_srvs import srv
from xyz_dispatch_msgs.srv import DispatchService
from std_msgs.msg import Int32
import time


if __name__ == "__main__":
    dispatching_proxy = rospy.ServiceProxy("/xyz_plan_dispatch/online_dispatch", DispatchService)
    begin = time.time()
    dispatching_proxy()

    dispatch_end = time.time()
    print(f"Total time  : {dispatch_end - begin}")
    pass