#!/usr/bin/env python

## Runs all integration tests.
# @ingroup integration_tests
# @file test_all.py
# @namespace scripts.test_all Integration test

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import subprocess
import rospy

from rcprg_ros_utils import exitError

if __name__ == "__main__":

    rospy.init_node('test_all', anonymous=True)

    rospy.sleep(0.5)

    tests = ["test_init.py", "test_head.py", "test_head_complex.py", "test_safe_col.py",
        "test_jimp.py", "test_grippers.py", "test_jimp_planning.py",
        "test_jimp_planning_attached.py", "test_cimp_pose.py", "test_cimp_tool.py"]

    i = 0
    for t in tests:
        i += 1
        print "running test " + str(i) + "/" + str(len(tests)) + ": " + t
        result = subprocess.call(['rosrun', 'velma_task_cs_ros_interface', t])
        print "test '" + t + "' ended with result " + str(result)
        if result != 0:
            exitError(1)
    exitError(0)

