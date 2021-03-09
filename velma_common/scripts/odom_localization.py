#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_common')

import rospy
import tf

def lookupTransformStamped(tf_listener, target_frame, source_frame, time):
    msg = tf_listener._buffer.lookup_transform(tf.listener.strip_leading_slash(target_frame), tf.listener.strip_leading_slash(source_frame), time)
    t = msg.transform.translation
    r = msg.transform.rotation
    return msg.header.stamp, [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]

if __name__ == "__main__":
    rospy.init_node('odom_localization', anonymous=False)
    rospy.sleep(0.5)

    tf_listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            time, pos, rot = lookupTransformStamped(tf_listener, 'odom', 'world', rospy.Time(0))
        except:
            time = None

        if not time is None:
            br.sendTransform(pos,
                     rot,
                     time,
                     'torso_base',
                     'world')
        rospy.sleep(0.05)

    exit(0)
