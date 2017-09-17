#!/usr/bin/env python
# -*- python -*-

import sys
import time
import unittest
import rospy
import rostest
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from o3d3xx.msg import Extrinsics

class TestCamera(unittest.TestCase):
    """
    This class tests the following:
      - Getting data from the camera
      - Computing the Cartesian data and comparing it to ground truth
      - To do the comparison to ground truther, the computed cartesian data
        must also be transformed, to do that we use the tf2 API and hence The
        transform from the optical frame to the camera frame that we are
        publishing is also tested.

    NOTE: The camera h/w is needed to run this test.
    """

    def __init__(self, *args):
        super(TestCamera, self).__init__(*args)
        self.success = False

        self.cloud_ = None # ground truth for cartesian computation
        self.rdis_ = None
        self.uvec_ = None
        self.extrinsics_ = None

    def test_camera(self):
        time.sleep(2.0) # <-- let rosmaster and camera node start
        rospy.init_node('test_camera')

        self.bridge_ = CvBridge()
        self.rdis_sub_ = \
          rospy.Subscriber("/depth", Image, self.image_cb,
                           queue_size=None, callback_args="rdis")
        self.cloud_sub_ = \
          rospy.Subscriber("/xyz_image", Image, self.image_cb,
                           queue_size=None, callback_args="cloud")
        self.uvec_sub_ = \
          rospy.Subscriber("/unit_vectors", Image, self.image_cb,
                           queue_size=None, callback_args="uvec")
        self.extrinsics_sub_ = \
          rospy.Subscriber("/extrinsics", Extrinsics, self.extrinsics_cb,
                           queue_size=None)

        # If it takes more than 10 secs to run our test, something is wrong.
        timeout_t = time.time() + 10.0
        rate = rospy.Rate(10.0)
        while ((not rospy.is_shutdown()) and
               (not self.success) and
               (time.time() < timeout_t)):
            # Make sure we have all the data we need to compute
            if ((self.rdis_ is not None) and
                (self.cloud_ is not None) and
                (self.uvec_ is not None) and
                (self.extrinsics_ is not None)):

                # Make sure the cloud and rdis are from the same image
                # acquisition.
                d = self.rdis_.header.stamp - self.cloud_.header.stamp
                if d.to_sec() == 0:
                    self.success = self.compute_cartesian()
                    break
                else:
                    # get new data
                    self.rdis_ = None
                    self.cloud_ = None

            rate.sleep()

        self.assertTrue(self.success)

    def extrinsics_cb(self, data, *args):
        if self.extrinsics_ is None:
            self.extrinsics_ = data

    def image_cb(self, data, *args):
        im_type = args[0]
        if im_type == "rdis":
            if self.rdis_ is None:
                self.rdis_ = data
        elif im_type == "cloud":
            if self.cloud_ is None:
                self.cloud_ = data
        elif im_type == "uvec":
            if self.uvec_ is None:
                self.uvec_ = data

    def compute_cartesian(self):
        """
        Computes the Cartesian data from the radial distance image, unit
        vectors, and extrinsics. Then transforms it to the camera frame using
        the tf2 api. Once transformed, it will do a pixel-by-pixel comparision
        to ground truth.

        Returns a bool indicating the the success of the computation.
        """
        rdis = np.array(self.bridge_.imgmsg_to_cv2(self.rdis_))
        uvec = np.array(self.bridge_.imgmsg_to_cv2(self.uvec_))
        cloud = np.array(self.bridge_.imgmsg_to_cv2(self.cloud_))

        # Get the unit vectors
        ex = uvec[:,:,0]
        ey = uvec[:,:,1]
        ez = uvec[:,:,2]

        # Cast the radial distance image to float
        rdis_f = rdis.astype(np.float32)

        # Compute Cartesian
        x_ = (ex * rdis_f) + self.extrinsics_.tx
        y_ = (ey * rdis_f) + self.extrinsics_.ty
        z_ = (ez * rdis_f) + self.extrinsics_.tz

        # Transform to target frame
        #
        # NOTE: This could obviously be vectorized if we exploit our apriori
        # knowledge of the transform, but we want to test the transform we are
        # broadcasting via tf and the tf2 api for doing the transformation.
        #
        # I agree, this loop is slow and ugly :-( -- mostly b/c using the tf2
        # interface with our data structures here is kind of wonky
        #
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        n_rows = x_.shape[0]
        n_cols = x_.shape[1]
        x_f = np.zeros((n_rows, n_cols), dtype=np.float64)
        y_f = np.zeros((n_rows, n_cols), dtype=np.float64)
        z_f = np.zeros((n_rows, n_cols), dtype=np.float64)
        for i in range(n_rows):
            for j in range(n_cols):
                p = PointStamped()
                p.header = self.rdis_.header
                p.point.x = x_[i,j]
                p.point.y = y_[i,j]
                p.point.z = z_[i,j]
                pt = tf_buffer.transform(p, self.cloud_.header.frame_id,
                                         rospy.Duration(2.0))
                x_f[i,j] = pt.point.x
                y_f[i,j] = pt.point.y
                z_f[i,j] = pt.point.z

        # Cast to int16_t
        x_i = x_f.astype(np.int16)
        y_i = y_f.astype(np.int16)
        z_i = z_f.astype(np.int16)

        # Explicitly set to zero any bad pixels
        mask = rdis == 0
        x_i[mask] = 0
        y_i[mask] = 0
        z_i[mask] = 0

        # Compare to ground truth -- its subtle, but we are testing here that
        # we are accurate to w/in 1mm
        x_mask = np.fabs(x_i - cloud[:,:,0]) > 1
        y_mask = np.fabs(y_i - cloud[:,:,1]) > 1
        z_mask = np.fabs(z_i - cloud[:,:,2]) > 1

        self.assertTrue(x_mask.sum() == 0)
        self.assertTrue(y_mask.sum() == 0)
        self.assertTrue(z_mask.sum() == 0)

        # if any of the above asserts fail, the test will error out,
        # so, we return True here carte blanche
        return True

def main():
    rostest.rosrun('o3d3xx', 'test_camera', TestCamera, sys.argv)
    return 0

if __name__ == '__main__':
    sys.exit(main())
