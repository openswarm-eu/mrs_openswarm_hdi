#!/usr/bin/env python
import rospy
from mrs_msgs.srv import ReferenceStampedSrv, ReferenceStampedSrvRequest
from mrs_msgs.msg import ReferenceStamped
import utm

class ReferenceCaller:
    def __init__(self):
        # wait for service availability (adjust names)
        self.service1_name = "/uav6/control_manager/reference"
        self.service2_name = "/uav7/control_manager/reference"
        rospy.loginfo(f"Waiting for service: {self.service1_name}")
        rospy.wait_for_service(self.service1_name)
        rospy.loginfo(f"Waiting for service: {self.service2_name}")
        rospy.wait_for_service(self.service2_name)

        # create service proxies
        self.proxy1 = rospy.ServiceProxy(self.service1_name, ReferenceStampedSrv)
        self.proxy2 = rospy.ServiceProxy(self.service2_name, ReferenceStampedSrv)

    def latlon_to_utm(self, lat, lon, alt=0.0):
        """Convert latitude and longitude to UTM coordinates."""
        u = utm.from_latlon(lat, lon)
        return u[0], u[1], alt  # x, y, z

    def make_request(self, x, y, z, yaw=0.0, frame_id="world"):
        """Helper to build a ReferenceStampedSrvRequest."""
        # The service probably takes a field of type ReferenceStamped
        # Check the .srv file: it is probably:
        # ReferenceStamped reference
        # (other fields?) — adjust as needed.

        request = ReferenceStampedSrvRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = frame_id
        request.reference.position.x = x
        request.reference.position.y = y
        request.reference.position.z = z
        request.reference.heading = 0.0
        # If orientation / yaw is part of the message:
        request.reference.heading = yaw  # or request.reference.yaw or similar

        rospy.loginfo(f"Created request: {request}")
        rospy.loginfo(f"Request details: x={x}, y={y}, z={z}, yaw={yaw}, frame_id={frame_id}")

        return request

    def call_services(self):
        # Example values — replace with your target reference values
        lat = 41.23175554608477
        lon = -8.489968067026277

        x1, y1, z1 = self.latlon_to_utm(lat, lon)
        z1 = 10.0
        req1 = self.make_request(x1, y1, z1, yaw=0.0, frame_id="uav6/utm_navsat")
        req2 = self.make_request(x1, y1, z1, yaw=-0.0, frame_id="uav7/utm_navsat")

        try:
            rospy.loginfo("Calling service1 …")
            resp1 = self.proxy1(req1)
            rospy.loginfo(f"Service1 response: {resp1}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service1 call failed: {e}")

        try:
            rospy.loginfo("Calling service2 …")
            resp2 = self.proxy2(req2)
            rospy.loginfo(f"Service2 response: {resp2}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service2 call failed: {e}")

    def spin_loop(self, rate_hz=1.0):
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.call_services()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("reference_stamped_caller_node")
    caller = ReferenceCaller()
    caller.call_services()  # Call once immediately
