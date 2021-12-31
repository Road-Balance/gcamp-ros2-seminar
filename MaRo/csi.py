import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 
def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class CISImageNode(Node):

  def __init__(self):
    super().__init__('csi_image_node')

    self._publisher = self.create_publisher(Image, 'csi_image', 10)

    timer_period = 0.1  # seconds
    self._timer = self.create_timer(timer_period, self.timer_callback)

    self._cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    self._cv_bridge = CvBridge()

  def timer_callback(self):

    ret, frame = self._cap.read()
          
    if ret == True:
      # image to a ROS 2 image message
      self._publisher.publish(self._cv_bridge.cv2_to_imgmsg(frame))
 
    # Display the message on the console
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  rclpy.init(args=args)
  
  image_publisher = CISImageNode()
  rclpy.spin(image_publisher)

  image_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()