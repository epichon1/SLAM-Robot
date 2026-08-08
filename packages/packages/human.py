import rclpy
import traceback
from geometry_msgs.msg          import PoseStamped
from rclpy.node         import Node
from shared169.planartransform  import PlanarTransform

X_MIN = Y_MIN = 0.0
RES = 0.200
WIDTH = 200
HEIGHT = 200
X_MAX = WIDTH * RES
Y_MAX = HEIGHT * RES
SERVORATE = 200.0

class Human(Node):
    def __init__(self,name):
        super().__init__(name)
        self.last_pose = (X_MAX/2, Y_MAX/2)
        self.pubpose = self.create_publisher(PoseStamped, '/human_pose', 10)
        self.timer   = self.create_timer(1/SERVORATE, self.cb_timer)
    
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()
    
    def cb_timer(self):
        pt = PlanarTransform.basic(self.last_pose[0], self.last_pose[1], 0.0)
        pose = pt.toPose()
        posemsg = PoseStamped()
        posemsg.pose = pose 
        posemsg.header.stamp = self.get_clock().now().to_msg()
        posemsg.header.frame_id = 'grid'
        self.pubpose.publish(posemsg)

def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = Human('human')

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()