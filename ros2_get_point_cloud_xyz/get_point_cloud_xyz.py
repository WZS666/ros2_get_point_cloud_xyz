import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class Subscriber(Node):

    def __init__(self):

        super().__init__('pc2_subscriber')

        self.subscription = self.create_subscription(PointCloud2,'camera/depth/color/points',self.callback_pointcloud,10)

    def callback_pointcloud(self,msg):
        assert isinstance(msg,PointCloud2)
        gen = point_cloud2.read_points(msg, field_names=("x","y","z"),skip_nans=True)
        for p in gen:
            print(" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))
            print("average: ",(p[0]+p[1]+p[2])/3.0)

def main(args=None):
    rclpy.init(args=args)
    pc2_subscriber = Subscriber()
    rclpy.spin(pc2_subscriber)
    pc2_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()