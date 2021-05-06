import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import PyKDL as kdl
import transformations
import json

class KDL_DKIN(Node):

    def __init__(self):
        super().__init__('KDL_DKIN')
        self.publisher_ = self.create_publisher(PoseStamped, 'kdl_fk', 10)
        self.vis = self.create_publisher(PoseStamped, 'visualisation_marker', 10)
        self.subsciber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        self.i = 0
        self.chain = self.build_chain()
        
    def listener_callback(self, msg):
        pose = self.solve_forward_kinematics(self.chain, msg, 0.1)
        self.publisher_.publish(pose)

    def get_xyz_rpy(self):
        with open("/home/piotr/anro1_ws/src/swiech_szmurlo/zadanie3/zadanie3/dh_params.json", "r") as file:
            dh_params = json.load(file)
        xyz_array = []
        rpy_array = []
        params_array = []
        for i in dh_params:
            dh_row = json.loads(json.dumps(i))
            a_translation = transformations.translation_matrix((dh_row["a"],0,0))
            d_translation = transformations.translation_matrix((0,0,dh_row["d"]))
            alpha_rotation = transformations.rotation_matrix(dh_row["alpha"],(1, 0, 0))
            theta_rotation = transformations.rotation_matrix(dh_row["theta"],(0, 0, 1))
            trans_matrix = a_translation @ alpha_rotation @ d_translation @ theta_rotation
            rpy = transformations.euler_from_matrix(trans_matrix)
            xyz = transformations.translation_from_matrix(trans_matrix)
            params_array.append({'xyz': xyz, 'rpy': rpy, 'd': dh_row['d']})
            xyz_array.append(xyz)
            rpy_array.append(rpy)
        return params_array

    def build_chain(self):
        params = self.get_xyz_rpy()
        chain = kdl.Chain()
        joint_base_1 = kdl.Joint(kdl.Joint.TransZ)
        joint_1_2 = kdl.Joint(kdl.Joint.TransY)
        joint_2_3 = kdl.Joint(kdl.Joint.TransY)
        frame0 = kdl.Frame(kdl.Rotation.RPY(params[0]['rpy'][0],params[0]['rpy'][1], 
        params[0]['rpy'][2]), kdl.Vector(params[0]['xyz'][0],params[0]['xyz'][1], params[0]['xyz'][2]))
        frame1 = kdl.Frame(kdl.Rotation.RPY(params[1]['rpy'][0],params[1]['rpy'][1],
        params[1]['rpy'][2]), kdl.Vector(params[1]['xyz'][0],params[1]['xyz'][1], params[1]['xyz'][2]))
        frame2 = kdl.Frame(kdl.Rotation.RPY(params[2]['rpy'][0],params[2]['rpy'][1],
        params[2]['rpy'][2]), kdl.Vector(params[2]['xyz'][0],params[2]['xyz'][1], params[2]['xyz'][2]))
        segment1 = kdl.Segment(joint_base_1,frame0)
        segment2 = kdl.Segment(joint_1_2,frame1)
        segment3 = kdl.Segment(joint_2_3,frame2)
        chain.addSegment(segment1)
        chain.addSegment(segment2)
        chain.addSegment(segment3)
        return chain

    def solve_forward_kinematics(self, chain, msg, tool_length):
        pose = PoseStamped()
        fk_solver = kdl.ChainFkSolverPos_recursive(chain)
        result_frame = kdl.Frame()
        joint_states = kdl.JntArray(3)
        joint_states[0] = msg.position[0]
        joint_states[1] = -msg.position[1]
        joint_states[2] = -msg.position[2]
        fk_solver.JntToCart(joint_states, result_frame)
        frame_quaternion = result_frame.M.GetQuaternion()
        tool_position = result_frame.p + kdl.Vector(tool_length,0,0)
        pose.pose.position.x = tool_position[0]
        pose.pose.position.y = tool_position[1]
        pose.pose.position.z = tool_position[2]+1
        pose.pose.orientation.x = frame_quaternion[0]
        pose.pose.orientation.y = frame_quaternion[1]
        pose.pose.orientation.z = frame_quaternion[2]
        pose.pose.orientation.w = frame_quaternion[3]
        pose.header.frame_id = "base_link"
        return pose

def main(args=None):
    rclpy.init(args=args)
    kdl_fk = KDL_DKIN()
    rclpy.spin(kdl_fk)
    kdl_fk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
