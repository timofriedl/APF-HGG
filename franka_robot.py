from frankx import Affine, LinearRelativeMotion, Robot, LinearMotion, InvalidOperationException, JointMotion


class FrankaRobot:
    def __init__(self, id="192.168.178.12"):
        self.robot = Robot(id)
        self.gripper = self.robot.get_gripper()
        self.robot.set_default_behavior()
        self.robot.velocity_rel = 1
        self.robot.acceleration_rel = 0.5
        self.robot.jerk_rel = 0.01
        self.robot.recover_from_errors()

    def move_to_init(self, initial_pos):
        self.robot.move(LinearMotion(Affine(*initial_pos, 0, 0, 0)))
        # self.robot.move(LinearMotion(Affine(0, 0, -0.03, 0, 0, 0)))

    def move_to_initial_joint(self, initial_q_pos):
        self.robot.move(JointMotion(initial_q_pos))

    def clamp(self):
        self.gripper.clamp()

    def release(self, disp=0.06):
        self.gripper.release(disp)

    def move(self, displacement):
        self.robot.move(LinearRelativeMotion(Affine(*displacement)))

    def current_pose(self):
        return self.robot.current_pose().vector()

    def current_joint_state(self):
        # Get the current state handling the read exception when the robot is in motion
        try:
            robot_state = self.robot.get_state(read_once=True)
        except InvalidOperationException:
            robot_state = self.robot.get_state(read_once=False)
        joint_pose = robot_state.q
        joint_vel = robot_state.dq
        return joint_pose, joint_vel


if __name__ == "__main__":
    initial_joints = [-1.4176983079983476, -1.4002775721633642, 1.7748181197400454, -1.976985159857231,
                      1.4911890585610361, 1.823851506586428, 0.5601574892451364]
    robot = FrankaRobot()
    cur_pose = robot.current_pose()
    cur_joint_pose, cur_joint_vel = robot.current_joint_state()

    print(cur_pose)
    print(cur_joint_pose)
    print(cur_joint_vel)

    input("Move to Initial Joint Pose")
    robot.move_to_initial_joint(initial_joints)

    cur_joint_pose, cur_joint_vel = robot.current_joint_state()

    print(cur_pose)
    print(cur_joint_pose)
    print(cur_joint_vel)
