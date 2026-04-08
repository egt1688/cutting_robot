import time
import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def plan_and_execute(
    robot,
    planning_component,
    logger,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

# Add a collision object to the scene
def add_collision_object(planning_scene_monitor):
    object_positions = [
        (0.5, 0.0, 0.25), # position of the box
        (0.5, 0.2, 0.25), # position of the box
        (0.5, -0.2, 0.25), # position of the box
        (0.5, 0.0, 0.5), # position of the box
    ]
    object_dimensions = [
        (0.2, 0.2, 0.5), # dimensions of the box
        (0.2, 0.2, 0.5), # dimensions of the box
        (0.2, 0.2, 0.5), # dimensions of the box
        (0.2, 0.2, 0.5), # dimensions of the box
    ]

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = scene.get_planning_frame()
        collision_object.id = "boxes"

        for position, dimension in zip(object_positions, object_dimensions):
            box_pos = Pose()
            box_pos.position.x = position[0]
            box_pos.position.y = position[1]
            box_pos.position.z = position[2]

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(dimension)

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pos)
            collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update() # required to update the robot state after applying a collision object

def main():
    rclpy.init()
    logger = get_logger("motion_planning_python_planning_scene")

    abb_irb1200_5_90 = MoveItPy("abb_irb1200_5_90")
    arm = abb_irb1200_5_90.get_planning_component("arm")
    planning_scene_monitor = abb_irb1200_5_90.get_planning_scene_monitor()
    logger.info("MoveItPy initialized")

    add_collision_object(planning_scene_monitor)
    arm.set_start_state(configuration_name="ready")
    arm.set_pose_goal(configuration_name="extended")
    plan_and_execute(
        abb_irb1200_5_90,
        arm,
        logger,
        sleep_time=3.0,
    )

    # Check collisions
    with planning_scene_monitor.read_only() as scene:
        robot_state = scene.get_current_state()
        original_joint_position = robot_state.joint_state.position("arm")

        # Set the pose goal
        pose_goal = Pose()
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.4
        pose_goal.orientation.x = 0.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 1.0

        # Set the robot state and check for collisions
        robot_state.set_from_ik(arm, pose_goal, gripper)
        robot_state.update()
        robot_collision_status = scene.is_state_colliding(
            robot_state=robot_state,
            joint_model_group=arm,
            verbose=True,
        )
        logger.info(f"Robot collision status: {robot_collision_status}\n")

        # Reset the robot state
        robot_state.set_joint_group_positions(
            arm, 
            original_joint_position
        )
        robot_state.update() # required to update the robot state after setting joint positions

    time.sleep(3.0)

    with planning_scene_monitor.read_write() as scene:
        scene.remove_all_collision_objects()
        scene.current_state.update() # required to update the robot state after removing collision objects

    arm.set_start_state_to_current_state()
    arm.set_pose_goal(configuration_name="ready")
    plan_and_execute(
        abb_irb1200_5_90,
        arm,
        logger,
        sleep_time=3.0,
    )

if __name__ == "__main__":
    main()
