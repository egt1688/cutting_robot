import time
import rclpy
from rclpy.logging import get_logger
from rclpy.clock import Clock

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

import math

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

import os
from ament_index_python.packages import get_package_share_directory

def add_attached_collision_object(robot_instance):
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle
    from geometry_msgs.msg import Pose, Point
    import trimesh

    # Get the path to the STL
    mesh_path = os.path.join(
        get_package_share_directory("cutting_robot"),
        "meshes", "Dukane_USK_41B30_62.stl"
    )

    # Load STL with trimesh
    mesh = trimesh.load(mesh_path)
    mesh.apply_scale(0.001)  # remove this if your STL is already in meters

    # Build ROS Mesh message
    ros_mesh = Mesh()
    for vertex in mesh.vertices:
        p = Point()
        p.x, p.y, p.z = float(vertex[0]), float(vertex[1]), float(vertex[2])
        ros_mesh.vertices.append(p)
    for face in mesh.faces:
        t = MeshTriangle()
        t.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
        ros_mesh.triangles.append(t)

    planning_scene_monitor = robot_instance.get_planning_scene_monitor()

    with planning_scene_monitor.read_write() as scene:
        attached_object = AttachedCollisionObject()
        attached_object.link_name = "tool0"
        attached_object.touch_links = ["tool0"]

        obj = CollisionObject()
        obj.id = "attached_tool"
        obj.header.frame_id = "tool0"

        # Position the mesh so that it is correctly oriented and positioned relative to the tool0 frame
        pose = Pose()
        pose.position.x = 0.11   # left/right relative to tool0
        pose.position.y = -0.01  # up/down relative to tool0
        pose.position.z = 0.19  # forward/back relative to tool0 (along the tool axis)

        pose.orientation.x = 0.0  
        pose.orientation.y = 0.9817477042
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0  

        obj.meshes = [ros_mesh]
        obj.mesh_poses = [pose]
        obj.operation = CollisionObject.ADD

        attached_object.object = obj
        scene.process_attached_collision_object(attached_object)
        scene.current_state.update(True)

    time.sleep(1.0)
    print("Attached mesh object added.")

def add_collision_object(robot_instance): #     
    planning_scene_monitor = robot_instance.get_planning_scene_monitor()
    planning_scene_monitor.wait_for_current_robot_state(rclpy.time.Time(), 10.0) # wait for the current robot state to be available

    object_positions = [
        (0.6096, 0.3048, 0.25), # position of the box
        # (0.4, 0.0, 0.25), # position of the box
        # (0.4, -0.3, 0.25), # position of the box
        # (0.4, 0.3, 0.5), # position of the box
    ]
    object_dimensions = [
        (0.1, 0.6096, 0.3048), # dimensions of the box
        # (0.1, 0.4, 0.1), # dimensions of the box
        # (0.2, 0.2, 0.2), # dimensions of the box
        # (0.15, 0.15, 0.15), # dimensions of the box
    ]

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "boxes"

        for position, dimensions in zip(object_positions, object_dimensions):
            box_pos = Pose()

            box_pos.position.x, box_pos.position.y, box_pos.position.z = position

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pos)

        collision_object.operation = CollisionObject.ADD
        scene.apply_collision_object(collision_object)
        scene.current_state.update(True)

    time.sleep(2.0)
    print("Objects should now be in the scene.")

def main():
    rclpy.init()
    logger = get_logger("moveit_py_planning_scene")

    abb_irb1200_5_90 = MoveItPy("abb_irb1200_5_90")
    arm = abb_irb1200_5_90.get_planning_component("manipulator")
    gripper = "tool0"
    planning_scene_monitor = abb_irb1200_5_90.get_planning_scene_monitor()
    logger.info("MoveItPy initialized")

    #add_collision_object(planning_scene_monitor)
    add_collision_object(abb_irb1200_5_90)
    add_attached_collision_object(abb_irb1200_5_90)
    arm.set_start_state(configuration_name="all_zero")
    arm.set_goal_state(configuration_name="extended")
    plan_and_execute(
        abb_irb1200_5_90,
        arm,
        logger,
        sleep_time=3.0,
    )

    # Check collisions
    with planning_scene_monitor.read_only() as scene:
        robot_state = scene.current_state
        original_joint_position = robot_state.get_joint_group_positions("manipulator")

        # Set the pose goal
        pose_goal = Pose()
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.25
        pose_goal.orientation.x = 0.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 1.0

        # Set the robot state and check for collisions
        robot_state.set_from_ik("manipulator", pose_goal, gripper)
        robot_state.update()
        robot_collision_status = scene.is_state_colliding(
            robot_state=robot_state,
            joint_model_group_name="manipulator",
            verbose=True,
        )
        logger.info(f"Robot collision status: {robot_collision_status}\n")

        # Reset the robot state
        robot_state.set_joint_group_positions(
            "manipulator", 
            original_joint_position
        )
        robot_state.update() # required to update the robot state after setting joint positions

    time.sleep(3.0)

    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="all_zero")
    plan_and_execute(
        abb_irb1200_5_90,
        arm,
        logger,
        sleep_time=3.0,
    )

if __name__ == "__main__":
    main()