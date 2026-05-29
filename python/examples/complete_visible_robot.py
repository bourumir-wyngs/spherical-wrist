from __future__ import annotations

import sys

from spherical_wrist import visualize_robot

from _common import collision_name, create_rx160_robot, dump_solutions, translation


def main() -> None:
    # This builds the same complete RX160 used in the Rust example: six joint
    # meshes, a base mesh, a tool mesh, four static environment meshes, joint
    # constraints, and safety distances for self/environment collision checks.
    robot = create_rx160_robot()

    pose = translation(0.0, 0.0, 1.5)
    print("IK solutions for a simple target pose:")
    dump_solutions(robot.inverse(pose))

    colliding_joints = (173.0, 0.0, -94.0, 0.0, 0.0, 0.0)
    if robot.collides(colliding_joints):
        print("\nCollision detected at:", colliding_joints)
        for first, second in robot.collision_details(colliding_joints):
            print(f"  {collision_name(first)} with {collision_name(second)}")

    # Visualization opens a Bevy window. Keep this example non-interactive by
    # default, so it remains safe to run from automated environments.
    if "--visualize" not in sys.argv:
        print("\nRun this example with --visualize to open the robot window.")
        return

    initial_angles = (173.0, -8.0, -94.0, 6.0, 83.0, 207.0)
    tcp_box = ((-2.0, 2.0), (-2.0, 2.0), (1.0, 2.0))
    handle = visualize_robot(robot, initial_angles, tcp_box)
    try:
        input("Window is running. Press Enter here to close it... ")
    finally:
        handle.close()


if __name__ == "__main__":
    main()
