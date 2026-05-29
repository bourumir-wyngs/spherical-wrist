from __future__ import annotations

from spherical_wrist import visualize_robot

from _common import (
    DEFAULT_TCP_BOX,
    collision_name,
    create_rx160_robot,
    dump_solutions,
    translation,
    wait_for_visualization,
)


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

    initial_angles = (173.0, -8.0, -94.0, 6.0, 83.0, 207.0)
    handle = visualize_robot(robot, initial_angles, DEFAULT_TCP_BOX)
    try:
        wait_for_visualization(handle)
    finally:
        handle.close()


if __name__ == "__main__":
    main()
