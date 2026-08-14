"""Small TF helpers for robust 2D robot pose lookup."""

from math import atan2, cos, sin

from rclpy.time import Time
from tf2_ros import TransformException


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def lookup_robot_xy(
    tf_buffer,
    map_frame: str,
    robot_frame: str,
    odom_frame: str,
) -> tuple[tuple[float, float] | None, bool, str | None]:
    """Return robot XY in map frame.

    First ask TF2 for the normal synchronized map->robot transform. If TF2
    cannot find a common timestamp across the chain, fall back to composing the
    newest map->odom and odom->robot transforms independently. The fallback is
    useful for SLAM pipelines where map->odom is published less frequently than
    odom->base_link.
    """
    try:
        transform = tf_buffer.lookup_transform(
            map_frame,
            robot_frame,
            Time(),
        )
        return (
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
            ),
            False,
            None,
        )
    except TransformException as direct_exc:
        if map_frame == odom_frame:
            return None, False, str(direct_exc)

        try:
            map_to_odom = tf_buffer.lookup_transform(
                map_frame,
                odom_frame,
                Time(),
            )
            odom_to_robot = tf_buffer.lookup_transform(
                odom_frame,
                robot_frame,
                Time(),
            )
        except TransformException as fallback_exc:
            return (
                None,
                False,
                f'direct={direct_exc}; split-chain={fallback_exc}',
            )

        yaw = _yaw_from_quaternion(map_to_odom.transform.rotation)
        map_odom_x = map_to_odom.transform.translation.x
        map_odom_y = map_to_odom.transform.translation.y
        odom_robot_x = odom_to_robot.transform.translation.x
        odom_robot_y = odom_to_robot.transform.translation.y

        robot_x = (
            map_odom_x
            + cos(yaw) * odom_robot_x
            - sin(yaw) * odom_robot_y
        )
        robot_y = (
            map_odom_y
            + sin(yaw) * odom_robot_x
            + cos(yaw) * odom_robot_y
        )
        return (robot_x, robot_y), True, str(direct_exc)
