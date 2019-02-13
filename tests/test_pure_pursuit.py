import utilities.pure_pursuit as pp
from utilities.pure_pursuit import PurePursuit
from hypothesis.strategies import floats
from hypothesis import given
import math


def test_trapezoidal():
    waypoints = [pp.Waypoint(0, 0, 0, 0), pp.Waypoint(10, 10, 0, 2)]
    trap = pp.insert_trapezoidal_waypoints(waypoints, 2, -2)
    assert len(trap) == 3
    assert trap[1][2] == waypoints[1][2], (
        "Intermediate waypoint should have end speed when accelerating: %s" % trap
    )

    waypoints = [pp.Waypoint(10, 10, 0, 2), pp.Waypoint(0, 0, 0, 0)]
    trap = pp.insert_trapezoidal_waypoints(waypoints, 2, -2)
    assert len(trap) == 3
    assert trap[1][2] == waypoints[0][2], (
        "Intermediate waypoint should have beginning speed when decelerating: %s" % trap
    )


@given(
    floats(min_value=0, max_value=2),
    floats(min_value=0, max_value=5),
    floats(min_value=0, max_value=5),
)
def test_find_speed(distance_along_path, start_speed, end_speed):
    pure_pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)
    start_path_distance = 0
    end_path_distance = 2

    speed = pure_pursuit.find_speed(
        start_path_distance,
        end_path_distance,
        start_speed,
        end_speed,
        distance_along_path,
    )
    if end_speed > start_speed:
        assert speed <= end_speed
        assert speed >= start_speed
    if end_speed < start_speed:
        assert speed >= end_speed - 1e-8
        assert speed <= start_speed + 1e-8


@given(floats(min_value=0, max_value=10))
def test_distance_along_path(x):
    pure_pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)
    pure_pursuit.last_robot_x = 0
    pure_pursuit.last_robot_y = 0
    robot_position = (x, 0)
    distance = pure_pursuit.distance_along_path(robot_position)

    assert math.isclose(distance, x, rel_tol=1e-8)


@given(
    floats(min_value=0, max_value=9),
    floats(min_value=-4, max_value=4),
    floats(min_value=-4, max_value=4),
)
def test_find_velocity(pos_x, start_spd, end_spd):
    pure_pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)
    pure_pursuit.last_robot_x = 0
    pure_pursuit.last_robot_y = 0
    robot_position = (pos_x, 0)
    pure_pursuit.waypoints = [
        pp.Segment(0, 0, 0, start_spd, 0),
        pp.Segment(10, 0, 0, end_spd, 10),
    ]
    vx, vy, _ = pure_pursuit.find_velocity(robot_position)
    if start_spd <= end_spd:
        assert start_spd - 1e-10 <= vx <= end_spd + 1e-10
    else:
        assert end_spd - 1e-10 <= vx <= start_spd + 1e-10
