from dwa import choose_dwa_command, lidar_obstacles


def test_dwa_returns_forward_motion_for_forward_goal():
    result = choose_dwa_command((0.0, 3.0), [], 0.0, max_speed=0.2)
    assert not result.blocked
    assert result.speed_mps >= 0
    assert result.trajectory


def test_dwa_stops_when_all_paths_blocked_nearby():
    # dense wall across the robot's forward corridor
    wall = [(x / 100.0, 0.15) for x in range(-80, 81, 5)]
    result = choose_dwa_command((0.0, 2.0), wall, 0.0, max_speed=0.2)
    assert result.blocked or result.speed_mps == 0.0


def test_lidar_ranges_stay_in_millimeters_and_convert_to_meters():
    pts = lidar_obstacles([1000], angle_offset_deg=0, max_range_mm=4000)
    assert len(pts) == 1
    assert abs(pts[0][1] - 1.0) < 1e-6
