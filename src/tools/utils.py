import carla
import inspect
import ctypes
import numpy as np

def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)

# Stop thread instantly
# Ref:https://blog.csdn.net/u010159842/article/details/55506011


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)

# deprecated
def xy_xyz(x, y):
    return [float(x), float(y), 5]

# deprecated
def xy_xyz_bylane(x, y):
    return [1.930856, 122.298492, 5]

# deprecated
def get_waypoint_by_location(location, world):
    return world.get_map().get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)

# deprecated
# 根据roadID和laneID获取WayPoint对象
def get_waypoint_by_mixed_laneid(map, mixed_lane_id, length):
    road_id, lane_id = mixed_lane_id.split('.')
    assert len(road_id) > 0 and len(lane_id) > 0
    waypoint = map.get_waypoint_xodr(int(road_id), int(lane_id), length)
    assert waypoint is not None
    return waypoint

# deprecated
# 根据roadID和laneID获取Location对象
def get_location_by_mixed_laneid(map, mixed_lane_id, length):
    return get_waypoint_by_mixed_laneid(map, mixed_lane_id, length).transform.location

# deprecated
# 根据roadID和laneID获取xyz坐标
def get_xyz_by_mixed_laneid(map, mixed_lane_id, length):
    location = get_location_by_mixed_laneid(map, mixed_lane_id, length)
    return [float(location.x), float(location.y), float(location.z)]

# MTL的dis
def dis(ego_vehicle_state, npc_vehicle1_ground):
    dx = ego_vehicle_state[0] - npc_vehicle1_ground[0]
    dy = ego_vehicle_state[1] - npc_vehicle1_ground[1]
    d1 = np.sqrt(np.square(dx)+np.square(dy))
    return d1

# MTL的diff
def diff(perception, ground):
    e = 0.1*dis(perception[0], ground[0]) + 0.1*dis(perception[1], ground[1]) + 0.1*dis(perception[2], ground[2])
    return e

# MTL 找出assertion发生的时间点
def get_assertion_timestamp(assertion_list):
    for i in reversed(range(len(assertion_list))):
        if assertion_list[i][1] == False:
            return assertion_list[i][0]   
    return None