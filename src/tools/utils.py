import carla
import inspect
import ctypes
import numpy as np
from threading import Timer

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
def dis(npc_vehicle_ground):
    # 从autoreare中收集的的关于npc、pedestrian的坐标都是相对于ego的相对坐标
    # npc从ego后方/前方/侧方撞击，应考虑ego车的bounding_box、ego车上的雷达相对于ego的位置
    ego_width = 2   #ego车宽
    ego_length = 4.54   #ego车长
    sensor_x = 2    #ego车上sensor相对于车左上角的x轴偏移
    sensor_y = 0    #ego车上sensor相对于车左上角的y轴偏移
    # 从前方撞击
    if npc_vehicle_ground[0] > 0:
        dx = npc_vehicle_ground[0] - sensor_y
    # 从后方撞击
    else:
        dx = - sensor_y - npc_vehicle_ground[0] - ego_length
    # 侧方撞击
    dy = abs(sensor_x - npc_vehicle_ground[1] - ego_width)
    return max(dx, dy)

# MTL的diff
def diff(perception, ground):
    pos_diff = np.linalg.norm(np.array(perception[0]) - np.array(ground[0]))
    heading_diff = np.linalg.norm(np.array(perception[1]) - np.array(ground[1]))
    velocity_diff = np.linalg.norm(np.array(perception[2]) - np.array(ground[2]))   
    difference = (pos_diff + heading_diff + velocity_diff)/3
    return difference

# MTL 找出assertion发生的时间点
def get_assertion_timestamp(assertion_list):
    for i in reversed(range(len(assertion_list))):
        if assertion_list[i][1] == False:
            return assertion_list[i][0]   
    return None

# 按一定时间间隔重复执行任务
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()
    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)
    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True
    def stop(self):
        self._timer.cancel()
        self.is_running = False