import math
import json
import roslibpy
import websockets
import asyncio


ros = None


class AutowareAdapter:
    def __init__(self, ip):
        self.adapted_ego = None
        self.ego_actor = None
        self.state_callback = None
        self.trace_callback = None
        # init ros connection
        # host需要更换为docker的ip，9090是ros bridge默认端口
        global ros
        self.ros_client = ros
        if ros is None:
            self.ros_client = roslibpy.Ros(host=ip, port=9090)
            self.ros_client.run()
            ros = self.ros_client

        #self.ros_client = roslibpy.Ros(host=ip, port=9090)
        #print('Is ROS connected?', self.ros_client.is_connected)
        # 关闭或终止都会导致下次无法通过run建立连接，所以连接不断开
      #  if self.ros_client.is_connected is False:
          #  self.ros_client.run()

        print('Is ROS connected?', self.ros_client.is_connected)
        # use websocket to control
        self.ws_uri = "ws://{}:9091".format(ip)

    def init(self):
        # init listener
        self.trace_listener = roslibpy.Topic(
            self.ros_client, '/trace', 'std_msgs/String')
        self.state_listener = roslibpy.Topic(
            self.ros_client, '/decision_maker/state_msg', 'autoware_msgs/State')
        self.EGO_LAUNCH_FLAG = False
        self.TARGET_SEND_FLAG = False
        self.EGO_REACH_FLAG = False

    def run(self, adapted_ego, state_callback, trace_callback):
        self.adapted_ego = adapted_ego
        self.state_callback = state_callback
        self.trace_callback = trace_callback
        # Bind callback function to listener
        self.trace_listener.subscribe(self.process_trace_msg)
        self.state_listener.subscribe(self.on_ego_state_change)

        # debug
        self.adapted_ego.draw_tips()

        print("Send run command")
        self.send_control_message("start to send run")
        # publish run command
        self.send_control_message("run", {
            "town": self.adapted_ego.world.get_map().name,
            "x": self.adapted_ego.start_transform.location.x,
            "y": self.adapted_ego.start_transform.location.y,
            "z": self.adapted_ego.start_transform.location.z,
            "roll": self.adapted_ego.start_transform.rotation.roll,
            "pitch": self.adapted_ego.start_transform.rotation.pitch,
            "yaw": -self.adapted_ego.start_transform.rotation.yaw# ???
        })
        print("Run command sent")
        print(adapted_ego.info())
        print("[Wait]Initializing Ego...")
        self.send_control_message("run has been sent")

    def send_target(self):
        # TODO: deal with the 'w' parameter
        print("Sending target position")
        self.send_control_message("target", {
            "position": {
                "x": self.adapted_ego.target_transform.location.x,
                "y": -self.adapted_ego.target_transform.location.y,
                "z": self.adapted_ego.target_transform.location.z
            },
            "orientation": {
                "x": self.adapted_ego.target_transform.rotation.roll,
                "y": self.adapted_ego.target_transform.rotation.pitch,
                "z": self.adapted_ego.target_transform.rotation.yaw
            }
        })

        print("Target position sent")

        self.send_control_message("target sent")

    def stop(self):
        self.send_control_message("ready to stop")

        try:
            self.trace_listener.unsubscribe()
            self.state_listener.unsubscribe()
        except Exception as exception:
            print(exception)

        self.send_control_message("stop")
        print("Stop sent")

        self.EGO_LAUNCH_FLAG = False
        self.TARGET_SEND_FLAG = False
        self.EGO_REACH_FLAG = False
        # self.ros_client.terminate()
        # self.ros_client.close()
        self.trace = []

    def ego_has_spawned(self):
        # judge whether the EGO has been created
        # by role_name set by launch config file
        if self.adapted_ego is None:
            return False
        for actor in self.adapted_ego.world.get_actors():
            if actor.type_id.split(".")[0] == "vehicle":
                if actor.attributes['role_name'] == "ego_vehicle":
                    self.ego_actor = actor
                    return True
        return False

    def ego_has_reached(self):
        delta_dist = self.distance_to_target()
        if delta_dist < 0.5:
            return True
        else:
            return False

    def distance_to_target(self):
        target_location = self.adapted_ego.target_transform.location
        ego_location = self.ego_actor.get_location()
        delta_dist = math.sqrt(
            ((target_location.x-ego_location.x)**2)+((target_location.y-ego_location.y)**2))
        return delta_dist

    def on_ego_state_change(self, message):
        vehicle_state = message["vehicle_state"].strip()
        mission_state = message["mission_state"].strip()
        behavior_state = message["behavior_state"].strip()

        if vehicle_state == "VehicleReady" and self.EGO_LAUNCH_FLAG == False and self.ego_has_spawned():
            self.EGO_LAUNCH_FLAG = True
            self.state_callback("READY")

        if mission_state == "Driving" and self.TARGET_SEND_FLAG == False and self.EGO_LAUNCH_FLAG == True:
            self.TARGET_SEND_FLAG = True
            self.send_control_message("get Driving state")
            self.state_callback("DRIVING")

        if behavior_state == "Stopping" and self.EGO_REACH_FLAG == False and self.EGO_LAUNCH_FLAG == True and self.TARGET_SEND_FLAG == True:
            self.EGO_REACH_FLAG = True
            self.send_control_message("get Stopping state")
            self.state_callback("STOP")

    def process_trace_msg(self, message):
        data = message['data']
        data = json.loads(data)
        self.trace_callback(data)

    # deprecated
    def send_control_message_block(self, cmd, data={'data': None}):
        request = roslibpy.ServiceRequest(
            {
                "cmd": cmd,
                "data": json.dumps(data)
            }
        )

        result = self.ros_manager.call(request, timeout=10)

        print("[{}]:{}".format(cmd, result))

    def send_control_message(self, cmd, data={'data': None}):
        msg = json.dumps(
            {
                "cmd": cmd,
                "data": data
            }
        )
        # self.send_to_ws(msg)
        # asyncio.create_task(self.send_to_ws(msg))
        asyncio.run(self.send_to_ws(msg))

    async def send_to_ws(self, msg):
        # TODO: optimize the call of connect()
        async with websockets.connect(self.ws_uri) as websocket:
            await websocket.send(msg)
            greeting = await websocket.recv()
            greeting = json.loads(greeting)
            print(greeting)
