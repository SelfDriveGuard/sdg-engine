import os
import threading
from src.main import autoware_adapter
from src.main import scenest_carla_adapter
from src.main.scenest_carla_adapter import ScenestCarlaAdapter
from src.main.scenic_carla_adapter import ScenicCarlaAdapter
from src.main.carla_adapter import AdaptedVehicle
from src.scenest_parser.ast import driver
from src.scenest_parser.ast.ast import ASTDumper
from src.scenest_parser.ast.error.error import IllegalTypeException
from src.scenest_parser.ast.assertion.assertion import AgentVisibleDetectionAssertion, AgentErrorDetectionAssertion, TrafficDetectionAssertion, AgentSafetyAssertion
from src.tools import utils
from src.scenic_parser import parser as scenic_parser
from src.tools.auto_criteria import CriteriaManager
from src.tools.utils import RepeatedTimer
import numpy as np
import mtl


class Engine(threading.Thread):
    def __init__(self, code_file, callback, map_name=None,language='scenest', time_limit=-1, is_load_map=False, start_event=None, stop_event=None):
        threading.Thread.__init__(self)
        self.code_file = code_file
        self.carla_adapter = None
        self.autoware_adapter = None
        self.trace = []
        self.scenenario_list = None
        self.scenenario_index = -1
        self.current_scenenario = None
        self.trace_mock = []
        self.trace_modified = []
        self.ast = None
        self.assertion = []
        self.callback = callback

        # time limit
        self.time = time_limit
        self.time_count_thread = None

        self.map_name = map_name
        self.is_load_map = is_load_map

        # 启动停止控制
        self.start_event = start_event
        self.stop_event = stop_event

        # set language
        self.language = language
        print("Using {}".format(self.language))

        # 评分
        self.criteria_manager = None
 
    def run(self):
        if os.environ.get("CARLA_SERVER_IP") == None:
            os.environ["CARLA_SERVER_IP"] = "127.0.0.1"
        if os.environ.get("ROS_BRIDGE_IP") == None:
            os.environ["ROS_BRIDGE_IP"] = "127.0.0.1"
        self.autoware_adapter = autoware_adapter.AutowareAdapter(
            os.environ.get("ROS_BRIDGE_IP"))

        if self.language == "scenest":
            self.carla_adapter = ScenestCarlaAdapter(
                os.environ.get("CARLA_SERVER_IP"))
        if self.language == "scenic":
            self.carla_adapter = ScenicCarlaAdapter(
                os.environ.get("CARLA_SERVER_IP"))

        # 实例化carla_adapter后，实例化criteria_manager
        # self.criteria_manager = CriteriaManager(self.carla_adapter, interval=1)

        # 前端指令切换地图
        if self.is_load_map:
            print("change map")
            self.carla_adapter.set_map(self.map_name)
            # 发送状态信息给前端页面
            self.callback(cmd="STOP", msg="map has load")
            return
        print("engine thread start, thread id = " + str(threading.get_ident()))
        self.start_event.wait()
        print("engine start test")
        self.start_test()
        
        self.stop_event.wait()
        print("caught stop event")
        self.stop()
        print("kill engine thread")

    def change_view(self, code):       
        self.carla_adapter.set_spectator_transform(code)

    def start_test(self):

        # Spectator
        # self.carla_adapter.set_spectator()

        if self.language == "scenest":
            try:
                # 前端指令提交代码（直接提交代码没有预加载地图/已经预加载地图）
                self.ast = driver.Parse(self.code_file)
            except IllegalTypeException as typerr:
                print (str(typerr))
                # 发送状态信息给前端页面
                self.callback(cmd="STOP", msg=str(typerr))
            except Exception as err:
                print (str(err))
                # 发送状态信息给前端页面
                self.callback(cmd="STOP", msg=str(err))

            # 如果想看一眼ast解析结果，可以取消下方注释
            # dumper = ASTDumper(self.ast)
            # dumper.dump()
            else:
                # get scenenario
                scenenario_list = self.ast.get_scenarios()
                self.scenenario_list = scenenario_list
                self.scenenario_index = -1
                self.__start_next_scenenario(self.map_name)

        if self.language == "scenic":
            print("using scenic")

            params = {}
            params["address"] = os.environ.get("CARLA_SERVER_IP")
            scenario = scenic_parser.parse(self.code_file, params)

            self.carla_adapter.init(scenario, self.map_name)
            # Spectator
            # self.carla_adapter.set_spectator()
            
            # get ego object, run autoware
            ego_object = self.carla_adapter.get_av_ego()
            if ego_object is not None:
                print("scenic has av ego")
                ego_start_coordinates = list(ego_object.position.coordinates)
                # TODO: fix y-axis problem: scenic use opposite y
                ego_start_coordinates[1] = -ego_start_coordinates[1]
                adapted_ego = AdaptedVehicle(
                    world=self.carla_adapter.world, name="ego_vehicle")
                adapted_ego.set_start_position(ego_start_coordinates)
                adapted_ego.set_random_target()
                print("ego start:{}".format(adapted_ego.start_transform))
                self.autoware_adapter.init()
                self.autoware_adapter.run(
                    adapted_ego, self.on_ego_state_change, self.on_trace_generated)
            else:
                self.carla_adapter.run()

    def __start_next_scenenario(self, map_name=None):
        self.scenenario_index = self.scenenario_index + 1
        if self.scenenario_index < len(self.scenenario_list):
            self.current_scenenario = self.scenenario_list[self.scenenario_index]
            if not map_name:
                # 前端指令提交代码（直接提交代码没有预加载地图)
                self.carla_adapter.set_map(
                    self.current_scenenario.get_map().get_map_name())
            else:
                self.carla_adapter.set_map(map_name)

            # Spectator
            self.carla_adapter.set_spectator()
            # Create NPCs early
            self.carla_adapter.init(self.current_scenenario)

            # Adapte ego
            ego = self.current_scenenario.get_ego_vehicle()
            adapted_ego = AdaptedVehicle(
                world=self.carla_adapter.world, name=ego.get_name())
            scenest_carla_adapter.set_position(ego, adapted_ego)
            # Run autoware adapter
            self.autoware_adapter.init()
            self.autoware_adapter.run(
                adapted_ego, self.on_ego_state_change, self.on_trace_generated)

        else:
            print("finish all scenenario_list")

    def stop(self):
        if self.time_count_thread is not None:
            self.time_count_thread.cancel()

        # stop infomation collector
        self.autoware_adapter.adapted_ego.stop_collect()
        infomation_dict = self.autoware_adapter.adapted_ego.infomation_dict
        infomation_dict_return = {
            "throttle": infomation_dict["throttle"],
            "steering": infomation_dict["steer"],
            "xDyPosition": [l.x for l in infomation_dict["location"]],
            "yDyPosition": [l.y for l in infomation_dict["location"]]
        }
        self.callback(cmd="RES", msg=infomation_dict_return)

        # 发送状态信息给前端页面
        self.callback(cmd="STOP",msg="Ego reached target")

        if self.language == "scenest":
            trace_list = self.ast.get_traces()
            self.check_assertion(trace_list)
            # 发送assert信息给前端页面
            self.callback(cmd="ASSERT", msg=self.assertion)

        # 评分模块停止记分并打分
        # 若需查看记录的违规Event的列表，取消下方注释
        # self.criteria_manager.get_global_event_report()
        # global_statistics = self.criteria_manager.compute_global_statistics()
        # self.criteria_manager.stop()
        # self.callback(cmd="CRITERIA", msg=global_statistics)

        # 结束arla_adapter和autoware_adapter
        print("Stoping engine")
        if self.autoware_adapter.ego_has_spawned():
            self.autoware_adapter.stop()
        self.carla_adapter.stop()
        self.trace = []
        self.time = -1
        self.time_count_thread = None

    # TODO: make message constant
    # state:
    #   "READY": ego has been initialized
    #   "DRIVING": target has been sent and processed, ego starts driving
    #   "STOP": ego stop

    def on_ego_state_change(self, state):
        if state == "READY":
            print("Ego launched")
            # 发送状态信息给前端页面
            self.callback(cmd="READY", msg="Ego launched")

            # Send target
            self.autoware_adapter.send_target()
            print("[Wait]Checking target...")
        elif state == "DRIVING":
            print("Ego start to drive")
            print("Start to create others")
            # 发送状态信息给前端页面
            self.callback(cmd="DRIVING", msg="Ego start to drive")
            # check time limit
            if self.time > 0:
                print("Time limit:{}s".format(self.time))
                self.time_count_thread = threading.Timer(interval = self.time, function = self.stop)
                self.time_count_thread.start()

            # start to collect ego infomation
            self.autoware_adapter.adapted_ego.start_to_collect()

            ## create criteria and registry to criteria_manager
            # criteria = self.autoware_adapter.adapted_ego.create_criterias()
            # self.criteria_manager.registry_criterias(criteria)
            ## start to collect infomation and evaluate
            #self.criteria_manager.start_to_evaluate()

            # create other elements after EGO has been launched
            self.carla_adapter.run()
        elif state == "STOP":
            print("Ego reached")
            self.stop()

    def on_trace_generated(self, trace):
        self.trace.append(trace)

    def check_assertion(self, trace_list):
        print("Check Assertion")
        self.assertion = []
        # trace数据转换
        self.__change_trace_key()

        # 按时间戳取出每一时刻的trace
        for trace in trace_list:
            detection_assertions = trace.get_detection_assertions()
            safety_assertions = trace.get_safety_assertions()
            intersection_assertion = trace.get_intersection_assertions()
            speed_constraint_assertion = trace.get_speed_constraint_assertions()
            # DetectionAssertions
            for item in detection_assertions:
                detection_assertion = item.get_assertions()
                assertion_result = self.__set_detection_assertion(
                    detection_assertion)
                if assertion_result:
                    self.assertion.append(assertion_result)
            # SafetyAssertions
            for item in safety_assertions:
                safety_assertion = item.get_assertions()
                assertion_result = self.__set_safety_assertion(
                    safety_assertion)
                if assertion_result:
                    self.assertion.append(assertion_result)
            # IntersectionAssertion
            for item in intersection_assertion:
                assertion_result = self.__set_intersection_assertion(item)
                if assertion_result:
                    self.assertion.append(assertion_result)
            # Speed
            for item in speed_constraint_assertion:
                assertion_result = self.__set_speed_constraint_assertion(item)
                if assertion_result:
                    self.assertion.append(assertion_result)

    def __change_trace_key(self):
        # 模拟trace数据
        self.trace_mock = [
            {'time': "118645426470116",
             'ego': [[108.08795928955078, -62.55291748046875], [2.259624168756988e-06, -0.0003826158057775823, -0.0011180206623436829, 0.9999993018146753], [0.9709860741011317, 0.0], [1.477900743484497, 0.010906191542744637]],
             'perception': {
                 "0": [[25.72591779055648, 3.572339975132998], [5.854146406845858e-20, -9.006379087455166e-21, -0.6587237439018757, 0.7523848943326121], [0.0, 0.0], [1.973646785717675, 1.8194498129931729, 2.0]],
                 "traffic": [100, 200]
             },
             "truth": {
                 "traffic": [100, 200],
                 "npc_522": [[28.237632751464844, 3.5545578002929688], [-0.015128828559103552, 0.0150291483727307, -0.9992509269240311, 0.032292851950311424], [-0.3341793715953827, -0.8122221827507019], [4.513522624969482, 2.006814479827881, 1.5248334407806396]],
                 "npc_524": [[27.31446075439453, 3.51800537109375], [0.13789920727354132, -0.030677308180103374, -0.9899020739017735, 0.011687406945917848], [-0.7569416761398315, -0.43403083086013794], [5.0267767906188965, 2.1515462398529053, 1.6355280876159668]],
                 "npc_525": [[-102.70937776565552, -51.47315216064453], [0.0, 0.0, 0.7093403609582224, 0.7048661236828301], [0.0, -0.0], [4.181210041046143, 1.9941171407699585, 1.385296106338501]],
                 "pedestrian_526": [[-89.08795928955078, 49.55291748046875], [0.0, -0.0, 0.0, 1.0], [0.0, -0.0], [0.6800000071525574, 0.6800000071525574, 1.8600000143051147]]
             }
             }, {'time': "118645826470121",
                 "ego": [[108.5118408203125, -62.549713134765625], [0.00018886769043826584, -0.00026104860027867155, -0.0009507881825217205, 0.9999994960921008], [1.3921367866978784, 0.0], [1.3272786140441895, 0.010134805925190449]],
                 'perception': {
                     "1": [[24.829751437016583, 3.1287125882765707], [-6.365046639273121e-22, -1.3578766163782657e-20, 0.06213676548264415, 0.9980676441881857], [0.0, 0.0], [1.3974641291772167, 1.3064991412609017, 2.0]],
                     "traffic": [100, 200]
                 },
                 "truth": {
                     "traffic": [100, 200],
                     "npc_522": [[27.615310668945312, 3.4910659790039062], [-0.008925140889005466, 0.018965299267674846, -0.9984970409819726, 0.050639099854200696], [-0.4103608727455139, 0.24971553683280945], [4.513522624969482, 2.006814479827881, 1.5248334407806396]],
                     "npc_524": [[26.57269287109375, 3.404956817626953], [0.13391539365163754, -0.02926021175281756, -0.9884210191766308, 0.06507223832893362], [-0.8205177187919617, -0.20952603220939636], [5.0267767906188965, 2.1515462398529053, 1.6355280876159668]],
                     "npc_525": [[-103.13325929641724, -51.476356506347656], [0.0, 0.0, 0.7093403609582224, 0.7048661236828301], [0.0, -0.0], [4.181210041046143, 1.9941171407699585, 1.385296106338501]],
                     "pedestrian_526": [[-86.74788284301758, 48.19402027130127], [0.0, 0.0, 0.17013631681159033, 0.9854205364725185], [0.028503574430942535, 0.007660765666514635], [0.6800000071525574, 0.6800000071525574, 1.8600000143051147]]
                 }
                 }, {'time': "118646126470126",
                     "ego": [[108.98670196533203, -62.55208206176758], [5.200484223220141e-05, -0.00025316735979103203, -0.0004354705184940739, 0.9999998717835976], [1.9731128729453935, 0.0], [1.3841056823730469, 0.0033290754072368145]],
                     'perception': {
                         "2": [[24.369866387952257, 2.785100269052336], [-2.877460203309997e-20, -1.534270772468026e-20, 0.9419784253126473, -0.3356734220123859], [0.0, 0.0], [2.2828353093088274, 1.068360543031982, 2.0]],
                         "traffic": [100, 200]
                     },
                     "truth": {
                         "traffic": [100, 200],
                         "npc_522": [[26.817283630371094, 3.730052947998047], [0.02166274215031435, 0.002614662617010013, -0.9846159190121248, 0.17336487870914724], [-0.863123893737793, 0.09216350317001343], [4.513522624969482, 2.006814479827881, 1.5248334407806396]],
                         "npc_524": [[25.88495635986328, 3.4266395568847656], [0.14595944463658497, 0.042127677714575845, -0.9717481122517698, 0.18062864010692894], [-0.3361729681491852, 0.4385520815849304], [5.0267767906188965, 2.1515462398529053, 1.6355280876159668]],
                         "npc_525": [[-103.6081280708313, -51.47390365600586], [-4.799219498621745e-05, -0.00020761099173416587, 0.7093403774493027, 0.704866074878417], [0.00016746658366173506, 1.131492626882391e-05], [4.181210041046143, 1.9941171407699585, 1.385296106338501]],
                         "pedestrian_526": [[-86.68246841430664, 48.457353591918945], [0.0, 0.0, 0.21689414132610285, 0.9761951298067475], [0.1101672500371933, 0.041900910437107086], [0.6800000071525574, 0.6800000071525574, 1.8600000143051147]]
                     },
                     }]

        # 按时间戳遍历
        for item in self.trace:

            # 按照id_name_map，修改ground字典的key
            ground = {}
            for ground_key in item['truth']:
                # 按照约定，截取"_"符号后的字符串为actor.id
                ground_key_id = ground_key[ground_key.rfind('_')+1:]
                if self.carla_adapter.id_name_map_has(ground_key_id):
                    ground[self.carla_adapter.id_corresponding_name(
                        ground_key_id)] = item['truth'][ground_key]
                else:
                    ground[ground_key] = item['truth'][ground_key]

            # 按照欧氏距离，修改perception字典的key
            perception = {}
            for perception_key in item['perception']:
                # 取出字典perception_key对应的值
                perception_item = item['perception'][perception_key]
                optimal_key = None
                optimal_dis = -1
                perception_array = []

                # npc类型/pedestrian类型
                if len(perception_item) == 4:
                    # 将perception_item从tuple转为numpy.array
                    # perception_array = np.array(perception_item, dtype=object)
                    perception_array = np.zeros([len(perception_item), len(
                        max(perception_item, key=lambda x: len(x)))])
                    for i, j in enumerate(perception_item):
                        perception_array[i][0:len(j)] = j

                    # 遍历ground字典，根据欧式距离匹配
                    for ground_key in ground:
                        # 取出字典中ground_key对应的值
                        ground_item = ground[ground_key]

                        # 元素长度一样，即：同为npc类型/pedestrian类型
                        if len(ground_item) == 4:
                            ground_array = np.array(ground_item, dtype=object)
                            ground_array = np.zeros([len(ground_item), len(
                                max(ground_item, key=lambda x: len(x)))])
                            for i, j in enumerate(ground_item):
                                ground_array[i][0:len(j)] = j

                            # print(ground_array, perception_array)
                            dis = np.linalg.norm(
                                ground_array - perception_array)
                            # print(dis)
                            # 计算得到的dis在容差范围内，且比先前计算得到的dis更近，则记录
                            if dis < 100 and (optimal_dis < 0 or optimal_dis > dis):
                                optimal_key = ground_key
                                optimal_dis = dis

                # traffic类型
                elif len(perception_item) == 2:
                    # 遍历ground字典，根据欧式距离匹配
                    for ground_key in ground:
                        ground_item = ground[ground_key]
                        # 元素长度一样，即：同为traffic类型
                        if len(ground_item) == 2:
                            optimal_key = ground_key

                if optimal_key:
                    perception[optimal_key] = perception_item

            self.trace_modified.append({
                'time': int(item['time']),
                'ego': item['ego'],
                'perception': perception,
                'truth': ground
            })

        # print(self.trace)
        # print(self.trace_modified)

    def __set_detection_assertion(self, detections):
        # 从parser中取出变量
        trace_time = 0  # 默认为0
        dis_agent_ground = ''  # 默认为空
        sensing_range = 0.0  # 默认为0

        diff_agent_state = ''  # 默认为空
        diff_agent_ground = ''  # 默认为空
        error_threshold = 0.0  # 默认为0

        description = ''  # 描述此类错误
        for detection in detections:
            if isinstance(detection, AgentVisibleDetectionAssertion):
                # 取出 dis(trace[1][ego], trace[1][truth][npc1])中trace[1][truth][npc1]的 1 -> 此处存疑：trace_time是否只取一次并全局使用
                # TODO: trace_time是否只取一次并全局使用
                trace_time = detection.get_agent_ground_distance(
                ).get_agent_ground_truth().get_trace_time().get_time()
                # 取出 dis(trace[1][ego], trace[1][truth][npc1])中trace[1][truth][npc1]的 npc1
                dis_agent_ground = detection.get_agent_ground_distance(
                ).get_agent_ground_truth().get_agent().get_name()
                # 取出 dis()<= 0.1 的 0.1
                sensing_range = detection.get_sensing_range()
                # 描述此类错误
                description += str(detection)
            elif isinstance(detection, AgentErrorDetectionAssertion):
                # 取出 diff(trace[1][perception][npc1], trace[1][truth][npc1])中trace[1][perception][npc1]的 npc1
                diff_agent_state = detection.get_agent_error(
                ).get_agent_state().get_agent().get_name()
                # 取出 diff(trace[1][perception][npc1], trace[1][truth][npc1])中trace[1][truth][npc1]的 npc1
                diff_agent_ground = detection.get_agent_error(
                ).get_agent_ground_truth().get_agent().get_name()
                # 取出 diff()<= 0.1 的 0.1
                error_threshold = detection.get_error_threshold()
                # 描述此类错误
                description += ' & {}'.format(detection)
            elif isinstance(detection, TrafficDetectionAssertion):
                pass
        print(trace_time, dis_agent_ground, sensing_range,
              diff_agent_state, diff_agent_ground, error_threshold)
        # print(self.trace_modified)
        # 调用MTL
        trace_mtl = {
            'a': [],
            'b': []
        }
        for item in self.trace_modified:
            # 防止出现：trace_time指定为从0开始，但trace_modified的第一个时间戳晚于0
            if item['time'] >= trace_time and dis_agent_ground in item['truth'] and diff_agent_state in item['perception'] and diff_agent_ground in item['truth']:
                # dis
                trace_mtl['a'].append((item['time'], utils.dis(item['truth'][dis_agent_ground][0]) <= sensing_range))
                # diff
                trace_mtl['b'].append((item['time'], utils.diff(
                    item['perception'][diff_agent_state], item['truth'][diff_agent_ground]) <= error_threshold))
        # print(trace_mtl)
        # 若trace中没有提取出与用户订阅相匹配的数据，则返回true
        for item in trace_mtl:
            if len(trace_mtl[item]) == 0:
                print('DetectionAssertions: None')
                return

        phi = mtl.parse('G(a&b)')
        assertion_result_list = phi(trace_mtl, time=None, quantitative=False)

        # 整合返回数据结构
        timestamp = utils.get_assertion_timestamp(assertion_result_list)
        if timestamp:
            assertion_result = {
                'type': 'DetectionAssertion',
                'timestamp': timestamp,
                'description': description
            }
            return assertion_result

    def __set_safety_assertion(self, safeties):
        # 从parser中取出变量
        trace_time = 0  # 默认为0
        dis_agent_ground = ''  # 默认为空
        sensing_range = 0.0  # 默认为0

        diff_agent_state = ''  # 默认为空
        diff_agent_ground = ''  # 默认为空
        error_threshold = 0.0  # 默认为0

        safety_agent_state = ''  # 默认为空
        safety_radius = 0.0  # 默认为0

        description = ''  # 描述此类错误
        for safety in safeties:
            if isinstance(safety, AgentVisibleDetectionAssertion):
                # 取出 dis(trace[1][ego], trace[1][truth][npc1])中trace[1][truth][npc1]的 1 -> 此处存疑：trace_time是否只取一次并全局使用
                trace_time = safety.get_agent_ground_distance(
                ).get_agent_ground_truth().get_trace_time().get_time()
                # 取出 dis(trace[1][ego], trace[1][truth][npc1])中trace[1][truth][npc1]的 npc1
                dis_agent_ground = safety.get_agent_ground_distance(
                ).get_agent_ground_truth().get_agent().get_name()
                # 取出 dis()<= 0.1 的 0.1
                sensing_range = safety.get_sensing_range()
                # 描述此类错误
                description += str(safety)
            elif isinstance(safety, AgentErrorDetectionAssertion):
                # 取出 diff(trace[1][perception][npc1], trace[1][truth][npc1])中trace[1][perception][npc1]的 npc1
                diff_agent_state = safety.get_agent_error().get_agent_state().get_agent().get_name()
                # 取出 diff(trace[1][perception][npc1], trace[1][truth][npc1])中trace[1][truth][npc1]的 npc1
                diff_agent_ground = safety.get_agent_error(
                ).get_agent_ground_truth().get_agent().get_name()
                # 取出 diff()<= 0.1 的 0.1
                error_threshold = safety.get_error_threshold()
                # 描述此类错误
                description += ' & {}'.format(safety)
            elif isinstance(safety, AgentSafetyAssertion):
                # 取出 dis(trace[1][ego], trace[1][perception][npc1])中trace[1][perception][npc1]的 npc1
                safety_agent_state = safety.get_agent_state().get_agent().get_name()
                # 取出 dis() >= 0.1 的0.1
                safety_radius = safety.get_safety_radius()
                # 描述此类错误
                description += ' & {}'.format(safety)

        print(trace_time, dis_agent_ground, sensing_range, diff_agent_state,
              diff_agent_ground, error_threshold, safety_agent_state, safety_radius)

        # 调用MTL
        trace_mtl = {
            'a': [],
            'b': [],
            'c': []
        }
        for item in self.trace_modified:
            # 防止出现：trace_time指定为从0开始，但trace_modified的第一个时间戳晚于0
            if item['time'] >= trace_time and dis_agent_ground in item['truth'] and diff_agent_state in item['perception'] and diff_agent_ground in item['truth'] and safety_agent_state in item['perception']:
                # dis
                trace_mtl['a'].append((item['time'], utils.dis(item['truth'][dis_agent_ground][0]) <= sensing_range))
                # diff
                trace_mtl['b'].append((item['time'], utils.diff(
                    item['perception'][diff_agent_state], item['truth'][diff_agent_ground]) <= error_threshold))
                # SafetyAssertion
                trace_mtl['c'].append((item['time'], utils.dis(item['perception'][safety_agent_state][0]) >= safety_radius))

        # 若trace中没有提取出与用户订阅相匹配的数据，则返回true
        for item in trace_mtl:
            if len(trace_mtl[item]) == 0:
                print('SafetyAssertions: None')
                return

        phi = mtl.parse('G(a&b&c)')
        assertion_result_list = phi(trace_mtl, time=None, quantitative=False)

        # 整合返回数据结构
        timestamp = utils.get_assertion_timestamp(assertion_result_list)
        if timestamp:
            assertion_result = {
                'type': 'SafetyAssertion',
                'timestamp': timestamp,
                'description': description
            }
            return assertion_result

    def __set_intersection_assertion(self, intersection):
        # 从parser中取出变量
        # 取出 trace[1][traffic]==green 的 1 -> 此处存疑：trace_time是否只取一次并全局使用
        trace_time = intersection.get_green_light_state().get_trace_time().get_time()
        # print(trace_time)
        # 调用MTL
        trace_mtl = {
            'a': [],
            'b': [],
            'c': [],
            'd': []
        }

        for item in self.trace_modified:
            # 防止出现：trace_time指定为从0开始，但trace_modified的第一个时间戳晚于0
            if item['time'] >= trace_time:
                trace_mtl['a'].append(
                    (item['time'], item['truth']['traffic'] == item['perception']['traffic']))
                trace_mtl['b'].append(
                    (item['time'], item['perception']['traffic'] == (100, 200)))  # 判断红灯
                trace_mtl['c'].append(
                    (item['time'], np.linalg.norm(item['ego'][2])))
                trace_mtl['d'].append(
                    (item['time'], item['perception']['traffic'] == (100, 100)))  # 判断绿灯

        # 若trace中没有提取出与用户订阅相匹配的数据，则返回true
        for item in trace_mtl:
            if len(trace_mtl[item]) == 0:
                print('IntersectionAssertion: None')
                return

        phi = mtl.parse('G((a&b)->(~c U (a&d)))')
        assertion_result_list = phi(trace_mtl, time=None, quantitative=False)

        # 整合返回数据结构
        timestamp = utils.get_assertion_timestamp(assertion_result_list)
        if timestamp:
            assertion_result = {
                'type': 'IntersectionAssertion',
                'timestamp': timestamp,
                'description': str(intersection)
            }
            return assertion_result

    def __set_speed_constraint_assertion(self, speed):
        # 从parser中取出变量
        # 取出 trace[1][perception][traffic]==trace[1][truth][traffic]中trace[1][perception][traffic]的 1 -> 此处存疑：trace_time是否只取一次并全局使用
        trace_time = speed.get_traffic_detection().get_left_trace_time().get_time()
        # 取出 ->F[0,2] 中 的2
        time_duration = speed.get_time_duration()
        # print(trace_time, time_duration)
        # 调用MTL
        trace_mtl = {
            'a': [],
            'b': [],
            'c': [],
            'd': []
        }

        for item in self.trace_modified:
            # 防止出现：trace_time指定为从0开始，但trace_modified的第一个时间戳晚于0
            if item['time'] >= trace_time:
                trace_mtl['a'].append(
                    (item['time'], item['truth']['traffic'] == item['perception']['traffic']))
                trace_mtl['b'].append(
                    (item['time'], item['perception']['traffic'] == (0, 2)))  # 检查速度范围
                trace_mtl['c'].append((item['time'], np.linalg.norm(
                    item['ego'][2]) < item['perception']['traffic'][0]))  # 低速检查
                trace_mtl['d'].append((item['time'], np.linalg.norm(
                    item['ego'][2]) > item['perception']['traffic'][1]))  # 超速检查

        # 若trace中没有提取出与用户订阅相匹配的数据，则返回true
        for item in trace_mtl:
            if len(trace_mtl[item]) == 0:
                print('Speed Constraint: None')
                return

        phi = mtl.parse(
            'G((a&b&(c|d))->(F[0,{}]~(c|d)))'.format(str(time_duration)))
        assertion_result_list = phi(trace_mtl, time=None, quantitative=False)

        # 整合返回数据结构
        timestamp = utils.get_assertion_timestamp(assertion_result_list)
        if timestamp:
            assertion_result = {
                'type': 'Speed Constraint',
                'timestamp': timestamp,
                'description': str(speed)
            }
            return assertion_result
