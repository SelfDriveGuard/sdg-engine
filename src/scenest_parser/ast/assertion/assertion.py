# The file defines assertions needed to meet with the trace
from src.scenest_parser.ast.scenario.scenario import Scenario
from src.scenest_parser.ast.npc.npc_vehicles import NPCVehicle
from src.scenest_parser.ast.pedestrian.pedestrians import Pedestrian
from src.scenest_parser.ast.obstacle.obstacles import Obstacle
from src.scenest_parser.ast.base.state import Coordinate,Speed,Variable,NodeType,Node
from src.scenest_parser.ast.traffic.traffic import SpeedRange
from typing import Union,AnyStr,Optional,List
# Denote a trace instantaneous description
# such as `time=trace_name[1]`
## TODO: __str__ for pretty dumping?
class Trace:
	...
class TraceTime:
	def __init__(self,trace:Trace,time:int):
		self._trace:Trace=trace
		self._time:int=time
	def get_trace(self)->Trace:
		return self._trace
	def get_time(self)->int:
		return self._time
	def __str__(self):
		# Trace definitely has a name.
		return f'{self._trace.get_name()}[{self._time}]'
class EgoState(Variable,Node):
	def __init__(self,trace_time:TraceTime,name:AnyStr=None):
		Variable.__init__(self,name)
		Node.__init__(self,NodeType.T_EGOSTATE)
		self._trace_time:TraceTime=trace_time
	def get_trace_time(self)->TraceTime:
		return f'{self._trace_time}'
	def __str__(self):
		return f'{str(self._trace_time)}[ego]'
class RedLightState:
	def __init__(self,trace_time:TraceTime):
		self._trace_time:TraceTime=trace_time
	def get_trace_time(self)->TraceTime:
		return self._trace_time
	def __str__(self):
		return f'{str(self._trace_time)}[traffic]==red'
class EgoSpeed:
	def __init__(self):
		self._velocity=None
	def set_velocity(self,coordinate:Coordinate):
		self._velocity=coordinate
	def get_velocity(self)->Coordinate:
		return self._velocity
	def __str__(self) -> str:
		return f'norm({self._velocity.get_x(),self._velocity.get_y()})'
class GreenLightState:
	def __init__(self,trace_time:TraceTime):
		self._trace_time:TraceTime=trace_time
	def get_trace_time(self)->TraceTime:
		return self._trace_time
	def __str__(self):
		return f'{str(self._trace_time)}[traffic]==green'
class AgentState(Variable,Node):
	def __init__(self,trace_time:TraceTime,name:AnyStr=None):
		Variable.__init__(self, name)
		Node.__init__(self, NodeType.T_AGENTSTATE)
		self._agent=None
		self._trace_time:TraceTime=trace_time
	def set_agent(self,agent:Union[Pedestrian,Obstacle,NPCVehicle]):
		self._agent=agent
	def get_agent(self)->Union[Pedestrian,Obstacle,NPCVehicle]:
		return self._agent
	def get_trace_time(self)->TraceTime:
		return self._trace_time
	def __str__(self) -> str:
		return f'{str(self._trace_time)}[perception][{self._agent.get_name()}]'
class AgentGroundTruth(Variable,Node):
	def __init__(self,trace_time:TraceTime,name:AnyStr=None):
		Variable.__init__(self, name)
		Node.__init__(self, NodeType.T_AGENTGROUNDTRUTH)
		self._agent=None
		self._trace_time: TraceTime = trace_time
	def set_agent(self,agent:Union[Pedestrian,Obstacle,NPCVehicle]):
		self._agent=agent
	def get_agent(self)->Union[Pedestrian,Obstacle,NPCVehicle]:
		return self._agent
	def get_trace_time(self)->TraceTime:
		return self._trace_time
	def __str__(self) -> str:
		return f'{str(self._trace_time)}[truth][{self._agent.get_name()}]'


# Holds EgoState and AgentGroundTruth, and they must have the same trace time.
class AgentGroundDistance(Variable,Node):
	def __init__(self,name:AnyStr=None):
		Variable.__init__(self,name)
		Node.__init__(self,NodeType.T_AGENTGROUNDDIS)
		self._ego_state=None
		self._agent_ground_truth=None
	def set_ego_state(self,ego_state:EgoState):
		self._ego_state=ego_state
	def set_agent_ground_truth(self,agent_ground_truth:AgentGroundTruth):
		self._agent_ground_truth=agent_ground_truth
	def get_ego_state(self)->EgoState:
		return self._ego_state
	def get_agent_ground_truth(self)->AgentGroundTruth:
		return self._agent_ground_truth
	def __str__(self) -> str:
		return f'dis({str(self._ego_state)},{str(self._agent_ground_truth)})'

# Holds AgentState and AgentGroundTruth, and they must have the same trace time.

class AgentError(Variable,Node):
	def __init__(self,name:AnyStr=None):
		Variable.__init__(self, name)
		Node.__init__(self, NodeType.T_AGENTERROR)
		self._agent_state = None
		self._agent_ground_truth = None

	def set_agent_state(self, agent_state: AgentState):
		self._agent_state = agent_state

	def set_agent_ground_truth(self, agent_ground_truth: AgentGroundTruth):
		self._agent_ground_truth = agent_ground_truth

	def get_agent_state(self)->AgentState:
		return self._agent_state

	def get_agent_ground_truth(self)->AgentGroundTruth:
		return self._agent_ground_truth
	def __str__(self) -> str:
		return f'diff({str(self._agent_state)},{str(self._agent_ground_truth)})'

# Holds AgentState and EgoState, and they must have the same trace time.

class AgentSafetyAssertion:
	def __init__(self, safety_radius: float):
		self._safety_radius = safety_radius
		self._agent_state = None
		self._ego_state=None
	def set_agent_state(self, agent_state: AgentState):
		self._agent_state = agent_state
	def set_ego_state(self,ego_state:EgoState):
		self._ego_state=ego_state
	def get_agent_state(self)->AgentState:
		return self._agent_state
	def get_ego_state(self)->EgoState:
		return self._ego_state
	def get_safety_radius(self) -> float:
		return self._safety_radius
	def __str__(self):
		return f'dis({str(self._ego_state),str(self._agent_state)})>={self._safety_radius}'


# trace[1][traffic]==trace[1][ground][traffic]
# The two trace time must have the same trace time.
class TrafficDetectionAssertion:
	def __init__(self):
		super().__init__()
		self._left_trace_time=None
		self._right_trace_time=None
	def __str__(self):
		pass
	def set_left_trace_time(self,trace_time:TraceTime):
		self._left_trace_time=trace_time
	def get_left_trace_time(self)->TraceTime:
		return self._left_trace_time
	def set_right_trace_time(self,trace_time:TraceTime):
		self._right_trace_time=trace_time
	def get_right_trace_time(self)->TraceTime:
		return self._right_trace_time
	def __str__(self) -> str:
		return f'{str(self._left_trace_time)}[perception][traffic]=={str(self._right_trace_time)}[truth][traffic]'


# All trace times must have the same trace time.
class IntersectionAssertion(Node,Variable):
	def __init__(self,name:AnyStr=None):
		Node.__init__(self,NodeType.T_INTERASSERT)
		Variable.__init__(self,name)
		self._left_traffic_detection=None
		self._right_traffic_detection=None
		self._red_light=None
		self._green_light=None
		self._ego_speed=None
	def set_ego_speed(self,ego_speed:EgoSpeed):
		self._ego_speed=ego_speed
	def get_ego_speed(self)->EgoSpeed:
		return self._ego_speed

	def set_left_traffic_detection(self,traffic_detection:TrafficDetectionAssertion):
		self._left_traffic_detection=traffic_detection

	def get_left_traffic_detection(self)->TrafficDetectionAssertion:
		return self._left_traffic_detection
	def set_right_traffic_detection(self,traffic_detection:TrafficDetectionAssertion):
		self._right_traffic_detection=traffic_detection

	def get_right_traffic_detection(self)->TrafficDetectionAssertion:
		return self._right_traffic_detection
	def set_red_light_state(self,red_light:RedLightState):
		self._red_light=red_light
	def get_red_light_state(self)->RedLightState:
		return self._red_light
	def set_green_light_state(self,green_light:GreenLightState):
		self._green_light=green_light
	def get_green_light_state(self)->GreenLightState:
		return self._green_light
	def __str__(self) -> str:
		return f'({str(self._left_traffic_detection)}&{str(self._red_light)})->(~{str(self._ego_speed)}U({str(self._right_traffic_detection)}&{str(self._green_light)}))'
class SpeedLimitationChecking:
	def __init__(self,trace_time:TraceTime):
		self._trace_time:TraceTime=trace_time
		self._speed_range=None
	def get_trace_time(self)->TraceTime:
		return self._trace_time
	def set_speed_range(self,speed_range:SpeedRange):
		self._speed_range=speed_range
	def get_speed_range(self)->SpeedRange:
		return self._speed_range
	def __str__(self) -> str:
		return f'{str(self._trace_time)}[traffic]=={self._speed_range.get_value()}'
class SpeedViolation:
	def __init__(self,trace_time:TraceTime,index0:bool):
		self._trace_time:TraceTime=trace_time
		self._speed=None
		self._index0=index0
	def get_trace_time(self)->TraceTime:
		return self._trace_time
	def set_speed(self,speed:Speed):
		self._speed=speed
	def get_speed(self)->Speed:
		return self._speed
	def isMinimumSpeedViolation(self)->bool:
		return self._index0
	def isMaximumSpeedViolation(self)->bool:
		return not self._index0
	def __str__(self) -> str:
		return f'{self._speed.get_speed_value()}<{str(self._trace_time)}[traffic][{0 if self._index0 else 1}]'

# All trace times must have the same trace time.
class SpeedConstraintAssertion(Node,Variable):
	def __init__(self,name:AnyStr=None):
		Node.__init__(self,NodeType.T_SPEEDCA)
		Variable.__init__(self,name)
		self._traffic_detection=None
		self._speed_limitation_checking=None
		self._left_speed_violation=None
		self._right_speed_violation=None
		self._time_duration:float=0
	def set_left_speed_violation(self,speed_violation:SpeedViolation):
		self._left_speed_violation=speed_violation
	def get_left_speed_violation(self)->SpeedViolation:
		return self._left_speed_violation
	def set_right_speed_violation(self,speed_violation:SpeedViolation):
		self._right_speed_violation=speed_violation
	def set_traffic_detection(self,traffic_detection:TrafficDetectionAssertion):
		self._traffic_detection=traffic_detection
	def get_traffic_detection(self)->TrafficDetectionAssertion:
		return self._traffic_detection
	def get_right_speed_violation(self)->SpeedViolation:
		return self._right_speed_violation
	def set_speed_limitation_checking(self,speed_limitation_checking:SpeedLimitationChecking):
		self._speed_limitation_checking=speed_limitation_checking
	def get_speed_limitation_checking(self)->SpeedLimitationChecking:
		return self._speed_limitation_checking
	def set_time_duration(self,duration:float):
		self._time_duration=duration
	def get_time_duration(self)->float:
		return self._time_duration
	def __str__(self) -> str:
		return f'({str(self._traffic_detection)}&{str(self._speed_limitation_checking)}&{str(self._left_speed_violation)})' \
			'->F[0,{self._time_duration}]~{str(self._right_speed_violation)}'
class AgentVisibleDetectionAssertion:
	def __init__(self, sensing_range: float):
		super().__init__()
		self._sensing_range=sensing_range
		self._agent_ground_distance=None
	def set_agent_ground_distance(self,agent_ground_distance:AgentGroundDistance):
		self._agent_ground_distance=agent_ground_distance
	def get_agent_ground_distance(self)->AgentGroundDistance:
		return self._agent_ground_distance
	def get_sensing_range(self)->float:
		return self._sensing_range
	def __str__(self) -> str:
		return f'{str(self._agent_ground_distance)}<={self._sensing_range}'
class AgentErrorDetectionAssertion:
	def __init__(self, error_threshold: float):
		super().__init__()
		self._error_threshold = error_threshold
		self._agent_error = None

	def set_agent_error(self, agent_error: AgentError):
		self._agent_error = agent_error

	def get_agent_error(self)->AgentError:
		return self._agent_error

	def get_error_threshold(self) -> float:
		return self._error_threshold
	def __str__(self) -> str:
		return f'{str(self._agent_error)}<={self._error_threshold}'
class DetectionAssertion(Node,Variable):
	def __init__(self,name:Optional[AnyStr]=None):
		Variable.__init__(self,name)
		Node.__init__(self,NodeType.T_DETECTIONS)
		self._assertions:List[Union[AgentVisibleDetectionAssertion,AgentErrorDetectionAssertion,TrafficDetectionAssertion]]=[]
	def add_assertion(self,detection:Union[AgentVisibleDetectionAssertion,AgentErrorDetectionAssertion,TrafficDetectionAssertion]):
		self._assertions.append(detection)
	def get_size(self):
		return len(self._assertions)
	def get_assertions(self)->List[Union[AgentVisibleDetectionAssertion,AgentErrorDetectionAssertion,TrafficDetectionAssertion]]:
		return self._assertions
	def __str__(self) -> str:
		pass
class SafetyAssertion(Variable,Node):
	def __init__(self,name:Optional[AnyStr]=None):
		Variable.__init__(self,name)
		Node.__init__(self,NodeType.T_SAFETYS)
		self._assertions:List[Union[AgentVisibleDetectionAssertion,AgentErrorDetectionAssertion,AgentSafetyAssertion]]=[]
	def add_assertion(self,assertion:Union[AgentVisibleDetectionAssertion,AgentErrorDetectionAssertion,AgentSafetyAssertion]):
		self._assertions.append(assertion)
	def get_size(self)->int:
		return len(self._assertions)
	def get_assertions(self)->List[Union[AgentVisibleDetectionAssertion,AgentErrorDetectionAssertion,AgentSafetyAssertion]]:
		return self._assertions
	def __str__(self) -> str:
		pass
class Trace(Node,Variable):
	def __init__(self,name:AnyStr,scenario:Scenario):
		Node.__init__(self,NodeType.T_TRACE)
		Variable.__init__(self,name)
		self._scenario=scenario
		self._detection_assertions:List[DetectionAssertion]=[]
		self._safety_assertions:List[SafetyAssertion]=[]
		self._intersection_assertions:List[IntersectionAssertion]=[]
		self._speed_constraint_assertions:List[SpeedConstraintAssertion]=[]
	def add_detection_assertion(self,detection_assertion:DetectionAssertion):
		self._detection_assertions.append(detection_assertion)
	def add_safety_assertion(self,safety_assertion:SafetyAssertion):
		self._safety_assertions.append(safety_assertion)
	def add_intersection_assertion(self,intersection_assertion:IntersectionAssertion):
		self._intersection_assertions.append(intersection_assertion)
	def add_speed_constraint_assertion(self,speed_constraint_assertion:SpeedConstraintAssertion):
		self._speed_constraint_assertions.append(speed_constraint_assertion)
	def get_detection_assertions(self)->List[DetectionAssertion]:
		return self._detection_assertions
	def get_safety_assertions(self)->List[SafetyAssertion]:
		return self._safety_assertions
	def get_intersection_assertions(self)->List[IntersectionAssertion]:
		return self._intersection_assertions
	def get_speed_constraint_assertions(self)->List[SpeedConstraintAssertion]:
		return self._speed_constraint_assertions
	def get_scenario(self)->Scenario:
		return self._scenario
	def __str__(self) -> str:
		pass
class AssignAssertionToTrace(Node,Variable):
	def __init__(self):
		# Empty Name
		Variable.__init__(self,'')
		Node.__init__(self,NodeType.T_AASSERTIONTRACE)
		self._trace=None
		self._assertion=None
	def set_trace(self,t:Trace):
		self._trace=t
	def set_assertion(self,assertion:Union[DetectionAssertion,SafetyAssertion,IntersectionAssertion,SpeedConstraintAssertion]):
		self._assertion=assertion
	def get_trace(self)->Trace:
		return self._trace
	def get_assertion(self)->Union[DetectionAssertion,SafetyAssertion,IntersectionAssertion,SpeedConstraintAssertion]:
		return self._assertion
	def __str__(self) -> str:
		pass