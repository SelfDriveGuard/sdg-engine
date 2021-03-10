# this is the main entry.
import re
from src.scenest_parser.ast.assertion.assertion import DetectionAssertion, SafetyAssertion

from antlr4.CommonTokenStream import CommonTokenStream
from antlr4.tree.Tree import ParseTreeWalker
from antlr4.FileStream import FileStream
from src.scenest_parser.ast.base.motion import Motion, StateList, UniformIndex, WaypointIndex
from src.scenest_parser.ast.base.pedestrian_type import Height
from src.scenest_parser.ast.base.vehicle_type import (Color, ColorList, ColorListEnum, GeneralType,
								   GeneralTypeEnum, RGBColor, SpecificType,
								   Type)
from src.scenest_parser.ast.base.weathers import (Weather, WeatherContinuousIndex,
							   WeatherDiscreteLevel, WeatherDiscreteLevelEnum,
							   WeatherKind)
from src.scenest_parser.ast.error.error import IllegalTypeException
from src.scenest_parser.ast.scenario.scenario import *
from src.scenest_parser.ast.unresolved.unresolved import *
from src.scenest_parser.ast.assertion.assertion import *
from src.scenest_parser.ast.ast import *
from src.scenest_parser import AVScenariosListener
from src.scenest_parser import AVScenariosLexer
from src.scenest_parser import AVScenariosParser


class Sema:
	class Current:
		# XXX: _states stores the State objects we created.
		# Due to the fact that we cannot use one State object
		# to figure out which object it belongs to.
		# e.g., EgoVehicle has two States,
		# NPCVehicle has two States and one VehicleMotion,while
		# VehicleMotion has one State or a StateList consisting 
		# of States elements.
		class TemporaryStates:
			def __init__(self):
				# _first and _second denote the Pedestrian and NPCVehicle
				# initial and target State.
				# _value denotes other State.
				self._first: Optional[State] = None
				self._second: Optional[State] = None
				self._value: Optional[State] = None
				self._flag = 0

		'''
		This class represents the curret parsing point
		'''

		def __init__(self):
			self._scenario = None
			self._map = None
			self._ego_vehicle = None
			self._pedestrians = None
			self._npc_vehicles = None
			self._obstacles = None
			self._traffic = None
			self._vehicle_type = None
			# TODO:reduce coordinate and lanecoordinate to one variable
			self._coordinate = None
			self._lane = None
			self._lane_coordinate = None
			self._direction = None
			self._pedestrian_motion = None
			self._pedestrian_type = None
			self._name_with_motion = None
			self._position = None
			self._heading = None
			self._type = None
			self._height = None
			self._color = None
			self._npc_vehicle = None
			self._state_list = None
			self._pedestrian = None
			self._obstacle = None
			self._shape = None
			self._env = None
			self._time = None
			self._weathers = None
			self._weather = None
			self._weather_continuous = None
			self._weather_discrete = None
			self._intersection_traffic = None
			self._intersection_id = None
			self._speed_limit = None
			self._speed_range = None
			self._speed = None
			self._states = self.TemporaryStates()
			self._ego_speed = None
			self._agent_state = None
			self._agent_ground_truth = None
			self._agent_ground_distance = None
			self._agent_visible_assert=None
			self._agent_error_assert=None
			self._agent_error = None
			self._agent_safety_assertion = None
			self._intersection_assertion = None
			self._speed_limitation_checking = None
			self._speed_violation=None
			self._speed_constraint_assertion = None
			self._trace_time = None
			self._ego_state = None
			self._traffic_detection_assert=None
			self._ego_speed=None
			self._red_light=None
			self._green_light=None
			self._detection_assertion=None
			self._safety_assertion=None
	def __init__(self):
		self._ast = AST()
		self._current = self.Current()

	def get_ast(self) -> AST:
		return self._ast

	def check_unique_id(self, name: AnyStr) -> None:
		self._ast.check_unique_id(name)

	# The following functions are helper functions that dealing with
	# different AST nodes during parsing.

	def begin_scenario(self, s: Scenario):
		pass

	def end_scenario(self, s: Scenario):
		# NOTICE:Scenarios can have a default(empty)
		# NPCVehicles,Obstacles,Environment and Traffic
		assert self._current._map is not None
		s.add_map(self._current._map)
		assert self._current._ego_vehicle is not None
		s.add_ego_vehicle(self._current._ego_vehicle)
		if self._current._npc_vehicles is not None:
			s.add_npc_vehicles(self._current._npc_vehicles)
		if self._current._pedestrians is not None:
			s.add_pedestrians(self._current._pedestrians)
		if self._current._obstacles is not None:
			s.add_obstacles(self._current._obstacles)
		if self._current._env is not None:
			s.add_environment(self._current._env)
		if self._current._traffic is not None:
			s.add_traffic(self._current._traffic)

	# we have already construct a scenario.
	def finish_scenario(self, s: Scenario):
		self._ast.add_scenario(s)
		self._ast.add_ast_node(s)

	def act_on_name_with_real_value(self, name: AnyStr, rv: float):
		self.check_unique_id(name)
		v = NameWithRealValue(rv, name)
		# self._ast.addNRV(v)
		self._ast.add_ast_node(v)

	def act_on_name_with_string(self, name: AnyStr, str_: AnyStr):
		self.check_unique_id(name)
		v = NameWithString(str_, name)
		# self._ast.addNS(v)
		self._ast.add_ast_node(v)

	def act_on_scenario(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._scenario is not None
		self._current._scenario.set_name(name)
		self.finish_scenario(self._current._scenario)

	def act_on_ego_vehicle(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._ego_vehicle is not None
		self._current._ego_vehicle.set_name(name)
		self.finish_ego_vehicle(self._current._ego_vehicle)

	def finish_ego_vehicle(self, e: EgoVehicle):
		self._ast.add_ast_node(e)

	# we must find the real context about this rule may stands for.
	# var may be <position>
	# or <type_>
	# or <state>
	# line and column is helpful to handle exception.
	def act_on_name_with_one_variable(self, name: AnyStr, var: AnyStr, info: Tuple[int, int]):
		self.check_unique_id(name)
		v, index = self._ast.find_node(var)
		if isinstance(v, Position):
			s = State(name)
			# state=(position) no heading no speed
			s.set_position(v)
			self._ast.add_ast_node(s)
		elif isinstance(v, NameWithTwoRealValues):
			s = State(name)
			p = self.cast_to_position(v)
			s.set_position(p)
			self._ast.add_ast_node(s)
			self._ast.set(index, p)
		elif isinstance(v, Type):
			vt = VehicleType(name)
			vt.set_type(v)
			self._ast.add_ast_node(vt)
		elif isinstance(v, NameWithString):
			vt = VehicleType(name)
			t = self.cast_to_type(v)
			vt.set_type(t)
			self._ast.add_ast_node(vt)
			self._ast.set(index, t)
		elif isinstance(v, State):
			sl = StateList(name)
			sl.add_state(v)
			self._ast.add_ast_node(sl)
		else:
			raise IllegalTypeException(v.get_name(), info[0], info[1], v.__class__.__name__, 'State', 'Position' \
									   , 'Type')
	def act_on_state(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._states._value is not None
		self._current._states._value.set_name(name)
		self.finish_state(self._current._states._value)
	def begin_state(self, name:AnyStr):
		pass

	def end_state(self, s: State):
		assert self._current._position is not None
		s.set_position(self._current._position)
		if self._current._heading is not None:
			s.set_heading(self._current._heading)
		if self._current._speed is not None:
			s.set_speed(self._current._speed)

	def finish_state(self, s: State):
		self._ast.add_ast_node(s)
	def act_on_vehicle_type(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._vehicle_type is not None
		self._current._vehicle_type.set_name(name)
		self.finish_vehicle_type(self._current._vehicle_type)
	def begin_vehicle_type(self, vt: VehicleType):
		pass

	def end_vehicle_type(self, vt: VehicleType):
		assert self._current._type is not None
		vt.set_type(self._current._type)
		if self._current._color is not None:
			vt.set_color(self._current._color)

	# TODO: material?
	def finish_vehicle_type(self, vt: VehicleType):
		self._ast.add_ast_node(vt)

	# we must find the real context about this rule may stands for.
	# v1 and v2 may be <position> <heading>
	# or <type_> <color>
	# or <height> <color>
	# or <state> <state>
	def act_on_name_with_two_variables(self, name: AnyStr, v1: AnyStr, v2: AnyStr, v1_info: Tuple[int, int]
									   , v2_info: Tuple[int, int]):
		self.check_unique_id(name)
		n1, index1 = self._ast.find_node(v1)
		n2, _ = self._ast.find_node(v2)
		if isinstance(n1, Position):
			if isinstance(n2, Heading):
				s = State(name)
				s.set_position(n1)
				s.set_heading(n2)
				self._ast.add_ast_node(s)
			else:
				raise IllegalTypeException(n2.get_name(), v2_info[0], v2_info[1], n2.__class__.__name__, 'Heading')
		elif isinstance(n1, NameWithTwoRealValues):
			if isinstance(n2, Heading):
				p = self.cast_to_position(n1)
				s = State(name)
				s.set_position(p)
				s.set_heading(n2)
				self._ast.add_ast_node(s)
				self._ast.set(index1, p)
			else:
				raise IllegalTypeException(n2.get_name(), v2_info[0], v2_info[1], n2.__class__.__name__, 'Heading')
		elif isinstance(n1, Type):
			if isinstance(n2, Color):
				vt = VehicleType(name)
				vt.set_type(n1)
				vt.set_color(n2)
				self._ast.add_ast_node(vt)
			else:
				raise IllegalTypeException(n2.get_name(), v2_info[0], v2_info[1], n2.__class__.__name__, 'Color')
		elif isinstance(n1, NameWithString):
			if isinstance(n2, Color):
				vt = VehicleType(name)
				t = self.cast_to_type(n1)
				vt.set_type(t)
				vt.set_color(n2)
				self._ast.add_ast_node(vt)
				self._ast.set(index1, t)
			else:
				raise IllegalTypeException(n2.get_name(), v2_info[0], v2_info[1], n2.__class__.__name__, 'Color')
		elif isinstance(n1, Height):
			if isinstance(n2, Color):
				pt = PedestrianType(name)
				pt.set_height(n1)
				pt.set_color(n2)
				self._ast.add_ast_node(pt)
			else:
				raise IllegalTypeException(n2.get_name(), v2_info[0], v2_info[1], n2.__class__.__name__, 'Color')
		elif isinstance(n1, NameWithRealValue):
			if isinstance(n2, Color):
				pt = PedestrianType(name)
				h = self.cast_to_height(n1)
				pt.set_height(h)
				pt.set_color(n2)
				self._ast.add_ast_node(pt)
				self._ast.set(index1, h)
			else:
				raise IllegalTypeException(n2.get_name(), v2_info[0], v2_info[1], n2.__class__.__name__, 'Color')
		elif isinstance(n1, State):
			if isinstance(n2, State):
				sl = StateList(name)
				sl.add_state(n1)
				sl.add_state(n2)
				self._ast.add_ast_node(sl)
			else:
				raise IllegalTypeException(n2.get_name(), v2_info[0], v2_info[1], n2.__class__.__name__, 'State')
		else:
			raise IllegalTypeException(n1.get_name(), v1_info[0], v1_info[1], n1.__class__.__name__,
									   'Position', 'Type', 'Height', 'State')
	def act_on_pedestrian_type(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._pedestrian_type is not None
		self._current._pedestrian_type.set_name(name)
		self.finish_pedestrian_type(self._current._pedestrian_type)
	def begin_pedestrian_type(self, pt: PedestrianType):
		pass

	def end_pedestrian_type(self, pt: PedestrianType):
		assert self._current._height is not None
		pt.set_height(self._current._height)
		assert self._current._color is not None
		pt.set_color(self._current._color)

	def finish_pedestrian_type(self, pt: PedestrianType):
		self._ast.add_ast_node(pt)

	def begin_name_with_one_variable_and_heading(self, name: AnyStr):
		self.check_unique_id(name)

	def end_name_with_one_variable_and_heading(self, name: AnyStr, v: AnyStr, info: Tuple[int, int]):
		# before this we have called begin_name_with_one_variable_and_heading
		s = State(name)
		n, index = self._ast.find_node(v)
		assert self._current._heading is not None
		s.set_heading(self._current._heading)
		if isinstance(n, Position):
			s.set_position(n)
		elif isinstance(n, NameWithTwoRealValues):
			p = self.cast_to_position(n)
			s.set_position(p)
			self._ast.set(index, p)
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__, 'Position')
		self.finish_state(s)

	def begin_name_with_one_variable_and_color(self, name: AnyStr):
		self.check_unique_id(name)

	def end_name_with_one_variable_and_color(self, name: AnyStr, v: AnyStr, info: Tuple[int, int]):
		# before this we have called begin_name_with_one_variable_and_color
		n, index = self._ast.find_node(v)
		if isinstance(n, Type):
			vt = VehicleType(name)
			vt.set_type(n)
			if self._current._color is not None:
				vt.set_color(self._current._color)
			self.finish_vehicle_type(vt)
		elif isinstance(n, NameWithString):
			vt = VehicleType(name)
			t = self.cast_to_type(n)
			vt.set_type(t)
			if self._current._color is not None:
				vt.set_color(self._current._color)
			self._ast.set(index, t)
			self.finish_vehicle_type(vt)
		elif isinstance(n, Height):
			assert self._current._color is not None
			pt = PedestrianType(name)
			pt.set_height(n)
			pt.set_color(self._current._color)
			self.finish_pedestrian_type(pt)
		elif isinstance(n, NameWithRealValue):
			assert self._current._color is not None
			pt = PedestrianType(name)
			h = self.cast_to_height(n)
			pt.set_height(h)
			self._ast.set(index, h)
			pt.set_color(self._current._color)
			self.finish_pedestrian_type(pt)
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Type', 'Height')

	def begin_name_with_position_and_one_variable(self, name: AnyStr):
		self.check_unique_id(name)

	def end_name_with_position_and_one_variable(self, name: AnyStr, v: AnyStr, info: Tuple[int, int]):
		# before this we have called begin_name_with_position_and_one_variable
		s = State(name)
		n, _ = self._ast.find_node(v)
		assert self._current._position is not None
		s.set_position(self._current._position)
		if isinstance(n, Heading):
			s.set_heading(n)
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Heading')
		self.finish_state(s)

	def begin_name_with_type_and_one_variable(self, name: AnyStr):
		self.check_unique_id(name)

	def end_name_with_type_and_one_variable(self, name: AnyStr, v: AnyStr, info: Tuple[int, int]):
		vt = VehicleType(name)
		n, _ = self._ast.find_node(v)
		if isinstance(n, Color):
			assert self._current._type is not None
			vt.set_type(self._current._type)
			vt.set_color(n)
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Color')
		self.finish_vehicle_type(vt)

	def begin_name_with_height_and_one_variable(self, name: AnyStr):
		self.check_unique_id(name)

	def end_name_with_height_and_one_variable(self, name: AnyStr, v: AnyStr, info: Tuple[int, int]):
		pt = PedestrianType(name)
		n, _ = self._ast.find_node(v)
		if isinstance(n, Color):
			assert self._current._height is not None
			pt.set_height(self._current._height)
			pt.set_color(n)
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Color')
		self.finish_pedestrian_type(pt)

	def finish_position(self, p: Position):
		self._ast.add_ast_node(p)

	def act_on_coordinate_position(self, name: AnyStr, coor: AnyStr, x: float, y: float,z:Optional[float]=None):
		self.check_unique_id(name)
		cf = Coordinate(x, y, z)
		p = Position(name)
		p.set_coordinate(cf)
		if coor == 'IMU':
			p.set_frame(CoordinateFrame.CF_IMU)
		elif coor == 'ENU':
			p.set_frame(CoordinateFrame.CF_ENU)
		elif coor == 'WGS84':
			p.set_frame(CoordinateFrame.CF_WGS84)
		self.finish_position(p)
	def act_on_lane_coordinate_position(self, name: AnyStr, coor: AnyStr, rv: float):
		self.check_unique_id(name)
		assert self._current._lane is not None
		lc = LaneCoordinate(rv)
		lc.set_lane(self._current._lane)
		p = Position(name)
		p.set_coordinate(lc)
		if coor == 'IMU':
			p.set_frame(CoordinateFrame.CF_IMU)
		elif coor == 'ENU':
			p.set_frame(CoordinateFrame.CF_ENU)
		elif coor == 'WGS84':
			p.set_frame(CoordinateFrame.CF_WGS84)	
		self.finish_position(p)
	def act_on_name_with_two_real_values(self, name: AnyStr, v1: float, v2: float):
		self.check_unique_id(name)
		v = NameWithTwoRealValues(v1, v2, name)
		self._ast.add_ast_node(v)

	# we must find the real context about this rule may stands for.
	# v1 v2, and v3 may be <position> <heading> <speed>
	# or <state> <state> <state>
	def act_on_name_with_three_variables(self, name: AnyStr, v1: AnyStr, v2: AnyStr, v3: AnyStr
										 , n1Info: Tuple[int, int], n2Info: Tuple[int, int], n3Info: Tuple[int, int]):
		self.check_unique_id(name)
		n1, index1 = self._ast.find_node(v1)
		n2, _ = self._ast.find_node(v2)
		n3, index3 = self._ast.find_node(v3)
		isPos = isinstance(n1, Position)
		isNTRV = isinstance(n1, NameWithTwoRealValues)
		isState = isinstance(n1, State)
		if not isPos and not isNTRV and not isState:
			raise IllegalTypeException(n1.get_name(), n1Info[0], n1Info[1], n1.__class__.__name__,
									   'Position', 'State')
		if not isState:
			if isNTRV:
				n1 = self.cast_to_position(n1)
				self._ast.set(index1, n1)
			isHead = isinstance(n2, Heading)
			if isHead:
				isSpeed = isinstance(n3, Speed)
				isNRV = isinstance(n3, NameWithRealValue)
				if not isSpeed and not isNRV:
					raise IllegalTypeException(n3.get_name(), n3Info[0], n3Info[1], n3.__class__.__name__,
											   'Speed')
				if isNRV:
					n3 = self.cast_to_speed(n3)
					self._ast.set(index3, n3)
				s = State(name)
				s.set_position(n1)
				s.set_heading(n2)
				s.set_speed(n3)
				self._ast.add_ast_node(s)
			else:
				raise IllegalTypeException(n2.get_name(), n2Info[0], n2Info[1], n2.__class__.__name__,
										   'Heading')
		else:
			if isinstance(n2, State):
				if isinstance(n3, State):
					sl = StateList(name)
					sl.add_state(n1)
					sl.add_state(n2)
					sl.add_state(n3)
					self._ast.add_ast_node(sl)
				else:
					raise IllegalTypeException(n3.get_name(), n3Info[0], n3Info[1], n3.__class__.__name__,
											   'State')
			else:
				raise IllegalTypeException(n2.get_name(), n2Info[0], n2Info[1], n2.__class__.__name__,
										   'State')

	def finish_heading(self, h: Heading):
		self._ast.add_ast_node(h)

	def act_on_heading(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._heading is not None
		self._current._heading.set_name(name)
		self.finish_heading(self._current._heading)

	def finish_type(self, t: Type):
		self._ast.add_ast_node(t)

	def act_on_type(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._type is not None
		self._current._type.set_name(name)
		self.finish_type(self._current._type)

	def finish_color(self, c: Color):
		self._ast.add_ast_node(c)

	def act_on_color(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._color is not None
		self._current._color.set_name(name)
		self.finish_color(self._current._color)

	def finish_npc_vehicle(self, n: NPCVehicle):
		self._ast.add_ast_node(n)

	def act_on_npc_vehicle(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._npc_vehicle is not None
		self._current._npc_vehicle.set_name(name)
		self.finish_npc_vehicle(self._current._npc_vehicle)

	def finish_state_list(self, sl: StateList):
		self._ast.add_ast_node(sl)

	def act_on_stateList(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._state_list is not None
		self._current._state_list.set_name(name)
		self.finish_state_list(self._current._state_list)

	def finish_pedestrians(self, ps: Pedestrians):
		self._ast.add_ast_node(ps)

	def act_on_pedestrians(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._pedestrians is not None
		self._current._pedestrians.set_name(name)
		self.finish_pedestrians(self._current._pedestrians)

	def finish_npc_vehicles(self, npcs: NPCVehicles):
		self._ast.add_ast_node(npcs)

	def act_on_npc_vehicles(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._npc_vehicles is not None
		self._current._npc_vehicles.set_name(name)
		self.finish_npc_vehicles(self._current._npc_vehicles)

	def finish_obstacles(self, obs: Obstacles):
		self._ast.add_ast_node(obs)

	def act_on_obstacles(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._obstacles is not None
		self._current._obstacles.set_name(name)
		self.finish_obstacles(self._current._obstacles)

	def finish_weathers(self, ws: Weathers):
		self._ast.add_ast_node(ws)

	def act_on_weathers(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._weathers is not None
		self._current._weathers.set_name(name)
		self.finish_weathers(self._current._weathers)

	def finish_traffic(self, t: Traffic):
		self._ast.add_ast_node(t)

	def act_on_traffic(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._traffic is not None
		self._current._traffic.set_name(name)
		self.finish_traffic(self._current._traffic)

	def finish_pedestrian(self, p: Pedestrian):
		self._ast.add_ast_node(p)

	def act_on_pedestrian(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._pedestrian is not None
		self._current._pedestrian.set_name(name)
		self.finish_pedestrian(self._current._pedestrian)

	def finish_obstacle(self, o: Obstacle):
		self._ast.add_ast_node(o)

	def act_on_obstacle(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._obstacle is not None
		self._current._obstacle.set_name(name)
		self.finish_obstacle(self._current._obstacle)

	def finish_environment(self, e: Environment):
		self._ast.add_ast_node(e)

	def act_on_environment(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._env is not None
		self._current._env.set_name(name)
		self.finish_environment(self._current._env)

	def finish_shape(self, s: Shape):
		self._ast.add_ast_node(s)

	def act_on_shape(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._shape is not None
		self._current._shape.set_name(name)
		self.finish_shape(self._current._shape)

	def finish_time(self, t: Time):
		self._ast.add_ast_node(t)

	def act_on_time(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._time is not None
		self._current._time.set_name(name)
		self.finish_time(self._current._time)

	def finish_weather(self, w: Weather):
		self._ast.add_ast_node(w)

	def act_on_weather(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._weather is not None
		self._current._weather.set_name(name)
		self.finish_weather(self._current._weather)

	def act_on_weather_discrete_level(self, name: AnyStr, value: AnyStr):
		if value == 'light':
			self._ast.add_ast_node(WeatherDiscreteLevel(
				WeatherDiscreteLevelEnum.WDL_LIGHT, name))
		elif value == 'middle':
			self._ast.add_ast_node(WeatherDiscreteLevel(
				WeatherDiscreteLevelEnum.WDL_MIDDLE, name))
		elif value == 'heavy':
			self._ast.add_ast_node(WeatherDiscreteLevel(
				WeatherDiscreteLevelEnum.WDL_HEAVY, name))

	def finish_intersection_traffic(self, it: IntersectionTraffic):
		self._ast.add_ast_node(it)

	def act_on_intersecion_traffic(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._intersection_traffic is not None
		self._current._intersection_traffic.set_name(name)
		self.finish_intersection_traffic(self._current._intersection_traffic)

	def finish_speed_limitation(self, sl: SpeedLimitation):
		self._ast.add_ast_node(sl)

	def act_on_speed_limitation(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._speed_limit is not None
		self._current._speed_limit.set_name(name)
		self.finish_speed_limitation(self._current._speed_limit)

	def finish_name_with_motion(self, nm: NameWithMotion):
		self._ast.add_ast_node(nm)

	def act_on_name_with_motion(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._name_with_motion is not None
		self._current._name_with_motion.set_name(name)
		self.finish_name_with_motion(self._current._name_with_motion)

	def find_map(self, name: AnyStr, info: Tuple[int, int]) -> Map:
		n, index = self._ast.find_node(name)
		if isinstance(n, Map):
			return n
		elif isinstance(n, NameWithString):
			m = self.cast_to_map(n)
			self._ast.set(index, m)
			return m
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Map')

	def find_ego_vehicle(self, name: AnyStr, info: Tuple[int, int]) -> EgoVehicle:
		n, _ = self._ast.find_node(name)
		if isinstance(n, EgoVehicle):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'EgoVehicle')

	def begin_ego_vehicle(self, e: EgoVehicle):
		# do not check unique because
		# e is anonymous
		pass

	def end_ego_vehicle(self, e: EgoVehicle):
		assert self._current._states._first is not None \
			   and self._current._states._second is not None
		e.set_first_state(self._current._states._first)
		e.set_second_state(self._current._states._second)
		if self._current._vehicle_type is not None:
			e.set_vehicle_type(self._current._vehicle_type)

	# do not call add_ast_node
	def find_state(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, State):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'State')

	def begin_position(self, p: Position):
		# do not call check_unique_id
		pass

	def end_position(self, p: Position, coor: AnyStr = ''):
		if coor == 'IMU':
			p.set_frame(CoordinateFrame.CF_IMU)
		elif coor == 'ENU':
			p.set_frame(CoordinateFrame.CF_ENU)
		elif coor == 'WGS84':
			p.set_frame(CoordinateFrame.CF_WGS84)
		assert not (self._current._lane_coordinate is None and
					self._current._coordinate is None)
		if self._current._lane_coordinate is not None:
			p.set_coordinate(self._current._lane_coordinate)
		elif self._current._coordinate is not None:
			p.set_coordinate(self._current._coordinate)

	def find_position(self, name: AnyStr, info: Tuple[int, int]):
		n, index = self._ast.find_node(name)
		if isinstance(n, Position):
			return n
		elif isinstance(n, NameWithTwoRealValues):
			p = self.cast_to_position(n)
			self._ast.set(index, p)
			return p
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Position')

	def find_speed(self, name: AnyStr, info: Tuple[int, int]):
		n, index = self._ast.find_node(name)
		if isinstance(n, Speed):
			return n
		if isinstance(n, NameWithRealValue):
			sp = self.cast_to_speed(n)
			self._ast.set(index, sp)
			return sp
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Speed')

	def begin_lane_coordinate(self, lc: LaneCoordinate):
		pass

	def end_lane_coordinate(self, lc: LaneCoordinate):
		assert self._current._lane is not None
		lc.set_lane(self._current._lane)

	def find_lane(self, name: AnyStr, info: Tuple[int, int]):
		n, index = self._ast.find_node(name)
		if isinstance(n, Lane):
			return n
		elif isinstance(n, NameWithString):
			l = self.cast_to_lane(n)
			self._ast.set(index, l)
			return l
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Lane')

	def find_heading(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Heading):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Heading')

	def begin_heading(self, value: float, h: Heading):
		h.set_raw_heading_angle(value)

	def end_heading(self, h: Heading):
		if self._current._direction is not None:
			h.set_direction(self._current._direction)

	def begin_direction(self, d: Union[PredefinedDirection, CustomizedDirection], is_pre: bool = True
						, frame: AnyStr=''):
		if not is_pre:
			if frame == 'IMU':
				d.set_frame(CoordinateFrame.CF_IMU)
			elif frame == 'ENU':
				d.set_frame(CoordinateFrame.CF_ENU)
			elif frame == 'WGS84':
				d.set_frame(CoordinateFrame.CF_WGS84)

	def end_direction(self, d: Union[PredefinedDirection, CustomizedDirection], is_pre: bool):
		if is_pre:
			assert self._current._lane is not None
			d.set_lane(self._current._lane)

	def find_vehicle_type(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, VehicleType):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'VehicleType')

	def find_type(self, name: AnyStr, info: Tuple[int, int]):
		n, index = self._ast.find_node(name)
		if isinstance(n, Type):
			return n
		elif isinstance(n, NameWithString):
			t = self.cast_to_type(n)
			self._ast.set(index, t)
			return t
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Type')

	def find_color(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Color):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Color')

	def begin_npc_vehicle(self, nv: NPCVehicle):
		pass

	def end_npc_vehicle(self, nv: NPCVehicle):
		assert self._current._states._first is not None
		nv.set_first_state(self._current._states._first)
		if self._current._states._second is not None:
			nv.set_second_state(self._current._states._second)
		if self._current._name_with_motion is not None:
			# motion is either waypointmotion or uniformmotion.
			# so we must build vehicle motion
			vm = VehicleMotion(self._current._name_with_motion.get_motion()
							   , self._current._name_with_motion.get_name())
			nv.setVehicleMotion(vm)
		if self._current._vehicle_type is not None:
			nv.set_vehicle_type(self._current._vehicle_type)

	def find_npc_vehicle(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, NPCVehicle):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'NPCVehicle')

	# XXX: we must return index,because when we figure
	# it as pedestrian motion or vehicle motion,we should
	# cast it to relative motion object. 
	def find_motion(self, name: AnyStr, info: Tuple[int, int]) -> Tuple[
		Union[PedestrianMotion, VehicleMotion, NameWithMotion], int]:

		n, index = self._ast.find_node(name)
		if isinstance(n, PedestrianMotion) or isinstance(n, VehicleMotion) \
				or isinstance(n, NameWithMotion):
			return n, index
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Motion')

	def begin_motion(self, m: NameWithMotion, isUniform: bool):
		pass

	def end_motion(self, m: NameWithMotion, isUniform: bool, ):
		if isUniform:
			# here we do not need set index.
			assert self._current._states._value is not None
			m.get_motion().set_state(self._current._states._value)
		else:
			assert self._current._state_list is not None
			m.get_motion().set_state_list(self._current._state_list)

	def find_state_list(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, StateList):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'StateList')

	def find_pedestrian(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Pedestrian):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Pedestrian')

	def begin_pedestrian(self, p: Pedestrian):
		pass

	def end_pedestrian(self, p: Pedestrian):
		assert self._current._states._first is not None
		p.set_first_state(self._current._states._first)
		if self._current._states._second is not None:
			p.set_second_state(self._current._states._second)
		if self._current._name_with_motion is not None:
			# motion is either waypointmotion or uniformmotion.
			# so we must build pedestrian motion
			pm = PedestrianMotion(self._current._name_with_motion.get_motion()
								  , self._current._name_with_motion.get_name())
			p.set_pedestrian_motion(pm)
		if self._current._pedestrian_type is not None:
			p.set_vehicle_type(self._current._pedestrian_type)

	def find_height(self, name: AnyStr, info: Tuple[int, int]):
		n, index = self._ast.find_node(name)
		if isinstance(n, Height):
			return n
		if isinstance(n, NameWithRealValue):
			h = self.cast_to_height(n)
			self._ast.set(index, h)
			return h
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Height')

	def find_obstacle(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Obstacle):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Obstacle')

	def begin_obstacle(self, o: Obstacle):
		pass

	def end_obstacle(self, o: Obstacle):
		assert self._current._position is not None
		o.set_position(self._current._position)
		if self._current._shape is not None:
			o.set_shape(self._current._shape)

	def find_shape(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Shape):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Shape')

	def find_env(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Environment):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Environment')

	def begin_env(self, e: Environment):
		pass

	def end_env(self, e: Environment):
		assert self._current._time is not None and self._current._weathers is not None
		e.set_time(self._current._time)
		e.set_weathers(self._current._weathers)

	def find_weathers(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Weathers):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Weathers')

	def find_time(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Time):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Time')

	def find_weather(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Weather):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Weather')

	def begin_weather(self, w: Weather):
		pass

	def end_weather(self, w: Weather):
		assert self._current._weather_continuous is not None \
			   or self._current._weather_discrete is not None
		if self._current._weather_continuous is not None:
			w.set_weather_kind_value(self._current._weather_continuous)
		elif self._current._weather_discrete is not None:
			w.set_weather_kind_value(self._current._weather_discrete)

	def find_weather_continuous_index(self, name: AnyStr, info: Tuple[int, int]):
		n, index = self._ast.find_node(name)
		if isinstance(n, WeatherContinuousIndex):
			return n
		if isinstance(n, NameWithRealValue):
			wi = self.cast_to_weather_continuous_index(n)
			self._ast.set(index, wi)
			return wi
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'WeatherContinuousIndex')

	def find_weather_discrete_level(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, WeatherDiscreteLevel):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'WeatherDiscreteLevel')

	def find_pedestrian_type(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, PedestrianType):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'PedestrianType')

	def find_intersection_traffic(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, IntersectionTraffic):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'IntersectionTraffic')

	def begin_intersection_traffic(self, it: IntersectionTraffic, traffic_light: int
								   , stop_sign: int, crosswalk: int):
		it.set_traffic_light(IntersectionTraffic.Sign.S_0
							 if traffic_light == 0 else IntersectionTraffic.Sign.S_1)
		it.set_traffic_light(IntersectionTraffic.Sign.S_0
							 if stop_sign == 0 else IntersectionTraffic.Sign.S_1)
		it.set_crosswalk(IntersectionTraffic.Sign.S_0
						 if crosswalk == 0 else IntersectionTraffic.Sign.S_1)

	def end_intersection_traffic(self, it: IntersectionTraffic):
		assert self._current._intersection_id is not None
		it.set_id(self._current._intersection_id)

	def find_intersection_id(self, name: AnyStr):
		n, index = self._ast.find_node(name)
		if isinstance(n, IntersectionID):
			return n
		if isinstance(n, NameWithRealValue):
			iid = self.cast_to_intersection_id(n)
			self._ast.set(index, iid)
			return iid

	def find_speed_limitation(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n._speed_limitation):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'SpeedLimitation')

	def begin_speed_limitation(self, sl: SpeedLimitation):
		pass

	def end_speed_limitation(self, sl: SpeedLimitation):
		assert self._current._lane is not None
		sl.set_lane(self._current._lane)
		assert self._current._speed_range is not None
		sl.set_speed_range(self._current._speed_range)

	def find_speed_range(self, name: AnyStr, info: Tuple[int, int]):
		n, index = self._ast.find_node(name)
		if isinstance(n, SpeedRange):
			return n
		if isinstance(n, NameWithTwoRealValues):
			sr = self.cast_to_speed_range(n)
			self._ast.set(index, sr)
			return sr
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'SpeedRange')

	def find_npc_vehicles(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, NPCVehicles):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'NPCVehicles')

	def find_pedestrians(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Pedestrians):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Pedestrians')

	def find_obstacles(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Obstacles):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Obstacles')

	def find_traffic(self, name: AnyStr, info: Tuple[int, int]):
		n, _ = self._ast.find_node(name)
		if isinstance(n, Traffic):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__,
									   'Traffic')

	# try to parse Pedestrians,Obstacles,Traffic,NPCVehicles
	# and Weather.
	def act_on_name_with_variables(self, name: AnyStr, first_arg: AnyStr, *args: AnyStr, info: Tuple[int, int]):
		self.check_unique_id(name)
		n, _ = self._ast.find_node(first_arg)
		if isinstance(n, Pedestrian):
			self.try_to_parse_pedestrians(name, n, args)
		elif isinstance(n, NPCVehicle):
			self.try_to_parse_npc_vehicles(name, n, args)
		elif isinstance(n, Obstacle):
			self.try_to_parse_obstacles(name, n, args)
		elif isinstance(n, Weather):
			self.try_to_parse_weathers(name, n, args)
		elif isinstance(n, IntersectionTraffic):
			self.try_to_parse_traffic(name, n, args)
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'Pedestrian', 'NPCVehicles', 'Obstacle', 'Weather', 'IntersectionTraffic')

	def try_to_parse_pedestrians(self, name: AnyStr, first_ele: Pedestrian, args: Tuple[AnyStr]):
		p = Pedestrians(name)
		p.add_pedestrian(first_ele)
		for v in args:
			n, _ = self._ast.find_node(v)
			if not isinstance(n, Pedestrian):
				# FIXME: add some line/column information here
				raise IllegalTypeException(n.get_name(), -1, -1, n.__class__.__name__
										   , 'Pedestrian')
			p.add_pedestrian(n)
		self._ast.add_ast_node(p)

	def try_to_parse_obstacles(self, name: AnyStr, first_ele: Obstacle, args: Tuple[AnyStr]):
		o = Obstacles(name)
		o.add_obstacle(first_ele)
		for v in args:
			n, _ = self._ast.find_node(v)
			if not isinstance(n, Obstacle):
				# FIXME: add some line/column information here
				raise IllegalTypeException(n.get_name(), -1, -1, n.__class__.__name__
										   , 'Obstacle')
			o.add_obstacle(n)
		self._ast.add_ast_node(o)

	def try_to_parse_weathers(self, name: AnyStr, first_ele: Weather, args: Tuple[AnyStr]):
		w = Weathers(name)
		w.add_weather(first_ele)
		for v in args:
			n, _ = self._ast.find_node(v)
			if not isinstance(n, Weather):
				# FIXME: add some line/column information here
				raise IllegalTypeException(n.get_name(), -1, -1, n.__class__.__name__
										   , 'Weather')
			w.add_weather(n)
		self._ast.add_ast_node(w)

	def try_to_parse_npc_vehicles(self, name: AnyStr, first_ele: NPCVehicle, args: Tuple[AnyStr]):
		np = NPCVehicles(name)
		np.add_npc_vehicle(first_ele)
		for v in args:
			n, _ = self._ast.find_node(v)
			if not isinstance(n, NPCVehicle):
				# FIXME: add some line/column information here
				raise IllegalTypeException(n.get_name(), -1, -1, n.__class__.__name__
										   , 'NPCVehicle')
			np.add_npc_vehicle(n)
		self._ast.add_ast_node(np)

	def try_to_parse_traffic(self, name: AnyStr, first_ele: IntersectionTraffic, args: Tuple[AnyStr]):
		t = Traffic(name)
		t.add_intersection_traffic(first_ele)
		is_inter = True
		for v in args:
			n, _ = self._ast.find_node(v)
			if is_inter:
				if isinstance(n, IntersectionTraffic):
					t.add_intersection_traffic(n)
				elif isinstance(n, SpeedLimitation):
					t.add_speed_limitation(n)
					is_inter = False
				else:
					# FIXME: add some line/column information here
					raise IllegalTypeException(n.get_name(), -1, -1, n.__class__.__name__
											   , 'IntersectionTraffic', 'SpeedLimitation')
			else:
				if isinstance(n, SpeedLimitation):
					t.add_speed_limitation(n)
				else:
					# FIXME: add some line/column information here
					raise IllegalTypeException(n.get_name(), -1, -1, n.__class__.__name__
											   , 'SpeedLimitation')
		self._ast.add_ast_node(t)

	def cast_to_height(self, n: NameWithRealValue) -> Height:
		return Height(n.get_value(), n.get_name())

	def cast_to_speed(self, n: NameWithRealValue) -> Speed:
		return Speed(n.get_value(), n.get_name())
	
	# FIXME: add line/column information here to diagnostics
	def cast_to_lane(self, n: NameWithString) -> Lane:
		Split=n.get_value().split('.')
		if len(Split)!=2:
			raise Exception(f"Lane:{n.get_name()} id must be the string consisting real number!")
		else:
			if Split[1]=='':
				raise Exception(f"Lane:{n.get_name()} id must be the string consisting real number!")
			for Ele in Split[0]:
				if not ('0'<=Ele<='9' or Ele=='-' or Ele=='+'):
					raise Exception(f"Lane:{n.get_name()} id must be the string consisting real number!")
			for Ele in Split[1]:
				if not ('0'<=Ele<='9' or Ele=='-' or Ele=='+'):
					raise Exception(f"Lane:{n.get_name()} id must be the string consisting real number!")
		return Lane(n.get_value(), n.get_name())
	def cast_to_weather_continuous_index(self, n: NameWithRealValue) -> WeatherContinuousIndex:
		return WeatherContinuousIndex(n.get_value(), n.get_name())

	def cast_to_intersection_id(self, n: NameWithRealValue) -> IntersectionID:
		if not isinstance(n.get_value(), int):
			# FIXME: add line/column information here
			raise Exception(f"IntersectionID:{n.get_name()} id must be integer!")
		return IntersectionID(int(n.get_value()), n.get_name())

	def cast_to_map(self, n: NameWithString) -> Map:
		return Map(n.get_value(), n.get_name())

	def cast_to_type(self, n: NameWithString) -> Type:
		t = SpecificType(n.get_value(),n.get_name())
		return t

	def cast_to_position(self, n: NameWithTwoRealValues) -> Position:
		p = Position(n.get_name())
		# default enum is 'ENU''
		p.set_frame(CoordinateFrame.CF_ENU)
		p.set_coordinate(Coordinate(n.get_value()[0], n.get_value()[1]))
		return p

	def cast_to_speed_range(self, n: NameWithTwoRealValues) -> SpeedRange:
		sr = SpeedRange(n.get_name())
		sr.set_x(n.get_value()[0])
		sr.set_y(n.get_value()[1])
		return sr

	def check_and_cast_motion(self, m: Union[PedestrianMotion, VehicleMotion, NameWithMotion]
							  , index: int, is_vehicle_motion: bool):
		if isinstance(m, PedestrianMotion) and not is_vehicle_motion:
			self._current._name_with_motion = NameWithMotion(m.get_motion())
			self._current._name_with_motion.set_name(m.get_name())
		elif isinstance(m, PedestrianMotion) and is_vehicle_motion:
			# FIXME: add line/column information here
			raise IllegalTypeException(m.get_name(), -1, -1, m.__class__.__name__
									   , 'VehicleMotion')
		elif isinstance(m, VehicleMotion) and is_vehicle_motion:
			self._current._name_with_motion = NameWithMotion(m.get_motion())
			self._current._name_with_motion.set_name(m.get_name())
		elif isinstance(m, VehicleMotion) and not is_vehicle_motion:
			# FIXME: add line/column information here
			raise IllegalTypeException(m.get_name(), -1, -1, m.__class__.__name__
									   , 'PedestrianMotion')
		elif isinstance(m, NameWithMotion) and is_vehicle_motion:
			self._current._name_with_motion = m
			# cast to ast tree
			self._ast.set(index, VehicleMotion(m.get_motion(), m.get_name()))
		elif isinstance(m, NameWithMotion) and not is_vehicle_motion:
			self._current._name_with_motion = m
			# cast to ast tree
			self._ast.set(index, PedestrianMotion(m.get_motion(), m.get_name()))

	# TODO: complete the find_ego_state
	def find_ego_state(self, name: AnyStr, info: Tuple[int, int]) -> EgoState:
		# self.check_unique_id(name)
		n, _ = self._ast.find_node(name)
		if isinstance(n, EgoState):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'Scenario')

	def find_scenario(self, name: AnyStr, info: Tuple[int, int]) -> Scenario:
		n, _ = self._ast.find_node(name)
		if isinstance(n, Scenario):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'Scenario')
	def find_trace(self,name:AnyStr, info: Tuple[int, int]) -> Trace:
		n, _ = self._ast.find_node(name)
		if isinstance(n, Trace):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'Trace')
	def find_agent_identifier(self, name:AnyStr,info:Tuple[int,int])->Union[NPCVehicle,Obstacle,Pedestrian]:
		n, _ = self._ast.find_node(name)
		if isinstance(n, NPCVehicle) or isinstance(n,Pedestrian) or isinstance(n,Obstacle):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'NPCVehicle','Obstacle','Pedestrian')

	def finish_trace(self, trace: Trace):
		self._ast.add_ast_node(trace)
		self._ast.add_trace(trace)
	def act_on_agent_ground_truth(self, name: AnyStr):
		self.check_unique_id(name)
		assert self._current._agent_ground_truth is not None
		self._current._agent_ground_truth.set_name(name)
		self.finish_agent_ground_truth(self._current._agent_ground_truth)

	def finish_agent_ground_truth(self, agt: AgentGroundTruth):
		self._ast.add_ast_node(agt)

	def act_on_agent_ground_distance(self, name:AnyStr):
		self.check_unique_id(name)
		assert self._current._agent_ground_distance is not None
		self._current._agent_ground_distance.set_name(name)
		self.finish_agent_ground_distance(self._current._agent_ground_distance)

	def finish_agent_ground_distance(self, agd: AgentGroundDistance):
		self._ast.add_ast_node(agd)

	def act_on_ego_state(self, name:AnyStr):
		self.check_unique_id(name)
		assert self._current._ego_state is not None
		self._current._ego_state.set_name(name)
		self.finish_ego_state(self._current._ego_state)

	def finish_ego_state(self, es: EgoState):
		self._ast.add_ast_node(es)

	def act_on_agent_error(self, name:AnyStr):
		self.check_unique_id(name)
		assert self._current._agent_error is not None
		self._current._agent_error.set_name(name)
		self.finish_agent_error(self._current._agent_error)

	def finish_agent_error(self, ae: AgentError):
		self._ast.add_ast_node(ae)

	def act_on_agent_state(self, name:AnyStr):
		self.check_unique_id(name)
		assert self._current._agent_state is not None
		self._current._agent_state.set_name(name)
		self.finish_agent_state(self._current._agent_state)

	def finish_agent_state(self, as_: AgentState):
		self._ast.add_ast_node(as_)

	def begin_agent_visible_detection_assertion(self, avda:AgentVisibleDetectionAssertion):
		pass
	def end_agent_visible_detection_assertion(self,avda:AgentVisibleDetectionAssertion):
		assert self._current._agent_ground_distance is not None
		avda.set_agent_ground_distance(self._current._agent_ground_distance)
	def begin_agent_ground_distance(self, agd:AgentGroundDistance):
		pass
	def end_agent_ground_distance(self,agd:AgentGroundDistance):
		assert self._current._ego_state is not None
		assert self._current._agent_ground_truth is not None
		agd.set_ego_state(self._current._ego_state )
		agd.set_agent_ground_truth(self._current._agent_ground_truth )
	def begin_agent_error(self, ae:AgentError):
		pass
	def end_agent_error(self,ae:AgentError):
		assert self._current._agent_state is not None
		assert self._current._agent_ground_truth is not None
		ae.set_agent_state(self._current._agent_state)
		ae.set_agent_ground_truth(self._current._agent_ground_truth)
	def begin_agent_error_detection_assertion(self, eda:AgentErrorDetectionAssertion):
		pass
	def end_agent_error_detection_assertion(self,aeda:AgentErrorDetectionAssertion):
		assert self._current._agent_error is not None
		aeda.set_agent_error(self._current._agent_error)

	def begin_agent_safety_assertion(self, asa: AgentSafetyAssertion):
		pass
	def end_agent_safety_assertion(self, asa:AgentSafetyAssertion):
		assert self._current._ego_state is not None
		assert self._current._agent_state is not None
		asa.set_ego_state(self._current._ego_state)
		asa.set_agent_state(self._current._agent_state)
	def begin_ego_speed(self,es:EgoSpeed):
		pass
	def end_ego_speed(self,es:EgoSpeed):
		assert self._current._coordinate is not None
		es.set_velocity(self._current._coordinate )
	def begin_detection_assertion(self,ds:DetectionAssertion):
		pass
	def end_detection_assertion(self,ds:DetectionAssertion):
		# Do not add detection assertion here,
		# because they will auto be added.
		pass
	def begin_safety_assertion(self,sa:SafetyAssertion):
		pass
	def end_safety_assertion(self,sa:SafetyAssertion):
		# Do not add detection assertion here,
		# because they will auto be added.
		pass
	def act_on_detection_assertion(self,name:AnyStr):
		self.check_unique_id(name)
		assert self._current._detection_assertion is not None
		self._current._detection_assertion.set_name(name)
		self.finish_detection_assertion(self._current._detection_assertion)
	def finish_detection_assertion(self,da:DetectionAssertion):
		self._ast.add_ast_node(da)
	def act_on_safety_assertion(self,name:AnyStr):
		self.check_unique_id(name)
		assert self._current._safety_assertion is not None
		self._current._safety_assertion.set_name(name)
		self.finish_safety_assertion(self._current._safety_assertion)
	def finish_safety_assertion(self,sa:SafetyAssertion):
		self._ast.add_ast_node(sa)
	def act_on_intersection_assertion(self,name:AnyStr):
		self.check_unique_id(name)
		assert self._current._intersection_assertion is not None
		self._current._intersection_assertion.set_name(name)
		self.finish_intersection_assertion(self._current._intersection_assertion)
	def finish_intersection_assertion(self,ia:IntersectionAssertion):
		self._ast.add_ast_node(ia)
	def act_on_speed_constraint_assertion(self,name:AnyStr):
		self.check_unique_id(name)
		assert self._current._speed_constraint_assertion is not None
		self._current._speed_constraint_assertion.set_name(name)
		self.finish_speed_constraint_assertion(self._current._speed_constraint_assertion)
	def finish_speed_constraint_assertion(self,sca:SpeedConstraintAssertion):
		self._ast.add_ast_node(sca)
	
	def find_detection_assertion(self,name:AnyStr, info: Tuple[int, int]) -> DetectionAssertion:
		n, _ = self._ast.find_node(name)
		if isinstance(n, DetectionAssertion):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'DetectionAssertion')
	def find_agent_ground_distance(self,name:AnyStr, info: Tuple[int, int]) -> AgentGroundDistance:
		n, _ = self._ast.find_node(name)
		if isinstance(n, AgentGroundDistance):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'AgentGroundDistance')
	def find_egp_state(self,name:AnyStr, info: Tuple[int, int]) -> EgoState:
		n, _ = self._ast.find_node(name)
		if isinstance(n, EgoState):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'EgoState')
	def find_agent_ground_truth(self,name:AnyStr, info: Tuple[int, int]) -> AgentGroundTruth:
		n, _ = self._ast.find_node(name)
		if isinstance(n, AgentGroundTruth):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'AgentGroundTruth')
	def find_agent_error(self,name:AnyStr, info: Tuple[int, int]) -> AgentError:
		n, _ = self._ast.find_node(name)
		if isinstance(n, AgentError):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'AgentError')
	def find_agent_state(self,name:AnyStr, info: Tuple[int, int]) -> AgentState:
		n, _ = self._ast.find_node(name)
		if isinstance(n, AgentState):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'AgentState')
	def find_safety_assertion(self,name:AnyStr, info: Tuple[int, int]) -> SafetyAssertion:
		n, _ = self._ast.find_node(name)
		if isinstance(n, SafetyAssertion):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'SafetyAssertion')
	
	def find_intersection_assertion(self,name:AnyStr, info: Tuple[int, int]) -> IntersectionAssertion:
		n, _ = self._ast.find_node(name)
		if isinstance(n, IntersectionAssertion):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'IntersectionAssertion')
	def find_speed_constraint_assertion(self,name:AnyStr, info: Tuple[int, int]) -> SpeedConstraintAssertion:
		n, _ = self._ast.find_node(name)
		if isinstance(n, SpeedConstraintAssertion):
			return n
		else:
			raise IllegalTypeException(n.get_name(), info[0], info[1], n.__class__.__name__
									   , 'SpeedConstraintAssertion')
	def handle_assertion(self,  v1: AnyStr, v2: AnyStr, v1_info: Tuple[int, int]
									   , v2_info: Tuple[int, int]):
		trace=self.find_trace(v1,v1_info)
		n,_=self._ast.find_node(v2)
		if isinstance(n,DetectionAssertion):
			trace.add_detection_assertion(n)
		elif isinstance(n,SafetyAssertion):
			trace.add_safety_assertion(n)
		elif isinstance(n,IntersectionAssertion):
			trace.add_intersection_assertion(n)
		elif isinstance(n,SpeedConstraintAssertion):
			trace.add_speed_constraint_assertion(n)
		else:
			raise IllegalTypeException(v2.get_name(), v2_info[0], v2_info[1], n.__class__.__name__
									   ,'DetectionAssertion','SafetyAssertion','IntersectionAssertion', 'SpeedConstraintAssertion')
		self.finish_assign_assertion_to_trace(trace,n)
	def finish_assign_assertion_to_trace(self,t:Trace,a:Union[DetectionAssertion,SafetyAssertion
								,IntersectionAssertion,SpeedConstraintAssertion]):
	# Construct a temporary AssignAssertionToTrace just for dumping
		tmp=AssignAssertionToTrace()
		tmp.set_trace(t)
		tmp.set_assertion(a)
		self._ast.add_ast_node(tmp)							
	# other functions
	# v:'(R,G,B)'
	# grammar syntax guarantees that we can match this successfully.
	def parse_rgb_color_internal(self, v: AnyStr) -> RGBColor:
		pattern = re.compile(r'^\( *([0-9]+) *, *([0-9]+) *, *([0-9]+) *\)$')
		result = re.match(pattern, v)
		assert result is not None
		assert len(result.groups()) == 3
		return RGBColor(int(result.group(1)), int(result.group(2)), int(result.group(3)))

	def parse_time_from_internal(self, v: AnyStr) -> Time:
		pattern = re.compile(r'^([0-9][0-9]):([0-9][0-9])$')
		result = re.match(pattern, v)
		assert result is not None
		assert len(result.groups()) == 2
		return Time(int(result.group(1)), int(result.group(2)))




class ASTListener(AVScenariosListener):

	def __init__(self, sema: Sema):
		self._sema = sema
		self._current = sema._current

	def enterEntry(self, ctx: AVScenariosParser.EntryContext):
		pass

	def exitEntry(self, ctx: AVScenariosParser.EntryContext):
		# complete the whole grammar parsing
		pass

	def enterAssigns(self, ctx: AVScenariosParser.AssignsContext):
		pass

	def exitAssigns(self, ctx: AVScenariosParser.AssignsContext):
		pass

	def enterAssign_scenario(self, ctx: AVScenariosParser.Assign_scenarioContext):
		pass

	def exitAssign_scenario(self, ctx: AVScenariosParser.Assign_scenarioContext):
		self._sema.act_on_scenario(ctx.children[0].getText())
		self._current._scenario = None

	def enterAssign_rv(self, ctx: AVScenariosParser.Assign_rvContext):
		#  ctx.children[1].getText()=='='
		self._sema.act_on_name_with_real_value(ctx.children[0].getText()
											   , ctx.children[2].getText())

	def exitAssign_rv(self, ctx: AVScenariosParser.Assign_rvContext):
		pass

	def enterAssign_str(self, ctx: AVScenariosParser.Assign_strContext):
		#  ctx.children[1].getText()=='='
		# strip the \"\"
		self._sema.act_on_name_with_string(ctx.children[0].getText()
										   , ctx.children[2].getText()[1:-1])

	def exitAssign_str(self, ctx: AVScenariosParser.Assign_strContext):
		pass

	def enterAssign_ego(self, ctx: AVScenariosParser.Assign_egoContext):
		pass

	def exitAssign_ego(self, ctx: AVScenariosParser.Assign_egoContext):
		self._sema.act_on_ego_vehicle(ctx.children[0].getText())
		self._current._ego_vehicle = None

	def enterAssign_variable(self, ctx: AVScenariosParser.Assign_variableContext):
		#  ctx.children[1].getText()=='='
		#  ctx.children[2].getText()=='('
		self._sema.act_on_name_with_one_variable(ctx.children[0].getText()
												 , ctx.children[3].getText()
												 , (ctx.children[3].start.line
													, ctx.children[3].start.column))

	def exitAssign_variable(self, ctx: AVScenariosParser.Assign_variableContext):
		pass



	def enterAssign_name_two_variables(self, ctx: AVScenariosParser.Assign_name_two_variablesContext):
		# ctx.children[1].getText()=='='
		# ctx.children[2].getText()=='('
		# ctx.children[6].getText()==')'
		self._sema.act_on_name_with_two_variables(ctx.children[0].getText()
												  , ctx.children[3].getText()
												  , ctx.children[5].getText()
												  , (ctx.children[3].start.line, ctx.children[3].start.column)
												  , (ctx.children[5].start.line, ctx.children[5].start.column))

	def exitAssign_name_two_variables(self, ctx: AVScenariosParser.Assign_name_two_variablesContext):
		pass

	def enterAssign_name_three_variables(self, ctx: AVScenariosParser.Assign_name_three_variablesContext):
		# ctx.children[1].getText()=='='
		# ctx.children[2].getText()=='('
		# ctx.children[4].getText()==','
		# ctx.children[6].getText()==','
		# ctx.children[8].getText()==')'
		self._sema.act_on_name_with_three_variables(ctx.children[0].getText()
													, ctx.children[3].getText()
													, ctx.children[5].getText()
													, ctx.children[7].getText()
													, (ctx.children[3].start.line, ctx.children[3].start.column)
													, (ctx.children[5].start.line, ctx.children[5].start.column)
													, (ctx.children[7].start.line, ctx.children[7].start.column))

	def exitAssign_name_three_variables(self, ctx: AVScenariosParser.Assign_name_three_variablesContext):
		pass
	def enterAssign_state(self, ctx: AVScenariosParser.Assign_stateContext):
		self._current._states._flag = 0
	def exitAssign_state(self, ctx: AVScenariosParser.Assign_stateContext):
		self._sema.act_on_state(ctx.children[0].getText())
		self._current._states._value = None
	def enterAssign_vehicle_type(self, ctx: AVScenariosParser.Assign_vehicle_typeContext):
		pass

	def exitAssign_vehicle_type(self, ctx: AVScenariosParser.Assign_vehicle_typeContext):
		self._sema.act_on_vehicle_type(ctx.chilren[0].getText())
		self._current._vehicle_type=None
	def enterAssign_state_list(self, ctx: AVScenariosParser.Assign_state_listContext):
		self._current._states._flag = 0

	def exitAssign_state_list(self, ctx: AVScenariosParser.Assign_state_listContext):
		self._sema.act_on_stateList(ctx.children[0].getText())
		self._current._state_list = None
		self._current._states._flag = 0
	def enterAssign_pedestrian_type(self, ctx: AVScenariosParser.Assign_pedestrian_typeContext):
		pass
	def exitAssign_pedestrian_type(self, ctx: AVScenariosParser.Assign_pedestrian_typeContext):
		self._sema.act_on_pedestrian_type(ctx.children[0].getText())
		self._current._pedestrian_type=None

	def enterAssign_rv_rv(self, ctx: AVScenariosParser.Assign_rv_rvContext):
		if len(ctx.children) == 8:
			# we meet a position
			self._sema.act_on_coordinate_position(ctx.children[0].getText()
												  , ctx.children[2].getText()
												  , float(ctx.children[4].getText())
												  , float(ctx.children[6].getText()))
		elif len(ctx.children)==10:
			# we meet a position
			if ctx.children[7].getText()=='+':
				self._sema.act_on_coordinate_position(ctx.children[0].getText()
												  , ""
												  , float(ctx.children[3].getText())
												  , float(ctx.children[5].getText())
												  , float(ctx.children[8].getText()))
			else:# ctx.children[7].getText()=='-'
				self._sema.act_on_coordinate_position(ctx.children[0].getText()
												  , ""
												  , float(ctx.children[3].getText())
												  , float(ctx.children[5].getText())
												  , -float(ctx.children[8].getText()))
		elif len(ctx.children)==11:
			# we meet a position
			if ctx.children[8].getText()=='+':
				self._sema.act_on_coordinate_position(ctx.children[0].getText()
												  , ctx.children[2].getText()
												  , float(ctx.children[4].getText())
												  , float(ctx.children[6].getText())
												  , float(ctx.children[9].getText()))
			else:#ctx.children[8].getText()=='-'
				self._sema.act_on_coordinate_position(ctx.children[0].getText()
												  , ctx.children[2].getText()
												  , float(ctx.children[4].getText())
												  , float(ctx.children[6].getText())
												  , -float(ctx.children[9].getText()))
		else:  # len(...)==7
			self._sema.act_on_name_with_two_real_values(ctx.children[0].getText()
														, float(ctx.children[3].getText())
														, float(ctx.children[5].getText()))

	def exitAssign_rv_rv(self, ctx: AVScenariosParser.Assign_rv_rvContext):
		pass

	def enterAssign_lane_rv(self, ctx: AVScenariosParser.Assign_lane_rvContext):
		pass

	def exitAssign_lane_rv(self, ctx: AVScenariosParser.Assign_lane_rvContext):
		if len(ctx.children) == 5:
			self._sema.act_on_lane_coordinate_position(ctx.children[0].getText(), '', float(ctx.children[4].getText()))
		else:  # len(...)==6
			self._sema.act_on_lane_coordinate_position(ctx.children[0].getText(), ctx.children[2].getText()
													   , float(ctx.children[5].getText()))
		self._current._lane = None

	def enterAssign_heading(self, ctx: AVScenariosParser.Assign_headingContext):
		pass

	def exitAssign_heading(self, ctx: AVScenariosParser.Assign_headingContext):
		self._sema.act_on_heading(ctx.children[0].getText())
		self._current._heading = None

	def enterAssign_general_type(self, ctx: AVScenariosParser.Assign_general_typeContext):
		pass

	def exitAssign_general_type(self, ctx: AVScenariosParser.Assign_general_typeContext):
		self._sema.act_on_type(ctx.children[0].getText())
		self._current._type = None

	def enterAssign_color(self, ctx: AVScenariosParser.Assign_colorContext):
		pass

	def exitAssign_color(self, ctx: AVScenariosParser.Assign_colorContext):
		self._sema.act_on_color(ctx.children[0].getText())
		self._current._color = None

	def enterAssign_npc(self, ctx: AVScenariosParser.Assign_npcContext):
		pass

	def exitAssign_npc(self, ctx: AVScenariosParser.Assign_npcContext):
		self._sema.act_on_npc_vehicle(ctx.children[0].getText())
		self._current._npc_vehicle = None

	def enterAssign_uniform_motion(self, ctx: AVScenariosParser.Assign_uniform_motionContext):
		self._current._states._flag = 0

	def exitAssign_uniform_motion(self, ctx: AVScenariosParser.Assign_uniform_motionContext):
		self._sema.act_on_name_with_motion(ctx.children[0].getText())
		self._current._name_with_motion = None
		self._current._states._flag = 0

	def enterAssign_waypoint_motion(self, ctx: AVScenariosParser.Assign_waypoint_motionContext):
		self._current._states._flag = 0

	def exitAssign_waypoint_motion(self, ctx: AVScenariosParser.Assign_waypoint_motionContext):
		self._sema.act_on_name_with_motion(ctx.children[0].getText())
		self._current._name_with_motion = None
		self._current._states._flag = 0



	def enterAssign_variables(self, ctx: AVScenariosParser.Assign_variablesContext):
		# ctx.children[1].getText()=='='
		# ctx.children[2].getText()=='('
		# ctx.children[4].getText()==','
		size = len(ctx.children)
		# size>=5
		values = [ctx.children[val].getText() for val in range(5, size - 1, 2)]
		self._sema.act_on_name_with_variables(ctx.children[0].getText(),
											  ctx.children[3].getText(), *values
											  , info=(ctx.children[3].start.line, ctx.children[3].start.column))

	def enterAssign_pedestrians(self, ctx: AVScenariosParser.Assign_pedestriansContext):
		pass

	def exitAssign_pedestrians(self, ctx: AVScenariosParser.Assign_pedestriansContext):
		self._sema.act_on_pedestrians(ctx.children[0].getText())
		self._current._pedestrians = None

	def enterAssign_npcs(self, ctx: AVScenariosParser.Assign_npcsContext):
		pass

	def exitAssign_npcs(self, ctx: AVScenariosParser.Assign_npcsContext):
		self._sema.act_on_npc_vehicles(ctx.children[0].getText())
		self._current._npc_vehicles = None

	def enterAssign_obstacles(self, ctx: AVScenariosParser.Assign_obstaclesContext):
		pass

	def exitAssign_obstacles(self, ctx: AVScenariosParser.Assign_obstaclesContext):
		self._sema.act_on_obstacles(ctx.children[0].getText())
		self._current._obstacles = None

	def enterAssign_weather(self, ctx: AVScenariosParser.Assign_weatherContext):
		pass

	def exitAssign_weather(self, ctx: AVScenariosParser.Assign_weatherContext):
		self._sema.act_on_weathers(ctx.children[0].getText())
		self._current._weathers = None

	def enterAssign_traffic(self, ctx: AVScenariosParser.Assign_trafficContext):
		pass

	def exitAssign_traffic(self, ctx: AVScenariosParser.Assign_trafficContext):
		self._sema.act_on_traffic(ctx.children[0].getText())
		self._current._traffic = None

	def enterAssign_ped(self, ctx: AVScenariosParser.Assign_pedContext):
		pass

	def exitAssign_ped(self, ctx: AVScenariosParser.Assign_pedContext):
		self._sema.act_on_pedestrian(ctx.children[0].getText())
		self._current._pedestrian = None

	def enterAssign_obs(self, ctx: AVScenariosParser.Assign_obsContext):
		pass

	def exitAssign_obs(self, ctx: AVScenariosParser.Assign_obsContext):
		self._sema.act_on_obstacle(ctx.children[0].getText())
		self._current._obstacle = None

	def enterAssign_shape(self, ctx: AVScenariosParser.Assign_shapeContext):
		pass

	def exitAssign_shape(self, ctx: AVScenariosParser.Assign_shapeContext):
		self._sema.act_on_shape(ctx.children[0].getText())
		self._current._shape = None

	def enterAssign_env(self, ctx: AVScenariosParser.Assign_envContext):
		pass

	def exitAssign_env(self, ctx: AVScenariosParser.Assign_envContext):
		self._sema.act_on_environment(ctx.children[0].getText())
		self._current._env = None

	def enterAssign_time(self, ctx: AVScenariosParser.Assign_timeContext):
		pass

	def exitAssign_time(self, ctx: AVScenariosParser.Assign_timeContext):
		self._sema.act_on_time(ctx.children[0].getText())
		self._current._time = None

	def enterAssign_weather_stmt(self, ctx: AVScenariosParser.Assign_weather_stmtContext):
		pass

	def exitAssign_weather_stmt(self, ctx: AVScenariosParser.Assign_weather_stmtContext):
		self._sema.act_on_weather(ctx.children[0].getText())
		self._current._weather = None

	def enterAssign_weather_discrete(self, ctx: AVScenariosParser.Assign_weather_discreteContext):
		self._sema.act_on_weather_discrete_level(ctx.children[0].getText()
												 , ctx.children[2].getText())

	def exitAssign_weather_discrete(self, ctx: AVScenariosParser.Assign_weather_discreteContext):
		pass

	def enterAssign_intersection(self, ctx: AVScenariosParser.Assign_intersectionContext):
		pass

	def exitAssign_intersection(self, ctx: AVScenariosParser.Assign_intersectionContext):
		self._sema.act_on_intersecion_traffic(ctx.children[0].getText())
		self._current._intersection_traffic = None

	def enterAssign_speed_limit(self, ctx: AVScenariosParser.Assign_speed_limitContext):
		pass

	def exitAssign_speed_limit(self, ctx: AVScenariosParser.Assign_speed_limitContext):
		self._sema.act_on_speed_limitation(ctx.children[0].getText())
		self._current._speed_limit = None

	def enterMap_load_name(self, ctx: AVScenariosParser.Map_load_nameContext):
		assert self._current._map is None
		# strip the \" \"
		self._current._map = Map(ctx.children[2].getText()[1:-1])

	def enterCreate_scenario(self, ctx: AVScenariosParser.Create_scenarioContext):
		assert self._current._scenario is None
		self._current._scenario = Scenario(ctx.children[0].getText())
		self._sema.begin_scenario(self._current._scenario)

	def exitCreate_scenario(self, ctx: AVScenariosParser.Create_scenarioContext):
		self._sema.end_scenario(self._current._scenario)
		# others complete added.
		self._current._map = None
		self._current._ego_vehicle = None
		self._current._npc_vehicles = None
		self._current._pedestrians = None
		self._current._obstacles = None
		self._current._env = None
		self._current._traffic = None

	def enterNpc_npc(self, ctx: AVScenariosParser.Npc_npcContext):
		pass

	def exitNpc_npc(self, ctx: AVScenariosParser.Npc_npcContext):
		pass

	def enterNpc_var(self, ctx: AVScenariosParser.Npc_varContext):
		assert self._current._npc_vehicles is None
		self._current._npc_vehicles = self._sema.find_npc_vehicles(ctx.children[0].getText()
																   , (ctx.children[0].start.line,
																	  ctx.children[0].start.column))

	def exitNpc_var(self, ctx: AVScenariosParser.Npc_varContext):
		pass

	def enterNpc_empty(self, ctx: AVScenariosParser.Npc_emptyContext):
		pass

	def exitNpc_empty(self, ctx: AVScenariosParser.Npc_emptyContext):
		pass

	def enterPedestrians_ped(self, ctx: AVScenariosParser.Pedestrians_pedContext):
		pass

	def exitPedestrians_ped(self, ctx: AVScenariosParser.Pedestrians_pedContext):
		pass

	def enterPedestrians_var(self, ctx: AVScenariosParser.Pedestrians_varContext):
		assert self._current._pedestrians is None
		self._current._pedestrians = self._sema.find_pedestrians(ctx.children[0].getText()
																 , (ctx.children[0].start.line,
																	ctx.children[0].start.column))

	def exitPedestrians_var(self, ctx: AVScenariosParser.Pedestrians_varContext):
		pass

	def enterPedestrians_empty(self, ctx: AVScenariosParser.Pedestrians_emptyContext):
		pass

	def exitPedestrians_empty(self, ctx: AVScenariosParser.Pedestrians_emptyContext):
		pass

	def enterObstacles_obs(self, ctx: AVScenariosParser.Obstacles_obsContext):
		pass

	def exitObstacles_obs(self, ctx: AVScenariosParser.Obstacles_obsContext):
		pass

	def enterObstacles_var(self, ctx: AVScenariosParser.Obstacles_varContext):
		assert self._current._obstacles is None
		self._current._obstacles = self._sema.find_obstacles(ctx.children[0].getText()
															 ,
															 (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitObstacles_var(self, ctx: AVScenariosParser.Obstacles_varContext):
		pass

	def enterTraffic_tra(self, ctx: AVScenariosParser.Traffic_traContext):
		pass

	def exitTraffic_tra(self, ctx: AVScenariosParser.Traffic_traContext):
		pass

	def enterTraffic_var(self, ctx: AVScenariosParser.Traffic_varContext):
		assert self._current._traffic is None
		self._current._traffic = self._sema.find_traffic(ctx.children[0].getText()
														 , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitTraffic_var(self, ctx: AVScenariosParser.Traffic_varContext):
		pass

	def enterTraffic_empty(self, ctx: AVScenariosParser.Traffic_emptyContext):
		pass

	def exitTraffic_empty(self, ctx: AVScenariosParser.Traffic_emptyContext):
		pass

	def exitMap_load_name(self, ctx: AVScenariosParser.Map_load_nameContext):
		# do not delete self._current._map
		pass

	def enterMap_load_var(self, ctx: AVScenariosParser.Map_load_varContext):
		assert self._current._map is None
		self._current._map = self._sema.find_map(ctx.children[2].getText()
												 , (ctx.children[2].start.line, ctx.children[2].start.column))

	def exitMap_load_var(self, ctx: AVScenariosParser.Map_load_varContext):
		pass
	def enterMap_name_str(self, ctx:AVScenariosParser.Map_name_strContext):
		pass
	def exitMap_name_str(self, ctx:AVScenariosParser.Map_name_strContext):
		pass
	def enterEgo_ego_vehicle(self, ctx: AVScenariosParser.Ego_ego_vehicleContext):
		pass

	def exitEgo_ego_vehicle(self, ctx: AVScenariosParser.Ego_ego_vehicleContext):
		pass

	def enterEgo_ego_var(self, ctx: AVScenariosParser.Ego_ego_varContext):
		assert self._current._ego_vehicle is None
		self._current._ego_vehicle = self._sema.find_ego_vehicle(ctx.children[0].getText()
																 , (ctx.children[0].start.line,
																	ctx.children[0].start.column))

	def exitEgo_ego_var(self, ctx: AVScenariosParser.Ego_ego_varContext):
		pass

	def enterEgo_av(self, ctx: AVScenariosParser.Ego_avContext):
		assert self._current._ego_vehicle is None
		# anonymous egoVehicle
		self._current._ego_vehicle = EgoVehicle()
		self._sema.begin_ego_vehicle(self._current._ego_vehicle)

	def exitEgo_av(self, ctx: AVScenariosParser.Ego_avContext):
		self._sema.end_ego_vehicle(self._current._ego_vehicle)
		self._current._vehicle_type = None
		self._current._states._first = None
		self._current._states._second = None

	def enterPar_list_ego_(self, ctx: AVScenariosParser.Par_list_ego_Context):
		self._current._states._flag = 1

	def exitPar_list_ego_(self, ctx: AVScenariosParser.Par_list_ego_Context):
		self._current._states._flag = 0

	def enterState_state(self, ctx: AVScenariosParser.State_stateContext):
		pass

	# do not call begin_state
	def exitState_state(self, ctx: AVScenariosParser.State_stateContext):
		pass

	def enterState_state_var(self, ctx: AVScenariosParser.State_state_varContext):
		if self._current._states._flag == 0:
			assert self._current._states._value is None
			self._current._states._value = \
				self._sema.find_state(ctx.children[0].getText()
									  , (ctx.children[0].start.line, ctx.children[0].start.column))
		elif self._current._states._flag == 1:
			assert self._current._states._first is None
			self._current._states._first = \
				self._sema.find_state(ctx.children[0].getText()
									  , (ctx.children[0].start.line, ctx.children[0].start.column))
		elif self._current._states._flag == 2:
			assert self._current._states._second is None
			self._current._states._second = \
				self._sema.find_state(ctx.children[0].getText()
									  , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitState_state_var(self, ctx: AVScenariosParser.State_state_varContext):
		if self._current._states._flag == 1:
			# NOTICE: enter VehicleMotion or PedestrianMotion will
			# set the flag=2 but these motions sometimes can be omitted.
			self._current._states._flag = 2

	def enterState_position(self, ctx: AVScenariosParser.State_positionContext):
		if self._current._states._flag == 0:
			assert self._current._states._value is None
			self._current._states._value = State()
			self._sema.begin_state(self._current._states._value)
		elif self._current._states._flag == 1:
			assert self._current._states._first is None
			self._current._states._first = State()
			self._sema.begin_state(self._current._states._first)
		elif self._current._states._flag == 2:
			assert self._current._states._second is None
			self._current._states._second = State()
			self._sema.begin_state(self._current._states._second)

	def exitState_position(self, ctx: AVScenariosParser.State_positionContext):
		if self._current._states._flag == 0:
			self._sema.end_state(self._current._states._value)
		elif self._current._states._flag == 1:
			self._sema.end_state(self._current._states._first)
			# NOTICE: enter VehicleMotion or PedestrianMotion will
			# set the flag=2 but these motions sometimes can be omitted.
			self._current._states._flag = 2
		elif self._current._states._flag == 2:
			self._sema.end_state(self._current._states._second)
		self._current._position = None
		self._current._heading = None
		self._current._speed = None

	def enterState_position_heading_speed(self, ctx: AVScenariosParser.State_position_heading_speedContext):
		if self._current._states._flag == 0:
			assert self._current._states._value is None
			self._current._states._value = State()
			self._sema.begin_state(self._current._states._value)
		elif self._current._states._flag == 1:
			assert self._current._states._first is None
			self._current._states._first = State()
			self._sema.begin_state(self._current._states._first)
		elif self._current._states._flag == 2:
			assert self._current._states._second is None
			self._current._states._second = State()
			self._sema.begin_state(self._current._states._second)

	def exitState_position_heading_speed(self, ctx: AVScenariosParser.State_position_heading_speedContext):
		if self._current._states._flag == 0:
			self._sema.end_state(self._current._states._value)
		elif self._current._states._flag == 1:
			self._sema.end_state(self._current._states._first)
			# NOTICE: enter VehicleMotion or PedestrianMotion will
			# set the flag=2 but these motions sometimes can be omitted.
			self._current._states._flag = 2
		elif self._current._states._flag == 2:
			self._sema.end_state(self._current._states._second)
		self._current._position = None
		self._current._heading = None
		self._current._speed = None

	def enterPos_coor_coor(self, ctx: AVScenariosParser.Pos_coor_coorContext):
		assert self._current._position is None
		# anonymous position
		self._current._position = Position()
		self._sema.begin_position(self._current._position)

	def exitPos_coor_coor(self, ctx: AVScenariosParser.Pos_coor_coorContext):
		if len(ctx.children) == 2:
			self._sema.end_position(self._current._position, ctx.children[0].getText())
		else:  # len(...)==1
			self._sema.end_position(self._current._position)
		self._current._lane_coordinate = None
		self._current._coordinate = None
	def enterCoor_imu(self, ctx:AVScenariosParser.Coor_imuContext):
		pass
	def exitCoor_imu(self, ctx:AVScenariosParser.Coor_imuContext):
		pass
	def enterCoor_enu(self, ctx:AVScenariosParser.Coor_enuContext):
		pass
	def exitCoor_enu(self, ctx:AVScenariosParser.Coor_enuContext):
		pass
	def enterCoor_wgs84(self, ctx:AVScenariosParser.Coor_wgs84Context):
		pass
	def exitCoor_wgs84(self, ctx:AVScenariosParser.Coor_wgs84Context):
		pass
	def enterPos_pos(self, ctx: AVScenariosParser.Pos_posContext):
		pass

	def exitPos_pos(self, ctx: AVScenariosParser.Pos_posContext):
		pass

	def enterPos_pos_var(self, ctx: AVScenariosParser.Pos_pos_varContext):
		assert self._current._position is None
		self._current._position = self._sema.find_position(ctx.children[0].getText()
														   , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitPos_pos_var(self, ctx: AVScenariosParser.Pos_pos_varContext):
		pass

	def enterSpeed_speed(self, ctx: AVScenariosParser.Speed_speedContext):
		pass

	def exitSpeed_speed(self, ctx: AVScenariosParser.Speed_speedContext):
		pass

	def enterSpeed_speed_var(self, ctx: AVScenariosParser.Speed_speed_varContext):
		assert self._current._speed is None
		self._current._speed = self._sema.find_speed(ctx.children[0].getText()
													 , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitSpeed_speed_var(self, ctx: AVScenariosParser.Speed_speed_varContext):
		pass

	def enterSpeed_rv(self, ctx: AVScenariosParser.Speed_rvContext):
		assert self._current._speed is None
		# anonymous speed.
		self._current._speed = Speed(float(ctx.children[0].getText()))

	def exitSpeed_rv(self, ctx: AVScenariosParser.Speed_rvContext):
		pass

	def enterRv(self, ctx: AVScenariosParser.RvContext):
		pass

	def exitRv(self, ctx: AVScenariosParser.RvContext):
		pass

	def enterNon_negative_rv(self, ctx: AVScenariosParser.Non_negative_rvContext):
		pass

	def exitNon_negative_rv(self, ctx: AVScenariosParser.Non_negative_rvContext):
		pass

	def enterNon_negative_float(self, ctx: AVScenariosParser.Non_negative_floatContext):
		pass

	def exitNon_negative_float(self, ctx: AVScenariosParser.Non_negative_floatContext):
		pass

	def enterNon_negative_number(self, ctx: AVScenariosParser.Non_negative_numberContext):
		pass

	def exitNon_negative_number(self, ctx: AVScenariosParser.Non_negative_numberContext):
		pass

	def enterNon_negative_conflict_0(self, ctx: AVScenariosParser.Non_negative_conflict_0Context):
		pass

	def exitNon_negative_conflict_0(self, ctx: AVScenariosParser.Non_negative_conflict_0Context):
		pass

	def enterNon_negative_conflict_1(self, ctx: AVScenariosParser.Non_negative_conflict_1Context):
		pass

	def exitNon_negative_conflict_1(self, ctx: AVScenariosParser.Non_negative_conflict_1Context):
		pass

	def enterCoor_rv_rv(self, ctx: AVScenariosParser.Coor_rv_rvContext):
		assert self._current._coordinate is None
		if len(ctx.children)==5:
			self._current._coordinate = Coordinate(float(ctx.children[1].getText())
					, float(ctx.children[3].getText()))
		else:#len(ctx.children)==8:
			if ctx.children[5].getText()=='+':
				self._current._coordinate = Coordinate(float(ctx.children[1].getText())
					, float(ctx.children[3].getText())
					, float(ctx.children[6].getText()))
			else:#ctx.children[5].getText()=='-'
				self._current._coordinate = Coordinate(float(ctx.children[1].getText())
					, float(ctx.children[3].getText())
					, -float(ctx.children[6].getText()))

	def exitCoor_rv_rv(self, ctx: AVScenariosParser.Coor_rv_rvContext):
		pass

	def enterCoor_laneID_rv(self, ctx: AVScenariosParser.Coor_laneID_rvContext):
		assert self._current._lane_coordinate is None
		self._current._lane_coordinate = LaneCoordinate(float(ctx.children[2].getText()))
		self._sema.begin_lane_coordinate(self._current._lane_coordinate)

	def exitCoor_laneID_rv(self, ctx: AVScenariosParser.Coor_laneID_rvContext):
		self._sema.end_lane_coordinate(self._current._lane_coordinate)
		self._current._lane = None

	def enterLaneID_laneID(self, ctx: AVScenariosParser.LaneID_laneIDContext):
		pass

	def exitLaneID_laneID(self, ctx: AVScenariosParser.LaneID_laneIDContext):
		pass

	def enterLaneID_laneID_var(self, ctx: AVScenariosParser.LaneID_laneID_varContext):
		assert self._current._lane is None
		self._current._lane = self._sema.find_lane(ctx.children[0].getText()
												   , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitLaneID_laneID_var(self, ctx: AVScenariosParser.LaneID_laneID_varContext):
		pass

	def enterLaneID_str(self, ctx: AVScenariosParser.LaneID_strContext):
		assert self._current._lane is None
		# Construct an anonymous NameWithString and cast it to Lane
		# Just for checking validity of this string
		# strip the \"\"
		self._current._lane = self._sema.cast_to_lane(NameWithString(ctx.children[0].getText()[1:-1],None))

	def exitLaneID_str(self, ctx: AVScenariosParser.LaneID_strContext):
		pass

	def enterHead_heading(self, ctx: AVScenariosParser.Head_headingContext):
		pass

	def exitHead_heading(self, ctx: AVScenariosParser.Head_headingContext):
		pass

	def enterHead_var(self, ctx: AVScenariosParser.Head_varContext):
		assert self._current._heading is None
		self._current._heading = self._sema.find_heading(ctx.children[0].getText()
														 , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitHead_var(self, ctx: AVScenariosParser.Head_varContext):
		pass

	def enterHead_rv(self, ctx: AVScenariosParser.Head_rvContext):
		assert self._current._heading is None
		# anonymous heading
		if ctx.children[1].getText() == 'deg':
			self._current._heading = Heading(Unit.U_DEG)
		else:  # ctx.children[1].getText()=='rad'
			self._current._heading = Heading(Unit.U_RAD)
		self._sema.begin_heading(ctx.children[0].getText(), self._current._heading)

	def exitHead_rv(self, ctx: AVScenariosParser.Head_rvContext):
		if len(ctx.children) == 2:
			self._sema.end_heading(self._current._heading)
		else:  # len(...)==4
			self._sema.end_heading(self._current._heading)
			self._current._direction = None

	def enterUnit_deg(self, ctx: AVScenariosParser.Unit_degContext):
		pass

	def exitUnit_deg(self, ctx: AVScenariosParser.Unit_degContext):
		pass

	def enterUnit_rad(self, ctx: AVScenariosParser.Unit_radContext):
		pass

	def exitUnit_rad(self, ctx: AVScenariosParser.Unit_radContext):
		pass

	def enterDirection_pre(self, ctx: AVScenariosParser.Direction_preContext):
		pass

	def exitDirection_pre(self, ctx: AVScenariosParser.Direction_preContext):
		pass

	def enterDirection_cus(self, ctx: AVScenariosParser.Direction_cusContext):
		pass

	def exitDirection_cus(self, ctx: AVScenariosParser.Direction_cusContext):
		pass

	def enterPre_lane(self, ctx: AVScenariosParser.Pre_laneContext):
		assert self._current._direction is None
		self._current._direction = PredefinedDirection()
		self._sema.begin_direction(self._current._direction, True)

	def exitPre_lane(self, ctx: AVScenariosParser.Pre_laneContext):
		self._sema.end_direction(self._current._direction, True)
		self._sema._lane = None

	def enterPre_ego(self, ctx: AVScenariosParser.Pre_egoContext):
		assert self._current._direction is None
		self._current._direction = PredefinedDirection()
		self._sema.begin_direction(self._current._direction, True)

	def exitPre_ego(self, ctx: AVScenariosParser.Pre_egoContext):
		pass

	def enterCus_rv_rv(self, ctx: AVScenariosParser.Cus_rv_rvContext):
		assert self._current._direction is None
		if len(ctx.children) == 5:
			self._current._direction = CustomizedDirection( float(ctx.children[1].getText())
														   , float(ctx.children[3].getText()))
			self._sema.begin_direction(self._current._direction, False)
		else:  # len(...)=7
			self._current._direction = CustomizedDirection( float(ctx.children[1].getText())
														   , float(ctx.children[3].getText()))
			self._sema.begin_direction(self._current._direction, False
									   , ctx.children[5].getText())

	def exitCus_rv_rv(self, ctx: AVScenariosParser.Cus_rv_rvContext):
		self._sema.end_direction(self._current._direction, False)
	
	def enterCus_rv_rv_rv(self, ctx: AVScenariosParser.Cus_rv_rv_rvContext):
		assert self._current._direction is None
		if len(ctx.children) == 8:
			if ctx.children[5].getText()=='+':
				self._current._direction = CustomizedDirection( float(ctx.children[1].getText())
														   , float(ctx.children[3].getText())
														   , float(ctx.children[6].getText()))
			else: #ctx.children[5].getText()=='-'
				self._current._direction = CustomizedDirection( float(ctx.children[1].getText())
														   , float(ctx.children[3].getText())
														   , -float(ctx.children[6].getText()))
			self._sema.begin_direction(self._current._direction, False)
		else:  # len(...)=10
			if ctx.children[5].getText()=='+':
				self._current._direction = CustomizedDirection( float(ctx.children[1].getText())
														   , float(ctx.children[3].getText())
														   , float(ctx.children[6].getText()))
			else:#ctx.children[5].getText()=='-'
				self._current._direction = CustomizedDirection( float(ctx.children[1].getText())
														   , float(ctx.children[3].getText())
														   , -float(ctx.children[6].getText()))
			self._sema.begin_direction(self._current._direction, False
									   , ctx.children[8].getText())

	def exitCus_rv_rv_rv(self, ctx: AVScenariosParser.Cus_rv_rvContext):
		self._sema.end_direction(self._current._direction, False) 
	def enterVehicle_vehicle_type(self, ctx: AVScenariosParser.Vehicle_vehicle_typeContext):
		pass

	def exitVehicle_vehicle_type(self, ctx: AVScenariosParser.Vehicle_vehicle_typeContext):
		pass

	def enterVehicle_vehicle_type_var(self, ctx: AVScenariosParser.Vehicle_vehicle_type_varContext):
		assert self._current._vehicle_type is None
		self._current._vehicle_type = self._sema.find_vehicle_type(ctx.children[0].getText()
																   , (ctx.children[0].start.line,
																	  ctx.children[0].start.column))

	def exitVehicle_vehicle_type_var(self, ctx: AVScenariosParser.Vehicle_vehicle_type_varContext):
		pass

	def enterVehicle_type_(self, ctx: AVScenariosParser.Vehicle_type_Context):
		assert self._current._vehicle_type is None
		# anonymous vehicle type
		self._current._vehicle_type = VehicleType()
		self._sema.begin_vehicle_type(self._current._vehicle_type)

	# do not call:self._sema.begin_vehicle_type(self._current._vehicle_type)
	def exitVehicle_type_(self, ctx: AVScenariosParser.Vehicle_type_Context):
		self._sema.end_vehicle_type(self._current._vehicle_type)
		self._current._type = None

	def enterVehicle_type_color(self, ctx: AVScenariosParser.Vehicle_type_colorContext):
		assert self._current._vehicle_type is None
		# anonymous vehicle type
		self._current._vehicle_type = VehicleType()
		self._sema.begin_vehicle_type(self._current._vehicle_type)

	# do not call:self._sema.begin_vehicle_type(self._current._vehicle_type)
	def exitVehicle_type_color(self, ctx: AVScenariosParser.Vehicle_type_colorContext):
		self._sema.end_vehicle_type(self._current._vehicle_type)
		self._current._type = None
		self._current._color = None

	def enterType_type_(self, ctx: AVScenariosParser.Type_type_Context):
		pass

	def exitType_type_(self, ctx: AVScenariosParser.Type_type_Context):
		pass

	def enterType_var(self, ctx: AVScenariosParser.Type_varContext):
		assert self._current._type is None
		self._current._type = self._sema.find_type(ctx.children[0].getText()
												   , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitType_var(self, ctx: AVScenariosParser.Type_varContext):
		pass

	def enterType_specific(self, ctx: AVScenariosParser.Type_specificContext):
		pass

	def exitType_specific(self, ctx: AVScenariosParser.Type_specificContext):
		pass

	def enterType_general(self, ctx: AVScenariosParser.Type_generalContext):
		pass

	def exitType_general(self, ctx: AVScenariosParser.Type_generalContext):
		pass

	def enterSpecific_str(self, ctx: AVScenariosParser.Specific_strContext):
		# construct an anonymous type with specific type
		assert self._current._type is None
		# strip the \" \"
		self._current._type= SpecificType(ctx.children[0].getText()[1:-1])

	def exitSpecific_str(self, ctx: AVScenariosParser.Specific_strContext):
		pass

	def enterGeneral_car(self, ctx: AVScenariosParser.General_carContext):
		assert self._current._type is None
		self._current._type= GeneralType(GeneralTypeEnum.GT_CAR)

	def exitGeneral_car(self, ctx: AVScenariosParser.General_carContext):
		pass

	def enterGeneral_bus(self, ctx: AVScenariosParser.General_busContext):
		assert self._current._type is None
		self._current._type= GeneralType(GeneralTypeEnum.GT_BUS)

	def exitGeneral_bus(self, ctx: AVScenariosParser.General_busContext):
		pass

	def enterGeneral_van(self, ctx: AVScenariosParser.General_vanContext):
		assert self._current._type is None
		self._current._type= GeneralType(GeneralTypeEnum.GT_VAN)

	def exitGeneral_van(self, ctx: AVScenariosParser.General_vanContext):
		pass

	def enterGeneral_truck(self, ctx: AVScenariosParser.General_truckContext):
		assert self._current._type is None
		self._current._type= GeneralType(GeneralTypeEnum.GT_TRUCK)
	def exitGeneral_truck(self, ctx: AVScenariosParser.General_truckContext):
		pass

	def enterGeneral_bicycle(self, ctx: AVScenariosParser.General_bicycleContext):
		assert self._current._type is None
		self._current._type= GeneralType(GeneralTypeEnum.GT_BICYCLE)

	def exitGeneral_bicycle(self, ctx: AVScenariosParser.General_bicycleContext):
		pass

	def enterGeneral_motorbicycle(self, ctx: AVScenariosParser.General_motorbicycleContext):
		assert self._current._type is None
		self._current._type= GeneralType(GeneralTypeEnum.GT_MOTORBICYCLE)

	def exitGeneral_motorbicycle(self, ctx: AVScenariosParser.General_motorbicycleContext):
		pass

	def enterGeneral_tricycle(self, ctx: AVScenariosParser.General_tricycleContext):
		assert self._current._type is None
		self._current._typ = GeneralType(GeneralTypeEnum.GT_TRICYCLE)

	def exitGeneral_tricycle(self, ctx: AVScenariosParser.General_tricycleContext):
		pass

	def enterColor_color(self, ctx: AVScenariosParser.Color_colorContext):
		pass

	def exitColor_color(self, ctx: AVScenariosParser.Color_colorContext):
		pass

	def enterColor_var(self, ctx: AVScenariosParser.Color_varContext):
		assert self._current._color is None
		self._current._color = self._sema.find_color(ctx.children[0].getText()
													 , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitColor_var(self, ctx: AVScenariosParser.Color_varContext):
		pass

	def enterColor_color_list(self, ctx: AVScenariosParser.Color_color_listContext):
		pass

	def exitColor_color_list(self, ctx: AVScenariosParser.Color_color_listContext):
		pass

	def enterColor_rgb_color(self, ctx: AVScenariosParser.Color_rgb_colorContext):
		pass

	def exitColor_rgb_color(self, ctx: AVScenariosParser.Color_rgb_colorContext):
		pass

	def enterColor_red(self, ctx: AVScenariosParser.Color_redContext):
		assert self._current._color is None
		self._current._color = ColorList(ColorListEnum.CL_RED)

	def exitColor_red(self, ctx: AVScenariosParser.Color_redContext):
		pass

	def enterColor_green(self, ctx: AVScenariosParser.Color_greenContext):
		assert self._current._color is None
		self._current._color = ColorList(ColorListEnum.CL_GREEN)

	def exitColor_green(self, ctx: AVScenariosParser.Color_greenContext):
		pass

	def enterColor_blue(self, ctx: AVScenariosParser.Color_blueContext):
		assert self._current._color is None
		self._current._color = ColorList(ColorListEnum.CL_BLUE)

	def exitColor_blue(self, ctx: AVScenariosParser.Color_blueContext):
		pass

	def enterColor_black(self, ctx: AVScenariosParser.Color_blackContext):
		assert self._current._color is None
		self._current._color = ColorList(ColorListEnum.CL_BLACK)

	def exitColor_black(self, ctx: AVScenariosParser.Color_blackContext):
		pass

	def enterColor_white(self, ctx: AVScenariosParser.Color_whiteContext):
		assert self._current._color is None
		self._current._color = ColorList(ColorListEnum.CL_WHITE)
	def exitColor_white(self, ctx: AVScenariosParser.Color_whiteContext):
		pass

	def enterRgb_rgb(self, ctx: AVScenariosParser.Rgb_rgbContext):
		assert self._current._color is None
		self._current._color= self._sema.parse_rgb_color_internal(ctx.children[0].getText())

	def exitRgb_rgb(self, ctx: AVScenariosParser.Rgb_rgbContext):
		pass

	def enterNpc(self, ctx: AVScenariosParser.NpcContext):
		assert self._current._npc_vehicles is None
		self._current._npc_vehicles = NPCVehicles()

	def exitNpc(self, ctx: AVScenariosParser.NpcContext):
		# when we finish multinpc, we will add ,
		# therefore do not need to addVehicle here
		pass

	def enterMulti_npc(self, ctx: AVScenariosParser.Multi_npcContext):
		pass

	def exitMulti_npc(self, ctx: AVScenariosParser.Multi_npcContext):
		self._current._npc_vehicles.add_npc_vehicle(self._current._npc_vehicle)
		self._current._npc_vehicle = None

	def enterMulti_multi_npc(self, ctx: AVScenariosParser.Multi_multi_npcContext):
		pass

	def exitMulti_multi_npc(self, ctx: AVScenariosParser.Multi_multi_npcContext):
		self._current._npc_vehicles.add_npc_vehicle(self._current._npc_vehicle)
		self._current._npc_vehicle = None

	def enterNpc_vehicle_par(self, ctx: AVScenariosParser.Npc_vehicle_parContext):
		assert self._current._npc_vehicle is None
		self._current._states._flag = 1
		self._current._npc_vehicle = NPCVehicle()
		self._sema.begin_npc_vehicle(self._current._npc_vehicle)

	def exitNpc_vehicle_par(self, ctx: AVScenariosParser.Npc_vehicle_parContext):
		self._sema.end_npc_vehicle(self._current._npc_vehicle)
		self._current._vehicle_type = None
		self._current._name_with_motion = None
		self._current._states._first = None
		self._current._states._second = None
		self._current._states._flag = 0

	def enterNpc_npc_vehicle(self, ctx: AVScenariosParser.Npc_npc_vehicleContext):
		pass

	def exitNpc_npc_vehicle(self, ctx: AVScenariosParser.Npc_npc_vehicleContext):
		pass

	def enterNpc_npc_vehicle_var(self, ctx: AVScenariosParser.Npc_npc_vehicle_varContext):
		assert self._current._npc_vehicle is None
		self._current._npc_vehicle = self._sema.find_npc_vehicle(ctx.children[0].getText()
																 , (ctx.children[0].start.line,
																	ctx.children[0].start.column))

	def exitNpc_npc_vehicle_var(self, ctx: AVScenariosParser.Npc_npc_vehicle_varContext):
		pass

	def enterPar_npc_state(self, ctx: AVScenariosParser.Par_npc_stateContext):
		pass

	def exitPar_npc_state(self, ctx: AVScenariosParser.Par_npc_stateContext):
		pass

	def enterPar_npc_state_vehicle(self, ctx: AVScenariosParser.Par_npc_state_vehicleContext):
		pass

	def exitPar_npc_state_vehicle(self, ctx: AVScenariosParser.Par_npc_state_vehicleContext):
		pass

	def enterPar_npc_state_vehicle_state(self, ctx: AVScenariosParser.Par_npc_state_vehicle_stateContext):
		pass

	def exitPar_npc_state_vehicle_state(self, ctx: AVScenariosParser.Par_npc_state_vehicle_stateContext):
		pass

	def enterVehicle_vehicle_motion(self, ctx: AVScenariosParser.Vehicle_vehicle_motionContext):
		self._current._states._flag = 0

	def exitVehicle_vehicle_motion(self, ctx: AVScenariosParser.Vehicle_vehicle_motionContext):
		self._current._states._flag = 2

	def enterVehicle_vehicle_motion_var(self, ctx: AVScenariosParser.Vehicle_vehicle_motion_varContext):
		assert self._current._name_with_motion is None
		self._current._states._flag = 0
		m, index = self._sema.find_motion(ctx.children[0].getText()
										  , (ctx.children[0].start.line, ctx.children[0].start.column))
		self._sema.check_and_cast_motion(m, index, True)

	def exitVehicle_vehicle_motion_var(self, ctx: AVScenariosParser.Vehicle_vehicle_motion_varContext):
		self._current._states._flag = 2

	def enterVehicle_motion_uniform(self, ctx: AVScenariosParser.Vehicle_motion_uniformContext):
		pass

	def exitVehicle_motion_uniform(self, ctx: AVScenariosParser.Vehicle_motion_uniformContext):
		pass

	def enterVehicle_motion_waypoint(self, ctx: AVScenariosParser.Vehicle_motion_waypointContext):
		pass

	def exitVehicle_motion_waypoint(self, ctx: AVScenariosParser.Vehicle_motion_waypointContext):
		pass

	def enterUniform(self, ctx: AVScenariosParser.UniformContext):
		self._current._states._flag = 0
		assert self._current._name_with_motion is None
		self._current._name_with_motion = NameWithMotion(UniformMotion())
		self._sema.begin_motion(self._current._name_with_motion, True)

	def exitUniform(self, ctx: AVScenariosParser.UniformContext):
		# here we do not need set index
		self._sema.end_motion(self._current._name_with_motion, True)
		self._current._states._value = None

	def enterUniform_Uniform(self, ctx: AVScenariosParser.Uniform_UniformContext):
		self._current._name_with_motion.get_motion().set_uniform_index(UniformIndex.UI_Uniform)

	def exitUniform_Uniform(self, ctx: AVScenariosParser.Uniform_UniformContext):
		pass

	def enterUniform_uniform(self, ctx: AVScenariosParser.Uniform_uniformContext):
		self._current._name_with_motion.get_motion().set_uniform_index(UniformIndex.UI_uniform)

	def exitUniform_uniform(self, ctx: AVScenariosParser.Uniform_uniformContext):
		pass

	def enterUniform_U(self, ctx: AVScenariosParser.Uniform_UContext):
		self._current._name_with_motion.get_motion().set_uniform_index(UniformIndex.UI_U)

	def exitUniform_U(self, ctx: AVScenariosParser.Uniform_UContext):
		pass

	def enterUniform_u(self, ctx: AVScenariosParser.Uniform_uContext):
		self._current._name_with_motion.get_motion().set_uniform_index(UniformIndex.UI_u)

	def exitUniform_u(self, ctx: AVScenariosParser.Uniform_uContext):
		pass

	def enterWaypoint(self, ctx: AVScenariosParser.WaypointContext):
		self._current._states._flag = 0
		assert self._current._name_with_motion is None
		self._current._name_with_motion = NameWithMotion(WaypointMotion())
		self._sema.begin_motion(self._current._name_with_motion, False)

	def exitWaypoint(self, ctx: AVScenariosParser.WaypointContext):
		# here we do not need set index
		self._sema.end_motion(self._current._name_with_motion, False)
		self._current._state_list = None

	def enterWaypoint_Waypoint(self, ctx: AVScenariosParser.Waypoint_WaypointContext):
		self._current._name_with_motion.get_motion().set_waypoint_index(WaypointIndex.WI_Waypoint)

	def exitWaypoint_Waypoint(self, ctx: AVScenariosParser.Waypoint_WaypointContext):
		pass

	def enterWaypoint_W(self, ctx: AVScenariosParser.Waypoint_WContext):
		self._current._name_with_motion.get_motion().set_waypoint_index(WaypointIndex.WI_W)

	def exitWaypoint_W(self, ctx: AVScenariosParser.Waypoint_WContext):
		pass

	def enterWaypoint_WP(self, ctx: AVScenariosParser.Waypoint_WPContext):
		self._current._name_with_motion.get_motion().set_waypoint_index(WaypointIndex.WI_WP)

	def exitWaypoint_WP(self, ctx: AVScenariosParser.Waypoint_WPContext):
		pass

	def enterWaypoint_wp(self, ctx: AVScenariosParser.Waypoint_wpContext):
		self._current._name_with_motion.get_motion().set_waypoint_index(WaypointIndex.WI_wp)

	def exitWaypoint_wp(self, ctx: AVScenariosParser.Waypoint_wpContext):
		pass

	def enterWaypoint_waypoint(self, ctx: AVScenariosParser.Waypoint_waypointContext):
		self._current._name_with_motion.get_motion().set_waypoint_index(WaypointIndex.WI_waypoint)

	def exitWaypoint_waypoint(self, ctx: AVScenariosParser.Waypoint_waypointContext):
		pass

	def enterWaypoint_w(self, ctx: AVScenariosParser.Waypoint_wContext):
		self._current._name_with_motion.get_motion().set_waypoint_index(WaypointIndex.WI_w)

	def exitWaypoint_w(self, ctx: AVScenariosParser.Waypoint_wContext):
		pass

	def enterState_state_list(self, ctx: AVScenariosParser.State_state_listContext):
		pass

	def exitState_state_list(self, ctx: AVScenariosParser.State_state_listContext):
		pass

	def enterState_state_list_var(self, ctx: AVScenariosParser.State_state_list_varContext):
		assert self._current._state_list is None
		self._current._state_list = self._sema.find_state_list(ctx.children[0].getText()
															   , (ctx.children[0].start.line,
																  ctx.children[0].start.column))

	def exitState_state_list_var(self, ctx: AVScenariosParser.State_state_list_varContext):
		pass

	def enterState_list_multi(self, ctx: AVScenariosParser.State_list_multiContext):
		self._current._states._flag = 0
		self._current._state_list = StateList()

	def exitState_list_multi(self, ctx: AVScenariosParser.State_list_multiContext):
		pass

	def enterMulti_states_par(self, ctx: AVScenariosParser.Multi_states_parContext):
		pass

	def exitMulti_states_par(self, ctx: AVScenariosParser.Multi_states_parContext):
		# NOTICE:add state to the StateList and release it
		assert self._current._states._value is not None
		self._current._state_list.add_state(self._current._states._value)
		self._current._states._value = None

	def enterMulti_states_par_state(self, ctx: AVScenariosParser.Multi_states_par_stateContext):
		pass

	def exitMulti_states_par_state(self, ctx: AVScenariosParser.Multi_states_par_stateContext):
		# NOTICE:add state to the StateList and release it
		assert self._current._states._value is not None
		self._current._state_list.add_state(self._current._states._value)
		self._current._states._value = None

	def enterPedestrians_multi(self, ctx: AVScenariosParser.Pedestrians_multiContext):
		assert self._current._pedestrians is None
		self._current._pedestrians = Pedestrians()

	def exitPedestrians_multi(self, ctx: AVScenariosParser.Pedestrians_multiContext):
		# when we finish multiped, we will add ,
		# therefore do not need to add_pedestrian here
		pass

	def enterMulti_pedestrian(self, ctx: AVScenariosParser.Multi_pedestrianContext):
		pass

	def exitMulti_pedestrian(self, ctx: AVScenariosParser.Multi_pedestrianContext):
		self._current._pedestrians.add_pedestrian(self._current._pedestrian)
		self._current._pedestrian = None

	def enterMulti_multi_pedestrian(self, ctx: AVScenariosParser.Multi_multi_pedestrianContext):
		pass

	def exitMulti_multi_pedestrian(self, ctx: AVScenariosParser.Multi_multi_pedestrianContext):
		self._current._pedestrians.add_pedestrian(self._current._pedestrian)
		self._current._pedestrian = None

	def enterPedestrian_pedestrian(self, ctx: AVScenariosParser.Pedestrian_pedestrianContext):
		pass

	def exitPedestrian_pedestrian(self, ctx: AVScenariosParser.Pedestrian_pedestrianContext):
		pass

	def enterPedestrian_pedestrian_var(self, ctx: AVScenariosParser.Pedestrian_pedestrian_varContext):
		assert self._current._pedestrian is None
		self._current._pedestrian = self._sema.find_pedestrian(ctx.children[0].getText()
															   , (ctx.children[0].start.line,
																  ctx.children[0].start.column))

	def exitPedestrian_pedestrian_var(self, ctx: AVScenariosParser.Pedestrian_pedestrian_varContext):
		pass

	def enterPedestrian_par(self, ctx: AVScenariosParser.Pedestrian_parContext):
		assert self._current._pedestrian is None
		self._current._pedestrian = Pedestrian()
		self._current._states._flag = 1
		self._sema.begin_pedestrian(self._current._pedestrian)

	def exitPedestrian_par(self, ctx: AVScenariosParser.Pedestrian_parContext):
		self._sema.end_pedestrian(self._current._pedestrian)
		self._current._states._first = None
		self._current._states._second = None
		self._current._name_with_motion = None
		self._current._pedestrian_type = None

	def enterPar_ped_state(self, ctx: AVScenariosParser.Par_ped_stateContext):
		pass

	def exitPar_ped_state(self, ctx: AVScenariosParser.Par_ped_stateContext):
		pass

	def enterPar_ped_state_ped(self, ctx: AVScenariosParser.Par_ped_state_pedContext):
		pass

	def exitPar_ped_state_ped(self, ctx: AVScenariosParser.Par_ped_state_pedContext):
		pass

	def enterPar_ped_state_ped_state(self, ctx: AVScenariosParser.Par_ped_state_ped_stateContext):
		pass

	def exitPar_ped_state_ped_state(self, ctx: AVScenariosParser.Par_ped_state_ped_stateContext):
		pass

	def enterPedestrian_motion_pedestrian(self, ctx: AVScenariosParser.Pedestrian_motion_pedestrianContext):
		self._current._states._flag = 0

	def exitPedestrian_motion_pedestrian(self, ctx: AVScenariosParser.Pedestrian_motion_pedestrianContext):
		self._current._states._flag = 2

	def enterPedestrian_motion_pedestrian_var(self, ctx: AVScenariosParser.Pedestrian_motion_pedestrian_varContext):
		assert self._current._name_with_motion is None
		self._current._states._flag = 0
		m, index = self._sema.find_motion(ctx.children[0].getText()
										  , (ctx.children[0].start.line, ctx.children[0].start.column))
		self._sema.check_and_cast_motion(m, index, False)

	def exitPedestrian_motion_pedestrian_var(self, ctx: AVScenariosParser.Pedestrian_motion_pedestrian_varContext):
		self._current._states._flag = 2

	def enterPedestrian_uniform(self, ctx: AVScenariosParser.Pedestrian_uniformContext):
		pass

	def exitPedestrian_uniform(self, ctx: AVScenariosParser.Pedestrian_uniformContext):
		pass

	def enterPedestrian_waypoint(self, ctx: AVScenariosParser.Pedestrian_waypointContext):
		pass

	def exitPedestrian_waypoint(self, ctx: AVScenariosParser.Pedestrian_waypointContext):
		pass

	def enterPedestrian_pedestrian_type(self, ctx: AVScenariosParser.Pedestrian_pedestrian_typeContext):
		pass

	def exitPedestrian_pedestrian_type(self, ctx: AVScenariosParser.Pedestrian_pedestrian_typeContext):
		pass

	def enterPedestrian_pedestrian_type_var(self, ctx: AVScenariosParser.Pedestrian_pedestrian_type_varContext):
		assert self._current._pedestrian_type is None
		self._current._pedestrian_type = self._sema.find_pedestrian_type(ctx.children[0].getText()
																		 , (ctx.children[0].start.line,
																			ctx.children[0].start.column))

	def exitPedestrian_pedestrian_type_var(self, ctx: AVScenariosParser.Pedestrian_pedestrian_type_varContext):
		pass

	def enterPedestrian_type_height_color(self, ctx: AVScenariosParser.Pedestrian_type_height_colorContext):
		assert self._current._pedestrian_type is None
		self._current._pedestrian_type = PedestrianType()
		self._sema.begin_pedestrian_type(self._current._pedestrian_type )

	# do not call:self._sema.begin_pedestrian(self._current._pedestrian_type)
	def exitPedestrian_type_height_color(self, ctx: AVScenariosParser.Pedestrian_type_height_colorContext):
		self._sema.end_pedestrian_type(self._current._pedestrian_type)
		self._current._height = None
		self._current._color = None

	def enterHeight_height(self, ctx: AVScenariosParser.Height_heightContext):
		pass

	def exitHeight_height(self, ctx: AVScenariosParser.Height_heightContext):
		pass

	def enterHeight_var(self, ctx: AVScenariosParser.Height_varContext):
		assert self._current._height is None
		self._current._height = self._sema.find_height(ctx.children[0].getText()
													   , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitHeight_var(self, ctx: AVScenariosParser.Height_varContext):
		pass

	def enterHeight_rv(self, ctx: AVScenariosParser.Height_rvContext):
		assert self._current._height is None
		# height must be real value
		self._current._height = Height(float(ctx.children[0].getText()))

	def exitHeight_rv(self, ctx: AVScenariosParser.Height_rvContext):
		pass

	def enterObstacles_multi(self, ctx: AVScenariosParser.Obstacles_multiContext):
		assert self._current._obstacles is None
		self._current._obstacles = Obstacles()

	def exitObstacles_multi(self, ctx: AVScenariosParser.Obstacles_multiContext):
		# when we finish multiobs, we will add ,
		# therefore do not need to add_obstacle here
		pass

	def enterObstacles_empty(self, ctx: AVScenariosParser.Obstacles_emptyContext):
		pass

	def exitObstacles_empty(self, ctx: AVScenariosParser.Obstacles_emptyContext):
		pass

	def enterObstacles_obstacle(self, ctx: AVScenariosParser.Obstacles_obstacleContext):
		pass

	def exitObstacles_obstacle(self, ctx: AVScenariosParser.Obstacles_obstacleContext):
		self._current._obstacles.add_obstacle(self._current._obstacle)
		self._current._obstacle = None

	def enterObstacles_multi_obstacle(self, ctx: AVScenariosParser.Obstacles_multi_obstacleContext):
		pass

	def exitObstacles_multi_obstacle(self, ctx: AVScenariosParser.Obstacles_multi_obstacleContext):
		self._current._obstacles.add_obstacle(self._current._obstacle)
		self._current._obstacle = None

	def enterObstacle_obstacle(self, ctx: AVScenariosParser.Obstacle_obstacleContext):
		pass

	def exitObstacle_obstacle(self, ctx: AVScenariosParser.Obstacle_obstacleContext):
		pass

	def enterObstacle_obstacle_var(self, ctx: AVScenariosParser.Obstacle_obstacle_varContext):
		assert self._current._obstacle is None
		self._current._obstacle = self._sema.find_obstacle(ctx.children[0].getText()
														   , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitObstacle_obstacle_var(self, ctx: AVScenariosParser.Obstacle_obstacle_varContext):
		pass

	def enterObstacle_para(self, ctx: AVScenariosParser.Obstacle_paraContext):
		assert self._current._obstacle is None
		self._current._obstacle = Obstacle()
		self._sema.begin_obstacle(self._current._obstacle)

	def exitObstacle_para(self, ctx: AVScenariosParser.Obstacle_paraContext):
		self._sema.end_obstacle(self._current._obstacle)
		self._current._position = None
		self._current._shape = None

	def enterPar_position_shape(self, ctx: AVScenariosParser.Par_position_shapeContext):
		pass

	def exitPar_position_shape(self, ctx: AVScenariosParser.Par_position_shapeContext):
		pass

	def enterShape_shape_var(self, ctx: AVScenariosParser.Shape_shape_varContext):
		assert self._current._shape is None
		self._current._shape = self._sema.find_shape(ctx.children[0].getText()
													 , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitShape_shape_var(self, ctx: AVScenariosParser.Shape_shape_varContext):
		pass

	def enterShape_shape(self, ctx: AVScenariosParser.Shape_shapeContext):
		pass

	def exitShape_shape(self, ctx: AVScenariosParser.Shape_shapeContext):
		pass

	def enterShape_sphere(self, ctx: AVScenariosParser.Shape_sphereContext):
		pass

	def exitShape_sphere(self, ctx: AVScenariosParser.Shape_sphereContext):
		pass

	def enterShape_box(self, ctx: AVScenariosParser.Shape_boxContext):
		pass

	def exitShape_box(self, ctx: AVScenariosParser.Shape_boxContext):
		pass

	def enterShape_cone(self, ctx: AVScenariosParser.Shape_coneContext):
		pass

	def exitShape_cone(self, ctx: AVScenariosParser.Shape_coneContext):
		pass

	def enterShape_cylinder(self, ctx: AVScenariosParser.Shape_cylinderContext):
		pass

	def exitShape_cylinder(self, ctx: AVScenariosParser.Shape_cylinderContext):
		pass

	def enterSphere_sphere(self, ctx: AVScenariosParser.Sphere_sphereContext):
		assert self._current._shape is None
		self._current._shape = Sphere(ctx.children[2].getText())

	def exitSphere_sphere(self, ctx: AVScenariosParser.Sphere_sphereContext):
		pass

	def enterBox_box(self, ctx: AVScenariosParser.Box_boxContext):
		assert self._current._shape is None
		self._current._shape = Box(ctx.children[2].getText(), ctx.children[4].getText()
								   , ctx.children[6].getText())

	def exitBox_box(self, ctx: AVScenariosParser.Box_boxContext):
		pass

	def enterCone_cone(self, ctx: AVScenariosParser.Cone_coneContext):
		assert self._current._shape is None
		self._current._shape = Cone(ctx.children[2].getText(), ctx.children[4].getText())

	def exitCone_cone(self, ctx: AVScenariosParser.Cone_coneContext):
		pass

	def enterCylinder_cylinder(self, ctx: AVScenariosParser.Cylinder_cylinderContext):
		assert self._current._shape is None
		self._current._shape = Cylinder(ctx.children[2].getText(), ctx.children[4].getText())

	def exitCylinder_cylinder(self, ctx: AVScenariosParser.Cylinder_cylinderContext):
		pass

	def enterEnv_var(self, ctx: AVScenariosParser.Env_varContext):
		assert self._current._env is None
		self._current._env = self._sema.find_env(ctx.children[0].getText()
												 , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitEnv_var(self, ctx: AVScenariosParser.Env_varContext):
		pass

	def enterEnv_env(self, ctx: AVScenariosParser.Env_envContext):
		pass

	def exitEnv_env(self, ctx: AVScenariosParser.Env_envContext):
		pass

	def enterEnv_empty(self, ctx: AVScenariosParser.Env_emptyContext):
		pass

	def exitEnv_empty(self, ctx: AVScenariosParser.Env_emptyContext):
		pass

	def enterEnv_par(self, ctx: AVScenariosParser.Env_parContext):
		assert self._current._env is None
		self._current._env = Environment()
		self._sema.begin_env(self._current._env)

	def exitEnv_par(self, ctx: AVScenariosParser.Env_parContext):
		self._sema.end_env(self._current._env)
		self._current._time = None
		self._current._weathers = None

	def enterPar_time_weather(self, ctx: AVScenariosParser.Par_time_weatherContext):
		pass

	def exitPar_time_weather(self, ctx: AVScenariosParser.Par_time_weatherContext):
		pass

	def enterWeather_var(self, ctx: AVScenariosParser.Weather_varContext):
		assert self._current._weathers is None
		self._current._weathers = self._sema.find_weathers(ctx.children[0].getText()
														   , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitWeather_var(self, ctx: AVScenariosParser.Weather_varContext):
		pass

	def enterWeather_wtr(self, ctx: AVScenariosParser.Weather_wtrContext):
		pass

	def exitWeather_wtr(self, ctx: AVScenariosParser.Weather_wtrContext):
		pass

	def enterTime_time(self, ctx: AVScenariosParser.Time_timeContext):
		pass

	def exitTime_time(self, ctx: AVScenariosParser.Time_timeContext):
		pass

	def enterTime_time_var(self, ctx: AVScenariosParser.Time_time_varContext):
		assert self._current._time is None
		self._current._time = self._sema.find_time(ctx.children[0].getText()
												   , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitTime_time_var(self, ctx: AVScenariosParser.Time_time_varContext):
		pass

	def enterTime_Time(self, ctx: AVScenariosParser.Time_TimeContext):
		assert self._current._time is None
		self._current._time = self._sema.parse_time_from_internal(ctx.children[0].getText())

	def exitTime_Time(self, ctx: AVScenariosParser.Time_TimeContext):
		pass

	def enterWeathers(self, ctx: AVScenariosParser.WeathersContext):
		assert self._current._weathers is None
		self._current._weathers = Weathers()

	def exitWeathers(self, ctx: AVScenariosParser.WeathersContext):
		# when we finish multiweather, we will add ,
		# therefore do not need to add_weather here
		pass

	def enterWeathers_weather(self, ctx: AVScenariosParser.Weathers_weatherContext):
		pass

	def exitWeathers_weather(self, ctx: AVScenariosParser.Weathers_weatherContext):
		self._current._weathers.add_weather(self._current._weather)
		self._current._weather = None

	def enterWeathers_multi_weather(self, ctx: AVScenariosParser.Weathers_multi_weatherContext):
		pass

	def exitWeathers_multi_weather(self, ctx: AVScenariosParser.Weathers_multi_weatherContext):
		self._current._weathers.add_weather(self._current._weather)
		self._current._weather = None

	def enterWeather_weather_var(self, ctx: AVScenariosParser.Weather_weather_varContext):
		assert self._current._weather is None
		self._current._weather = self._sema.find_weather(ctx.children[0].getText()
														 , (ctx.children[0].start.line, ctx.children[0].start.column))

	def exitWeather_weather_var(self, ctx: AVScenariosParser.Weather_weather_varContext):
		pass

	def enterWeather_weather(self, ctx: AVScenariosParser.Weather_weatherContext):
		pass

	def exitWeather_weather(self, ctx: AVScenariosParser.Weather_weatherContext):
		pass

	def enterWeather_continuous(self, ctx: AVScenariosParser.Weather_continuousContext):
		assert self._current._weather is None
		self._current._weather = Weather()
		self._sema.begin_weather(self._current._weather)

	def exitWeather_continuous(self, ctx: AVScenariosParser.Weather_continuousContext):
		self._sema.end_weather(self._current._weather)
		self._current._weather_continuous = None
		self._current._weather_discrete = None

	def enterWeather_discrete(self, ctx: AVScenariosParser.Weather_discreteContext):
		assert self._current._weather is None
		self._current._weather = Weather()
		self._sema.begin_weather(self._current._weather)

	def exitWeather_discrete(self, ctx: AVScenariosParser.Weather_discreteContext):
		self._sema.end_weather(self._current._weather)
		self._current._weather_continuous = None
		self._current._weather_discrete = None

	def enterKind_sunny(self, ctx: AVScenariosParser.Kind_sunnyContext):
		self._current._weather.set_weather_kind(WeatherKind.WK_SUNNY)

	def exitKind_sunny(self, ctx: AVScenariosParser.Kind_sunnyContext):
		pass

	def enterKind_fog(self, ctx: AVScenariosParser.Kind_fogContext):
		self._current._weather.set_weather_kind(WeatherKind.WK_FOG)

	def exitKind_fog(self, ctx: AVScenariosParser.Kind_fogContext):
		pass

	def enterKind_rain(self, ctx: AVScenariosParser.Kind_rainContext):
		self._current._weather.set_weather_kind(WeatherKind.WK_RAIN)

	def exitKind_rain(self, ctx: AVScenariosParser.Kind_rainContext):
		pass

	def enterKind_snow(self, ctx: AVScenariosParser.Kind_snowContext):
		self._current._weather.set_weather_kind(WeatherKind.WK_SNOW)

	def exitKind_snow(self, ctx: AVScenariosParser.Kind_snowContext):
		pass

	def enterKind_wetness(self, ctx: AVScenariosParser.Kind_wetnessContext):
		self._current._weather.set_weather_kind(WeatherKind.WK_WETNESS)

	def exitKind_wetness(self, ctx: AVScenariosParser.Kind_wetnessContext):
		pass

	def enterWeather_continuous_value(self, ctx: AVScenariosParser.Weather_continuous_valueContext):
		assert self._current._weather_continuous is None
		v = ctx.children[0].getText()
		vf = float(v)
		if not (0 <= vf <= 1 and len(v) == 3):
			raise Exception(f'line:{ctx.children[0].start.line}:'
							f'{ctx.children[0].start.column} {vf} must be 0.0-1.0')
		else:
			self._current._weather_continuous = \
				WeatherContinuousIndex(float(ctx.children[0].getText()))

	def exitWeather_continuous_value(self, ctx: AVScenariosParser.Weather_continuous_valueContext):
		pass

	def enterWeather_continuous_var(self, ctx: AVScenariosParser.Weather_continuous_varContext):
		assert self._current._weather_continuous is None
		self._current._weather_continuous = self._sema.find_weather_continuous_index(ctx.children[0].getText()
																					 , (ctx.children[0].start.line,
																						ctx.children[0].start.column))

	def exitWeather_continuous_var(self, ctx: AVScenariosParser.Weather_continuous_varContext):
		pass

	def enterWeather_discrete_level_par(self, ctx: AVScenariosParser.Weather_discrete_level_parContext):
		assert self._current._weather_discrete is None
		if ctx.children[0].getText() == 'light':
			self._current._weather_discrete = WeatherDiscreteLevel(WeatherDiscreteLevelEnum.WDL_LIGHT)
		elif ctx.children[0].getText() == 'middle':
			self._current._weather_discrete = WeatherDiscreteLevel(WeatherDiscreteLevelEnum.WDL_MIDDLE)
		elif ctx.children[0].getText() == 'heavy':
			self._current._weather_discrete = WeatherDiscreteLevel(WeatherDiscreteLevelEnum.WDL_HEAVY)

	def exitWeather_discrete_level_par(self, ctx: AVScenariosParser.Weather_discrete_level_parContext):
		pass

	def enterWeather_discrete_light(self, ctx: AVScenariosParser.Weather_discrete_lightContext):
		pass

	def exitWeather_discrete_light(self, ctx: AVScenariosParser.Weather_discrete_lightContext):
		pass

	def enterWeather_discrete_middle(self, ctx: AVScenariosParser.Weather_discrete_middleContext):
		pass

	def exitWeather_discrete_middle(self, ctx: AVScenariosParser.Weather_discrete_middleContext):
		pass

	def enterWeather_discrete_heavy(self, ctx: AVScenariosParser.Weather_discrete_heavyContext):
		pass

	def exitWeather_discrete_heavy(self, ctx: AVScenariosParser.Weather_discrete_heavyContext):
		pass

	def enterWeather_discrete_var(self, ctx: AVScenariosParser.Weather_discrete_varContext):
		assert self._current._weather_discrete is None
		self._current._weather_discrete = self._sema.find_weather_discrete_level(ctx.children[0].getText()
																				 , (ctx.children[0].start.line,
																					ctx.children[0].start.column))

	def exitWeather_discrete_var(self, ctx: AVScenariosParser.Weather_discrete_varContext):
		pass

	def enterTraffic_traffic(self, ctx: AVScenariosParser.Traffic_trafficContext):
		pass

	def exitTraffic_traffic(self, ctx: AVScenariosParser.Traffic_trafficContext):
		pass

	def enterTraffic_stmt(self, ctx: AVScenariosParser.Traffic_stmtContext):
		pass

	def exitTraffic_stmt(self, ctx: AVScenariosParser.Traffic_stmtContext):
		pass

	def enterIntersection(self, ctx: AVScenariosParser.IntersectionContext):
		pass

	def exitIntersection(self, ctx: AVScenariosParser.IntersectionContext):
		pass

	def enterMeta_intersection_meta_var(self, ctx: AVScenariosParser.Meta_intersection_meta_varContext):
		assert self._current._intersection_traffic is None
		self._current._intersection_traffic = self._sema.find_intersection_traffic(ctx.children[0].getText()
																				   , (ctx.children[0].start.line,
																					  ctx.children[0].start.column))
		self._current._traffic.add_intersection_traffic(self._current._intersection_traffic)
		self._current._intersection_traffic = None

	def exitMeta_intersection_meta_var(self, ctx: AVScenariosParser.Meta_intersection_meta_varContext):
		pass

	def enterMeta_intersection_meta(self, ctx: AVScenariosParser.Meta_intersection_metaContext):
		pass

	def exitMeta_intersection_meta(self, ctx: AVScenariosParser.Meta_intersection_metaContext):
		# add to the traffic.
		self._current._traffic.add_intersection_traffic(self._current._intersection_traffic)
		self._current._intersection_traffic = None

	def enterMeta_intersection_intersection(self, ctx: AVScenariosParser.Meta_intersection_intersectionContext):
		assert self._current._intersection_traffic is None
		self._current._intersection_traffic = IntersectionTraffic()
		self._sema.begin_intersection_traffic(self._current._intersection_traffic
											  , int(ctx.children[4].getText()), int(ctx.children[6].getText())
											  , int(ctx.children[8].getText()))

	def exitMeta_intersection_intersection(self, ctx: AVScenariosParser.Meta_intersection_intersectionContext):
		self._sema.end_intersection_traffic(self._current._intersection_traffic)
		self._current._intersection_id = None

	def enterIntersection_intersection(self, ctx: AVScenariosParser.Intersection_intersectionContext):
		pass

	def exitIntersection_intersection(self, ctx: AVScenariosParser.Intersection_intersectionContext):
		pass

	def enterIntersection_intersection_var(self, ctx: AVScenariosParser.Intersection_intersection_varContext):
		assert self._current._intersection_id is None
		self._current._intersection_id = self._sema.find_intersection_id(ctx.children[0].getText())

	def exitIntersection_intersection_var(self, ctx: AVScenariosParser.Intersection_intersection_varContext):
		pass

	def enterIntersection_signal(self, ctx: AVScenariosParser.Intersection_signalContext):
		assert self._current._intersection_id is None
		# get the whole text.
		self._current._intersection_id = IntersectionID(ctx.getText())

	def exitIntersection_signal(self, ctx: AVScenariosParser.Intersection_signalContext):
		pass

	def enterLane_speed_limit(self, ctx: AVScenariosParser.Lane_speed_limitContext):
		pass

	def exitLane_speed_limit(self, ctx: AVScenariosParser.Lane_speed_limitContext):
		pass

	def enterLane_lane_speed_limit(self, ctx: AVScenariosParser.Lane_lane_speed_limitContext):
		pass

	def exitLane_lane_speed_limit(self, ctx: AVScenariosParser.Lane_lane_speed_limitContext):
		pass

	def enterSpeed_limit(self, ctx: AVScenariosParser.Speed_limitContext):
		pass

	def exitSpeed_limit(self, ctx: AVScenariosParser.Speed_limitContext):
		# add to the traffic.
		self._current._traffic.add_speed_limitation(self._current._speed_limit)
		self._current._speed_limit = None

	def enterSpeed_limit_var(self, ctx: AVScenariosParser.Speed_limit_varContext):
		assert self._current._speed_limit is None
		self._current._speed_limit = self._sema.find_speed_limitation(ctx.children[0].getText()
																	  , (ctx.children[0].start.line,
																		 ctx.children[0].start.column))
		self._current._traffic.add_speed_limitation(self._current._speed_limit)
		self._current._speed_limit = None

	def exitSpeed_limit_var(self, ctx: AVScenariosParser.Speed_limit_varContext):
		pass

	def enterSpeed_limit_speed_limit(self, ctx: AVScenariosParser.Speed_limit_speed_limitContext):
		assert self._current._speed_limit is None
		self._current._speed_limit = SpeedLimitation()
		self._sema.begin_speed_limitation(self._current._speed_limit)

	def exitSpeed_limit_speed_limit(self, ctx: AVScenariosParser.Speed_limit_speed_limitContext):
		self._sema.end_speed_limitation(self._current._speed_limit)
		self._current._speed_range = None
		self._current._lane = None

	def enterSpeed_range_speed(self, ctx: AVScenariosParser.Speed_range_speedContext):
		pass

	def exitSpeed_range_speed(self, ctx: AVScenariosParser.Speed_range_speedContext):
		pass

	def enterSpeed_range_var(self, ctx: AVScenariosParser.Speed_range_varContext):
		assert self._current._speed_range is None
		self._current._speed_range = self._sema.find_speed_range(ctx.children[0].getText()
																 , (ctx.children[0].start.line,
																	ctx.children[0].start.column))

	def exitSpeed_range_var(self, ctx: AVScenariosParser.Speed_range_varContext):
		pass

	def enterSpeed_range_value(self, ctx: AVScenariosParser.Speed_range_valueContext):
		assert self._current._speed_range is None
		self._current._speed_range = SpeedRange()
		self._current._speed_range.set_x(float(ctx.children[1].getText()))
		self._current._speed_range.set_y(float(ctx.children[3].getText()))

	def exitSpeed_range_value(self, ctx: AVScenariosParser.Speed_range_valueContext):
		pass
	def enterIdentifier(self, ctx: AVScenariosParser.IdentifierContext):
		pass
	def exitIdentifier(self, ctx: AVScenariosParser.IdentifierContext):
		pass
	## The following methods are designed for assertion implementation
	## TODO:
	def enterTrace_scenario(self, ctx: AVScenariosParser.Trace_scenarioContext):
		# create a trace attached with a scenario
		s = self._sema.find_scenario(ctx.children[5].getText()
									 , (ctx.children[5].start.line
										, ctx.children[5].start.column))
		self._sema.check_unique_id(ctx.children[1].getText())
		trace = Trace(ctx.children[1].getText(), s)
		self._sema.finish_trace(trace)
	def exitTrace_scenario(self, ctx: AVScenariosParser.Trace_scenarioContext):
		pass

	def enterTrace_id(self, ctx: AVScenariosParser.Trace_idContext):
		pass

	def enterTrace_detection(self, ctx: AVScenariosParser.Trace_detectionContext):
		assert self._current._detection_assertion is None
		self._current._detection_assertion=DetectionAssertion()
		self._sema.begin_detection_assertion(self._current._detection_assertion)

	def exitTrace_detection(self, ctx: AVScenariosParser.Trace_detectionContext):
		t = self._sema.find_trace(ctx.children[0].getText()
									 , (ctx.children[0].start.line
										, ctx.children[0].start.column))
		assert self._current._detection_assertion is not None
		self._sema.end_detection_assertion(self._current._detection_assertion)
		t.add_assertion(self._current._detection_assertion)
		self._sema.finish_assign_assertion_to_trace(t,self._current._detection_assertion)
		self._current._detection_assertion=None
	def enterDetection_single(self, ctx: AVScenariosParser.Detection_singleContext):
		pass

	def exitDetection_single(self, ctx: AVScenariosParser.Detection_singleContext):
		pass

	def enterDetection_detection_single(self, ctx: AVScenariosParser.Detection_detection_singleContext):
		pass

	def exitDetection_detection_single(self, ctx: AVScenariosParser.Detection_detection_singleContext):
		pass

	def enterSingle_agent(self, ctx: AVScenariosParser.Single_agentContext):
		pass

	def exitSingle_agent(self, ctx: AVScenariosParser.Single_agentContext):
		pass

	def enterSingle_traffic(self, ctx: AVScenariosParser.Single_trafficContext):
		pass

	def exitSingle_traffic(self, ctx: AVScenariosParser.Single_trafficContext):
		## After exiting this rule, add it to the DetectionAssertion
		assert self._current._detection_assertion is not None
		self._current._detection_assertion.add_assertion(self._current._traffic_detection_assert)
		self._current._traffic_detection_assert=None

	def enterAgent_visible(self, ctx: AVScenariosParser.Agent_visibleContext):
		pass

	def exitAgent_visible(self, ctx: AVScenariosParser.Agent_visibleContext):
		## Agent Detection assertion can belong to DetectionAssertion or SafetyAssertion
		assert self._current._agent_visible_assert is not None
		assert self._current._agent_error_assert is not None
		if self._current._detection_assertion is not None:
			self._current._detection_assertion.add_assertion(self._current._agent_visible_assert)
			self._current._detection_assertion.add_assertion(self._current._agent_error_assert)
		elif self._current._safety_assertion is not None:
			self._current._safety_assertion.add_assertion(self._current._agent_visible_assert)
			self._current._safety_assertion.add_assertion(self._current._agent_error_assert)
		self._current._agent_visible_assert=None
		self._current._agent_error_assert=None
	def enterAgent_visible_assert(self, ctx: AVScenariosParser.Agent_visible_assertContext):
		assert self._current._agent_visible_assert is None
		self._current._agent_visible_assert = AgentVisibleDetectionAssertion(float(ctx.children[2].getText()))
		self._sema.begin_agent_visible_detection_assertion(self._current._agent_visible_assert)
	def exitAgent_visible_assert(self, ctx: AVScenariosParser.Agent_visible_assertContext):
		self._sema.end_agent_visible_detection_assertion(self._current._agent_visible_assert)
		self._current._agent_ground_distance=None
	def enterAgent_id(self, ctx: AVScenariosParser.Agent_idContext):
		assert self._current._agent_ground_distance is None
		self._current._agent_ground_distance=self._sema.find_agent_ground_distance(ctx.children[0].getText()
									,(ctx.children[0].start.line
									,ctx.children[0].start.column))

	def exitAgent_id(self, ctx: AVScenariosParser.Agent_idContext):
		pass

	def enterAgent_par(self, ctx: AVScenariosParser.Agent_parContext):
		pass

	def exitAgent_par(self, ctx: AVScenariosParser.Agent_parContext):
		pass

	def enterAgent_ground(self, ctx: AVScenariosParser.Agent_groundContext):
		assert self._current._agent_ground_distance is None
		self._current._agent_ground_distance = AgentGroundDistance()
		self._sema.begin_agent_ground_distance(self._current._agent_ground_distance)

	def exitAgent_ground(self, ctx: AVScenariosParser.Agent_groundContext):
		self._sema.end_agent_ground_distance(self._current._agent_ground_distance)
		self._current._agent_ground_truth= None
		self._current._ego_state=None

	def enterEgo_state_id(self, ctx: AVScenariosParser.Ego_state_idContext):
		assert self._current._ego_state is None
		self._current._ego_state=self._sema.find_ego_state(ctx.children[0].getText()
									,(ctx.children[0].start.line
									,ctx.children[0].start.column))

	def exitEgo_state_id(self, ctx: AVScenariosParser.Ego_state_idContext):
		pass

	def enterEgo_state_par(self, ctx: AVScenariosParser.Ego_state_parContext):
		pass

	def exitEgo_state_par(self, ctx: AVScenariosParser.Ego_state_parContext):
		pass

	def enterEgo_state_ego(self, ctx: AVScenariosParser.Ego_state_egoContext):
		pass

	def exitEgo_state_ego(self, ctx: AVScenariosParser.Ego_state_egoContext):
		assert self._current._trace_time is not None
		assert self._current._ego_state is None
		self._current._ego_state = EgoState(self._current._trace_time)
		self._current._trace_time=None
	def enterAgent_ground_truth_id(self, ctx: AVScenariosParser.Agent_ground_truth_idContext):
		assert self._current._agent_ground_truth is None
		self._current._agent_ground_truth=self._sema.find_agent_ground_truth(ctx.children[0].getText()
									,(ctx.children[0].start.line
									,ctx.children[0].start.column))

	def exitAgent_ground_truth_id(self, ctx: AVScenariosParser.Agent_ground_truth_idContext):
		pass

	def enterAgent_ground_truth_par(self, ctx: AVScenariosParser.Agent_ground_truth_parContext):
		pass

	def exitAgent_ground_truth_par(self, ctx: AVScenariosParser.Agent_ground_truth_parContext):
		pass

	def enterAgent_ground_id(self, ctx: AVScenariosParser.Agent_ground_idContext):
		pass

	def exitAgent_ground_id(self, ctx: AVScenariosParser.Agent_ground_idContext):
		assert self._current._agent_ground_truth is None
		assert self._current._trace_time is not None
		self._current._agent_ground_truth = AgentGroundTruth(self._current._trace_time)
		agent=self._sema.find_agent_identifier(ctx.children[5].getText()
												,(ctx.children[5].start.line
												  ,ctx.children[5].start.column))
		self._current._agent_ground_truth.set_agent(agent)
		self._current._trace_time=None
	def enterTrace_number(self, ctx: AVScenariosParser.Trace_numberContext):
		assert self._current._trace_time is None
		trace=self._sema.find_trace(ctx.children[0].getText()
													,(ctx.children[0].start.line
													,ctx.children[0].start.column))
		self._current._trace_time=TraceTime(trace,int(ctx.children[2].getText()))
	def exitTrace_number(self, ctx: AVScenariosParser.Trace_numberContext):
		pass

	def enterSensing_value(self, ctx: AVScenariosParser.Sensing_valueContext):
		pass

	def exitSensing_value(self, ctx: AVScenariosParser.Sensing_valueContext):
		pass

	def enterAgent_error_assert(self, ctx: AVScenariosParser.Agent_error_assertContext):
		assert self._current._agent_error_assert is None
		self._current._agent_error_assert =AgentErrorDetectionAssertion(float(ctx.children[2].getText()))
		self._sema.begin_agent_error_detection_assertion(self._current._agent_error_assert)

	def exitAgent_error_assert(self, ctx: AVScenariosParser.Agent_error_assertContext):
		self._sema.end_agent_error_detection_assertion(self._current._agent_error_assert)
		self._current._agent_error= None

	def enterError_value(self, ctx: AVScenariosParser.Error_valueContext):
		pass

	def exitError_value(self, ctx: AVScenariosParser.Error_valueContext):
		pass

	def enterAgent_error_id(self, ctx: AVScenariosParser.Agent_error_idContext):
		assert self._current._agent_error is None
		self._current._agent_error=self._sema.find_agent_error(ctx.children[0].getText()
									,(ctx.children[0].start.line
									,ctx.children[0].start.column))

	def exitAgent_error_id(self, ctx: AVScenariosParser.Agent_error_idContext):
		pass

	def enterAgent_error_par(self, ctx: AVScenariosParser.Agent_error_parContext):
		pass

	def exitAgent_error_par(self, ctx: AVScenariosParser.Agent_error_parContext):
		pass

	def enterAgent_error_stmt(self, ctx: AVScenariosParser.Agent_error_stmtContext):
		assert self._current._agent_error is None
		self._current._agent_error = AgentError()
		self._sema.begin_agent_error(self._current._agent_error)

	def exitAgent_error_stmt(self, ctx: AVScenariosParser.Agent_error_stmtContext):
		self._sema.end_agent_error(self._current._agent_error)
		self._current._agent_ground_truth = None
		self._current._agent_state = None

	def enterAgent_state_id(self, ctx: AVScenariosParser.Agent_state_idContext):
		assert self._current._agent_state is None
		self._current._agent_state=self._sema.find_agent_state(ctx.children[0].getText()
									,(ctx.children[0].start.line
									,ctx.children[0].start.column))

	def exitAgent_state_id(self, ctx: AVScenariosParser.Agent_state_idContext):
		pass

	def enterAgent_state_par(self, ctx: AVScenariosParser.Agent_state_parContext):
		pass

	def exitAgent_state_par(self, ctx: AVScenariosParser.Agent_state_parContext):
		pass

	def enterAgent_state_trace(self, ctx: AVScenariosParser.Agent_state_traceContext):
		pass

	def exitAgent_state_trace(self, ctx: AVScenariosParser.Agent_state_traceContext):
		assert self._current._agent_state is None
		assert self._current._trace_time is not None
		self._current._agent_state = AgentState(self._current._trace_time)
		agent = self._sema.find_agent_identifier(ctx.children[5].getText()
												 , (ctx.children[5].start.line
													, ctx.children[5].start.column))
		self._current._agent_state.set_agent(agent)
		self._current._trace_time = None

	def enterTraffic_detection_assert(self, ctx: AVScenariosParser.Traffic_detection_assertContext):
		pass

	def exitTraffic_detection_assert(self, ctx: AVScenariosParser.Traffic_detection_assertContext):
		pass
	def enterTraffic_detection_assert_right(self, ctx:AVScenariosParser.Traffic_detection_assert_rightContext):
		assert self._current._trace_time is not None
		assert self._current._traffic_detection_assert is None
		self._current._traffic_detection_assert = TrafficDetectionAssertion()
		self._current._traffic_detection_assert.set_left_trace_time(self._current._trace_time)
		self._current._trace_time = None
	def exitTraffic_detection_assert_right(self, ctx:AVScenariosParser.Traffic_detection_assert_rightContext):
		assert self._current._trace_time is not None
		assert self._current._traffic_detection_assert is not None
		self._current._traffic_detection_assert.set_right_trace_time(self._current._trace_time)
		self._current._trace_time = None
	def enterTrace_safety(self, ctx: AVScenariosParser.Trace_safetyContext):
		assert self._current._safety_assertion is None
		self._current._safety_assertion=SafetyAssertion()
		self._sema.begin_safety_assertion(self._current._safety_assertion)
	def exitTrace_safety(self, ctx: AVScenariosParser.Trace_safetyContext):
		t = self._sema.find_trace(ctx.children[0].getText()
									 , (ctx.children[0].start.line
										, ctx.children[0].start.column))
		assert self._current._safety_assertion is not None
		self._sema.end_safety_assertion(self._current._safety_assertion)
		t.add_safety_assertion(self._current._safety_assertion)
		self._sema.finish_assign_assertion_to_trace(t,self._current._safety_assertion)
		self._current._safety_assertion=None
	def enterSafety_single(self, ctx: AVScenariosParser.Safety_singleContext):
		pass

	def exitSafety_single(self, ctx: AVScenariosParser.Safety_singleContext):
		pass

	def enterSafety_single_single(self, ctx: AVScenariosParser.Safety_single_singleContext):
		pass

	def exitSafety_single_single(self, ctx: AVScenariosParser.Safety_single_singleContext):
		pass

	def enterSingle_safety(self, ctx: AVScenariosParser.Single_safetyContext):
		pass

	def exitSingle_safety(self, ctx: AVScenariosParser.Single_safetyContext):
		## Add the SafetyAssertion to SafetyAssertion
		assert self._current._safety_assertion is not None
		assert self._current._agent_safety_assertion is not None
		self._current._safety_assertion.add_assertion(self._current._agent_safety_assertion)
		self._current._agent_safety_assertion=None
	def enterAgent_safety(self, ctx: AVScenariosParser.Agent_safetyContext):
		assert self._current._agent_safety_assertion is None
		self._current._agent_safety_assertion=AgentSafetyAssertion(float(ctx.children[7].getText()))
		self._sema.begin_agent_safety_assertion(self._current._agent_safety_assertion)

	def exitAgent_safety(self, ctx: AVScenariosParser.Agent_safetyContext):
		self._sema.end_agent_safety_assertion(self._current._agent_safety_assertion)
		self._current._ego_state= None
		self._current._agent_state = None

	def enterSafety_radius_value(self, ctx: AVScenariosParser.Safety_radius_valueContext):
		pass

	def exitSafety_radius_value(self, ctx: AVScenariosParser.Safety_radius_valueContext):
		pass

	def enterTrace_intersection(self, ctx: AVScenariosParser.Trace_intersectionContext):
		pass

	def exitTrace_intersection(self, ctx: AVScenariosParser.Trace_intersectionContext):
		t = self._sema.find_trace(ctx.children[0].getText()
									 , (ctx.children[0].start.line
										, ctx.children[0].start.column))
		assert self._current._intersection_assertion is not None
		t.add_intersection_assertion(self._current._intersection_assertion)
		self._sema.finish_assign_assertion_to_trace(t,self._current._intersection_assertion)
		self._current._intersection_assertion=None

	def enterRed_light_stmt(self, ctx: AVScenariosParser.Red_light_stmtContext):
		pass

	def exitRed_light_stmt(self, ctx: AVScenariosParser.Red_light_stmtContext):
		pass
	def enterRed_light_stmt_right(self, ctx:AVScenariosParser.Red_light_stmt_rightContext):
		assert self._current._traffic_detection_assert is not None
		assert self._current._red_light is not None
		assert self._current._intersection_assertion is None
		self._current._intersection_assertion = IntersectionAssertion()
		self._current._intersection_assertion.set_red_light_state(self._current._red_light)
		self._current._intersection_assertion.set_left_traffic_detection(self._current._traffic_detection_assert)
		self._current._red_light = None
		self._current._traffic_detection_assert=None
	def exitRed_light_stmt_right(self, ctx:AVScenariosParser.Red_light_stmt_rightContext):
		assert self._current._intersection_assertion is not None
		assert self._current._traffic_detection_assert is not None
		assert self._current._ego_speed is not None
		assert self._current._green_light is not None
		self._current._intersection_assertion.set_right_traffic_detection(self._current._traffic_detection_assert)
		self._current._intersection_assertion.set_ego_speed(self._current._ego_speed)
		self._current._intersection_assertion.set_green_light_state(self._current._green_light)
		self._current._traffic_detection_assert = None
		self._current._ego_speed=None
		self._current._green_light=None
	def enterRed(self, ctx: AVScenariosParser.RedContext):
		pass

	def exitRed(self, ctx: AVScenariosParser.RedContext):
		assert self._current._trace_time is not None
		assert self._current._red_light is None
		self._current._red_light =RedLightState(self._current._trace_time)
		self._current._trace_time=None

	def enterGreen(self, ctx: AVScenariosParser.GreenContext):
		pass

	def exitGreen(self, ctx: AVScenariosParser.GreenContext):
		assert self._current._trace_time is not None
		assert self._current._green_light is None
		self._current._green_light =GreenLightState(self._current._trace_time)
		self._current._trace_time=None

	def enterEgo_speed_value(self, ctx: AVScenariosParser.Ego_speed_valueContext):
		assert self._current._ego_speed is None
		self._current._ego_speed=EgoSpeed()
		self._sema.begin_ego_speed(self._current._ego_speed)

	def exitEgo_speed_value(self, ctx: AVScenariosParser.Ego_speed_valueContext):
		self._sema.end_ego_speed(self._current._ego_speed)
		self._current._coordinate=None

	def enterEgo_velocity_value(self, ctx: AVScenariosParser.Ego_velocity_valueContext):
		pass

	def exitEgo_velocity_value(self, ctx: AVScenariosParser.Ego_velocity_valueContext):
		pass

	def enterTrace_speed(self, ctx: AVScenariosParser.Trace_speedContext):
		pass

	def exitTrace_speed(self, ctx: AVScenariosParser.Trace_speedContext):
		t = self._sema.find_trace(ctx.children[0].getText()
									 , (ctx.children[0].start.line
										, ctx.children[0].start.column))
		assert self._current._speed_constraint_assertion is not None
		t.add_speed_constraint_assertion(self._current._speed_constraint_assertion)
		self._sema.finish_assign_assertion_to_trace(t,self._current._speed_constraint_assertion)
		self._current._speed_constraint_assertion=None


	def enterSpeed_assert(self, ctx: AVScenariosParser.Speed_assertContext):
		pass

	def exitSpeed_assert(self, ctx: AVScenariosParser.Speed_assertContext):
		pass
	def enterSpeed_assert_right(self, ctx: AVScenariosParser.Speed_assert_rightContext):
		assert self._current._traffic_detection_assert is not None
		assert self._current._speed_limitation_checking is not None
		assert self._current._speed_violation is not None
		assert self._current._speed_constraint_assertion is None
		self._current._speed_constraint_assertion = SpeedConstraintAssertion()
		self._current._speed_constraint_assertion.set_speed_limitation_checking(self._current._speed_limitation_checking)
		self._current._speed_constraint_assertion.set_traffic_detection(self._current._traffic_detection_assert)
		self._current._speed_constraint_assertion.set_left_speed_violation(self._current._speed_violation)
		self._current._traffic_detection_assert=None
		self._current._speed_limitation_checking=None
		self._current._speed_violation =None
	def exitSpeed_assert_right(self, ctx: AVScenariosParser.Speed_assert_rightContext):
		assert self._current._speed_constraint_assertion is not None
		assert self._current._speed_violation is not None
		self._current._speed_constraint_assertion.set_right_speed_violation(self._current._speed_violation)
		self._current._speed_constraint_assertion.set_time_duration(float(ctx.children[5].getText()))
		self._current._speed_violation=None
	def enterSpeed_checking(self, ctx: AVScenariosParser.Speed_checkingContext):
		pass

	def exitSpeed_checking(self, ctx: AVScenariosParser.Speed_checkingContext):
		assert self._current._trace_time is not None
		assert self._current._speed_range is not None
		assert self._current._speed_limitation_checking is None
		self._current._speed_limitation_checking =SpeedLimitationChecking(self._current._trace_time)
		self._current._speed_limitation_checking.set_speed_range(self._current._speed_range)
		self._current._trace_time=None
		self._current._speed_range =None
	def enterSpeed_violation_stmt0(self, ctx: AVScenariosParser.Speed_violation_stmt0Context):
		pass

	def exitSpeed_violation_stmt0(self, ctx: AVScenariosParser.Speed_violation_stmt0Context):
		assert self._current._trace_time is not None
		assert self._current._speed is not None
		assert self._current._speed_violation is None
		self._current._speed_violation =SpeedViolation(self._current._trace_time,True)
		self._current._speed_violation.set_speed(self._current._speed)
		self._current._speed =None
		self._current._trace_time=None
	def enterSpeed_violation_stmt1(self, ctx: AVScenariosParser.Speed_violation_stmt1Context):
		pass

	def exitSpeed_violation_stmt1(self, ctx: AVScenariosParser.Speed_violation_stmt1Context):
		assert self._current._trace_time is not None
		assert self._current._speed is not None
		assert self._current._speed_violation is None
		self._current._speed_violation =SpeedViolation(self._current._trace_time,False)
		self._current._speed_violation.set_speed(self._current._speed)
		self._current._speed =None
		self._current._trace_time=None
	def enterTime_duration_value(self, ctx: AVScenariosParser.Time_duration_valueContext):
		pass

	def exitTime_duration_value(self, ctx: AVScenariosParser.Time_duration_valueContext):
		pass

	def enterAssign_trace(self, ctx: AVScenariosParser.Assign_traceContext):
		pass

	def exitAssign_trace(self, ctx: AVScenariosParser.Assign_traceContext):
		pass
	def enterAssign_assertion(self, ctx: AVScenariosParser.Assign_assertionContext):
		self._sema.handle_assertion(ctx.children[0].getText()
									,ctx.children[3].getText()
									,(ctx.children[0].start.line
									,ctx.children[0].start.column)
									,(ctx.children[3].start.line
									,ctx.children[3].start.column))
	def exitAssign_assertion(self, ctx: AVScenariosParser.Assign_assertionContext):
		pass
	def enterAssign_detection(self, ctx: AVScenariosParser.Assign_detectionContext):
		pass

	def exitAssign_detection(self, ctx: AVScenariosParser.Assign_detectionContext):
		pass

	def enterAssign_safety(self, ctx: AVScenariosParser.Assign_safetyContext):
		pass

	def exitAssign_safety(self, ctx: AVScenariosParser.Assign_safetyContext):
		pass

	def enterAssign_intersection_assert(self, ctx: AVScenariosParser.Assign_intersection_assertContext):
		pass

	def exitAssign_intersection_assert(self, ctx: AVScenariosParser.Assign_intersection_assertContext):
		pass

	def enterAssign_speed_constraint(self, ctx: AVScenariosParser.Assign_speed_constraintContext):
		pass
	def exitAssign_speed_constraint(self, ctx: AVScenariosParser.Assign_speed_constraintContext):
		pass
	def enterAssign_detection_id(self,ctx:AVScenariosParser.Assign_detection_idContext):
		assert self._current._detection_assertion is None
		self._current._detection_assertion=DetectionAssertion()
	def exitAssign_detection_id(self,ctx:AVScenariosParser.Assign_detection_idContext):
		self._sema.act_on_detection_assertion(ctx.children[0].getText())
		self._current._detection_assertion=None
	def enterAssign_safety_id(self, ctx: AVScenariosParser.Assign_safety_idContext):
		assert self._current._safety_assertion is None
		self._current._safety_assertion=SafetyAssertion()
	def exitAssign_safety_id(self, ctx: AVScenariosParser.Assign_safety_idContext):
		self._sema.act_on_safety_assertion(ctx.children[0].getText())
		self._current._safety_assertion=None
	def enterAssign_intersection_assert_id(self, ctx: AVScenariosParser.Assign_intersection_assert_idContext):
		pass
	def exitAssign_intersection_assert_id(self, ctx: AVScenariosParser.Assign_intersection_assert_idContext):
		self._sema.act_on_intersection_assertion(ctx.children[0].getText())
		self._current._intersection_assertion=None
	def enterAssign_speed_constraint_id(self, ctx: AVScenariosParser.Assign_speed_constraint_idContext):
		pass
	def exitAssign_speed_constraint_id(self, ctx: AVScenariosParser.Assign_speed_constraint_idContext):
		self._sema.act_on_speed_constraint_assertion(ctx.children[0].getText())
		self._current._speed_constraint_assertion=None
	def enterAssign_agent_ground(self, ctx: AVScenariosParser.Assign_agent_groundContext):
		pass

	def exitAssign_agent_ground(self, ctx: AVScenariosParser.Assign_agent_groundContext):
		self._sema.act_on_agent_ground_truth(ctx.children[0].getText())
		self._current._agent_ground_truth = None

	def enterAssign_agent_ground_dis(self, ctx: AVScenariosParser.Assign_agent_ground_disContext):
		pass

	def exitAssign_agent_ground_dis(self, ctx: AVScenariosParser.Assign_agent_ground_disContext):
		self._sema.act_on_agent_ground_distance(ctx.children[0].getText())
		self._current._agent_ground_distance = None

	def enterAssign_ego_state(self, ctx: AVScenariosParser.Assign_ego_stateContext):
		pass

	def exitAssign_ego_state(self, ctx: AVScenariosParser.Assign_ego_stateContext):
		self._sema.act_on_ego_state(ctx.children[0].getText())
		self._current._ego_state = None

	def enterAssign_agent_error(self, ctx: AVScenariosParser.Assign_agent_errorContext):
		pass

	def exitAssign_agent_error(self, ctx: AVScenariosParser.Assign_agent_errorContext):
		self._sema.act_on_agent_error(ctx.children[0].getText())
		self._current._agent_error = None

	def enterAssign_agent_state(self, ctx: AVScenariosParser.Assign_agent_stateContext):
		pass

	def exitAssign_agent_state(self, ctx: AVScenariosParser.Assign_agent_stateContext):
		self._sema.act_on_agent_state(ctx.children[0].getText())
		self._current._agent_state = None


# Global parsing function:parse a file into AST.
def Parse(file_name: AnyStr) -> AST:
	file = FileStream(file_name)
	lexer = AVScenariosLexer(file)
	tokens = CommonTokenStream(lexer)
	parser = AVScenariosParser(tokens)
	sema = Sema()
	listener = ASTListener(sema)
	walker = ParseTreeWalker()
	walker.walk(listener, parser.scenarios())
	return sema.get_ast()
