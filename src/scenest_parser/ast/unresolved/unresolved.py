# The file Unresolved.py defines class 
# that representing unresolved data types until we can figure it out.
from src.scenest_parser.ast.base.state import Node,NodeType
from typing import Union,AnyStr,NoReturn,Tuple
from src.scenest_parser.ast.base.motion import UniformMotion,WaypointMotion
class NameWithRealValue(Node):
	def __init__(self,value:float,name:AnyStr):
		super().__init__(NodeType.T_NRV)
		self._value:float=value
		self._name:AnyStr=name
	def get_value(self)->float:
		return self._value
	def get_name(self)->AnyStr:
		return self._name
	def __str__(self):
		return 'unresolved'
class NameWithString(Node):
	def __init__(self,value:AnyStr,name:AnyStr):
		super().__init__(NodeType.T_NS)
		self._value:AnyStr=value
		self._name=name
	def get_value(self)->AnyStr:
		return self._value
	def get_name(self)->AnyStr:
		return self._name
	def __str__(self):
		return 'unresolved'
class NameWithTwoRealValues(Node):
	def __init__(self,v1:float,v2:float,name:AnyStr) :
		Node.__init__(self,NodeType.T_NTRV)
		self._value:Tuple[float,float]=(v1,v2)
		self._name=name
	def get_value(self)->Tuple[float,float]:
		return self._value
	def get_name(self):
		return self._name
	def __str__(self):
		return 'unresolved'
class NameWithMotion(Node):
	def __init__(self,motion:Union[UniformMotion,WaypointMotion]):
		Node.__init__(self,NodeType.T_NMOTION)
		self._name=None
		self._motion=motion
	def is_uniform_motion(self)->bool:
		return isinstance(self._motion,UniformMotion)
	def is_waypoint_motion(self)->bool:
		return isinstance(self._motion,WaypointMotion)
	def get_motion(self)->Union[UniformMotion,WaypointMotion]:
		return self._motion
	def set_name(self,name:AnyStr)->NoReturn:
		self._name=name
	def get_name(self)->AnyStr:
		return self._name
	def __str__(self):
		return 'unresolved'