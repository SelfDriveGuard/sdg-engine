# The file PedestrianType.py defines class representing pedestrian type
from typing import NoReturn, Optional,AnyStr
from src.scenest_parser.ast.base.state import Variable,Node,NodeType
from src.scenest_parser.ast.base.vehicle_type import Color,RGBColor,ColorList
class Height(Variable,Node):
	def __init__(self,height:float,name:Optional[AnyStr]=None):
		Variable.__init__(self,name)
		Node.__init__(self,NodeType.T_HEIGHT)
		self._value=height
	def get_value(self)->float:
		return self._value
class PedestrianType(Variable,Node):
	def __init__(self,name:Optional[AnyStr]=None):
		Variable.__init__(self,name)
		Node.__init__(self,NodeType.T_PEDTYPE)
		self._height=None
		self._color=None
	def set_height(self,height:Height)->NoReturn:
		self._height=height
	def set_color(self,color:Color)->NoReturn:
		self._color=color
	def get_height(self)->Height:
		return self._height
	def get_color(self)->Color:
		return self._color
	def is_rgb_color(self)->bool:
		return isinstance(self._color,RGBColor)
	def is_color_list(self)->bool:
		return isinstance(self._color,ColorList)