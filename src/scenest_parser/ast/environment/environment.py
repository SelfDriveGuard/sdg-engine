# The file Environment.py defines classes that describe environment
from src.scenest_parser.ast.base.state import Variable,NodeType,Node
from src.scenest_parser.ast.base.weathers import Weathers
from src.scenest_parser.ast.base.time import Time
from typing import Optional,AnyStr
class Environment(Variable,Node):
    def __init__(self,name:Optional[AnyStr]=None):
        Variable.__init__(self,name)
        Node.__init__(self,NodeType.T_ENV)
        self._time=None
        self._weathers=None
    def set_time(self,time:Time):
        self._time=time
    def set_weathers(self,weathers:Weathers):
        self._weathers=weathers
    def get_time(self)->Time:
        return self._time
    def get_weathers(self)->Weathers:
        return self._weathers