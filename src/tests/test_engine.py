import time
import sys
import os
from os.path import abspath, join, dirname
sys.path.insert(0, abspath(dirname(dirname(dirname(__file__))))) # project root path
sys.path.insert(0, join( abspath(dirname(dirname(dirname(__file__)))), 'third-party/carla-0.9.10-py3.7-linux-x86_64.egg'))
from src.main.engine import Engine

# set env variable for local test
os.environ["CARLA_SERVER_IP"] = "127.0.0.1"
os.environ["ROS_BRIDGE_IP"] = "127.0.0.1"
assertion = []

def main():
    engine = Engine(join(abspath(dirname(dirname(__file__))), 'scenest_parser/inputs/input6(assertion).txt'), on_assertion_succeed)
    engine.run()
    print(assertion)
    # 若要尝试中断运行，则将以下代码取消注释
    # print("Sleep")
    # for i in range(8):
    #     print(8-i)
    #     time.sleep(1)
    # print("Send stop")
    # engine.stop()
def on_assertion_succeed(assertion_list):
    global assertion
    assertion = assertion_list

if __name__ == '__main__':
    main()