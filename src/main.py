import os
import sys
import asyncio
import websockets
import json
import threading
from os.path import abspath, join, dirname
sys.path.insert(0, abspath(dirname(dirname(dirname(__file__))))) # project root path
sys.path.insert(0, join( abspath(dirname(dirname(dirname(__file__)))), 'third-party/carla-0.9.10-py3.7-linux-x86_64.egg'))
from src.main.engine import Engine
# windows的bug，不能接受KeyboardInterrupt
# https://stackoverflow.com/questions/27480967/why-does-the-asyncios-event-loop-suppress-the-keyboardinterrupt-on-windows
import signal
import time
# signal.signal(signal.SIGINT, signal.SIG_DFL)

input_dir = "input_temp"
input_file_name = "input.sc"
input_file = os.path.join('input_temp', 'input.sc')

engine_listen_port = "8093"
engine_websocket = None

class EngineWebsocket:
    def __init__(self):
        self.engine = None #判断是否正在运行测试
        self.is_load_map = False  #存储当前提交运行，是否为加载地图
        self.is_brief = False
        self.engine_is_running = False
        self.websocket = None
        self.start_event = threading.Event()
        self.stop_event = threading.Event()

    def set_websocket(self, ws):
        self.websocket = ws

    def get_websocket(self):
        return self.websocket

    def is_engine_running(self):
        return self.is_engine_running
    
    def set_engine_running(self, is_running_stop):
        self.is_engine_running = is_running_stop

    def set_load_map(self, is_load_map):
        self.is_load_map = is_load_map

    def get_load_map(self):
        return self.is_load_map

    def set_engine(self, e):
        self.engine = e

    def get_engine(self):
        return self.engine

    def get_start_event(self):
        return self.start_event

    def get_stop_event(self):
        return self.stop_event

    def set_brief(self, is_brief):
        self.is_brief = is_brief

    async def send_msg(self, cmd=None, msg=None):
        info = {}
        # 若有cmd，则先相应地更新is_engine_running的取值
        if cmd:
            info['cmd'] = cmd
            if cmd == "ASSERT" or cmd == "STOP" or cmd == "RES" or cmd == "CRITERIA":
                self.set_engine_running(False)
            else:
                self.set_engine_running(True)
        # 若有msg，则向前端发送msg
        if msg:
            info['msg'] = msg
        # state字段取自self.is_engine_running。
        info['state'] = "isRunning" if self.is_engine_running is True else "notRunning"
        await self.websocket.send(json.dumps(info))

    def callback(self, cmd, msg=None):
        if (self.is_brief and cmd == "RES") or (not self.is_brief and cmd != "RES"):
            asyncio.run(self.send_msg(cmd, msg))

def _signal_handler(signum, frame):
    print("[Terminate] Destroy engine when receiving a signal interrupt]")
    engine = engine_websocket.get_engine()

    stop_event = engine_websocket.get_stop_event()
    if engine is not None:
        stop_event.set()
    print("[Terminate] Destroying...")
    time.sleep(5)
    print("[Terminate] Destroyed")
    sys.exit(0)

async def main(websocket, path):
        engine_websocket.set_websocket(websocket)
        # 发送状态信息给前端页面 isRunning/notRunning
        # await engine_websocket.send_msg()

        async for data in websocket:
            engine = engine_websocket.get_engine()

            start_event = engine_websocket.get_start_event()
            stop_event = engine_websocket.get_stop_event()

            msg = json.loads(data)
            cmd = None
            if "cmd" in msg:
                cmd = msg['cmd']
            if cmd == "run":
                print("start to run test")
                #判断前端的run命令是否为选择地图
                map_name = msg['map_name'] if 'map_name' in msg else None
                is_load_map = msg['is_load_map'] if 'is_load_map' in msg else False
                language = msg['lang'] if 'lang' in msg else "scenest"
                time_limit = msg['time'] if 'time' in msg else -1
                
                # 1. engine == None 通过
                # 2. engine 不为空，
                #   2.1 get_load_map == True -> 上次操作是切换地图，通过
                #   2.2 get_load_map == False -> 上次操作不是切换地图，正在模拟测试，不通过，不能重复run ，需要让用户先stop

                # 只有在没有运行
                #if engine is not None and not engine_websocket.get_load_map():
                #    print("already running, please stop first")
                #    msg = {'state' : "engine already running"}
                #    await websocket.send(json.dumps(msg))
                 #   return
                if not os.path.exists(input_dir):
                    os.makedirs(input_dir)
                scenest_file = open(input_file, 'w')
                scenest_file.write(msg['code'])
                scenest_file.close()

                engine_websocket.set_load_map(is_load_map)

                start_event.clear() # 重置状态
                stop_event.clear() 
                # 新线程中运行engine
                engine = Engine(input_file, engine_websocket.callback, map_name, language, time_limit, is_load_map, start_event, stop_event)
                engine.start()
                start_event.set() # 启动
                engine_websocket.set_engine(engine)
                engine_websocket.set_engine_running(True)
                # 发送状态信息给前端页面 isRunning/notRunning
                await engine_websocket.send_msg()
                signal.signal(signal.SIGINT, _signal_handler)
                print("engine started")
            elif cmd == "stop":
                print("stop test")
                if engine is not None:
                    stop_event.set()
                    #utils.stop_thread(engine)
                    #engine.stop()
                engine_websocket.set_engine(None)
                engine_websocket.set_load_map(False)
                engine_websocket.set_engine_running(False)
                os.remove(input_file)
                # 发送状态信息给前端页面 isRunning/notRunning
                await engine_websocket.send_msg()
            elif cmd == "query":
                # 发送状态信息给前端页面 isRunning/notRunning
                await engine_websocket.send_msg()
            elif cmd == "move":
                print("move test: {}".format(msg['code']))
                if engine is not None:
                    engine.change_view(msg['code'])
            elif cmd == "get_drv_decision":
                print("start to run test sciently")
                engine_websocket.set_brief(True)
                language = msg['lang'] if 'lang' in msg else "scenest"
                time_limit = msg['time'] if 'time' in msg else -1
                if not os.path.exists(input_dir):
                    os.makedirs(input_dir)
                scenest_file = open(input_file, 'w')
                scenest_file.write(msg['code'])
                scenest_file.close()

                start_event.clear() # 重置状态
                stop_event.clear() 
                # 新线程中运行engine
                engine = Engine(input_file, engine_websocket.callback, language=language, time_limit=time_limit, start_event=start_event, stop_event=stop_event)
                engine.start()
                start_event.set() # 启动
                engine_websocket.set_engine(engine)
                engine_websocket.set_engine_running(True)
                print("engine started")
            else:
                print("error")

if __name__ == "__main__":
    engine_websocket = EngineWebsocket()
    start_server = websockets.serve(main, "0.0.0.0", engine_listen_port)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
