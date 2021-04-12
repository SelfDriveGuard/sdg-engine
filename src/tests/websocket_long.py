# -*- coding:utf-8 -*-

import asyncio
import websockets
import json

# 向服务器端发送请求
async def send_msg(websocket):
    while True:
        _text = input("Please press Enter to start the test:")
        # 从test.txt中读测试脚本
        with open('test.txt', 'r', encoding='utf-8') as f:
            code = f.read()
            cont = {
                'cmd': "get_drv_decision",
                'code': code,
                'lang': "scenic",
                'time': 10
            }
            await websocket.send(json.dumps(cont, ensure_ascii=False))
            recv_text = await websocket.recv()
            print(f"{recv_text}")

# 客户端主逻辑
async def main_logic():
    # 以engine部署在172.16.111.153:8093为例
    async with websockets.connect('ws://172.16.111.153:8093') as websocket:
        await send_msg(websocket)

if __name__ == "__main__":
        asyncio.get_event_loop().run_until_complete(main_logic())