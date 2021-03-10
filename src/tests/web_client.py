import asyncio
import websockets
import json

async def hello():
    uri = "ws://localhost:8888"
    async with websockets.connect(uri) as websocket:
        data = { 'cmd' : 'stop', 'lang' : 'scenest', 'code' : 'xxxxxx'}
        await websocket.send(json.dumps(data))
        ret = await websocket.recv()
        print(ret)

asyncio.get_event_loop().run_until_complete(hello())
