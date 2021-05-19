import queue
import time
from flask import Flask, render_template, Response
import cv2
import flask
import numpy as np
import threading
import src.tools.global_var as glv
from src.tools.utils import stop_thread

#Initialize the Flask app
app = Flask(__name__)
flask_thread = None
STOP_FLAG = False
queue_list = []

def init():
    global queue_list
    glv.set("queue_front", queue.Queue())
    queue_list.append(glv.get("queue_front"))
    glv.set("queue_global", queue.Queue())
    queue_list.append(glv.get("queue_global"))

def run():
    global flask_thread
    global STOP_FLAG
    STOP_FLAG = False
    flask_thread = threading.Thread(target=app.run, kwargs={"debug":False, "use_reloader": False, "host":"0.0.0.0", "port":8096})
    print("Video server start")
    flask_thread.start()

def stop():
    global flask_thread
    global STOP_FLAG
    STOP_FLAG = True
    if flask_thread is not None:
        print("Video server stop")
        stop_thread(flask_thread)
        flask_thread = None

def clear_all_queues():
    global queue_list
    for q in queue_list:
        hard_clear_queue(q)

def hard_clear_queue(this_queue):
    while this_queue.qsize() > 3:
        this_queue.get()

def image_to_bgr(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    return array

def gen_frames(this_queue):
    global front_view_queue
    global STOP_FLAG
    while True:
        if STOP_FLAG:
            break
        # print("*"*this_queue.qsize())
        # print("Queue Size:{}".format(this_queue.qsize()))
        qsize = this_queue.qsize()
        if qsize < 3:
            # print("waiting")
            time.sleep(0.1)
            continue
        else:
            carla_image = this_queue.get()
            frame = image_to_bgr(carla_image)
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
            if qsize < 10:
                time.sleep(0.1) # 10fps
            elif qsize < 50:
                time.sleep(0.05) # 20 fps
            else:
                time.sleep(1/24) # 24fps

@app.route('/global')
def video_global():
    return Response(gen_frames(glv.get("queue_global")), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/front')
def video_front():
    return Response(gen_frames(glv.get("queue_front")), mimetype='multipart/x-mixed-replace; boundary=frame')