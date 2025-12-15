"""
ECE 40862 Lab 5-6 - Motion Detector using MPU6050 and ESP32

Author: Si-Ci (Simon) Chou
"""

import time
import ujson
import socket
import network
import neopixel
from MPU6050 import MPU
from machine import Pin, I2C, Timer

X_THRESHOLD = 0.45
Y_THRESHOLD = 0.45
Z_THRESHOLD = 0.45

PIN_NEOPIXEL_PWR = 2
PIN_NEOPIXEL_DATA = 0
PIN_I2C_SCL = 14
PIN_I2C_SDA = 22

I2C_FREQ = 400000
POLLSTATUS_INTERVAL = 30000
MOTIONDETECT_INTERVAL = 2000

SSID = "test"
PASSWORD = "password"

POLLSTATUS_CHANNEL_ID = 3203294

POLLSTATUS_KEY = "A680K1DN950ROOQJ"
POLLSTATUS_SERVER = "api.thingspeak.com"
POLLSTATUS_PORT = 80

MOTIONDETECT_KEY = "bFjv7T3w5RAvhRDztjulsl"
MOTIONDETECT_EVENT = "motion_detect"
MOTIONDETECT_SERVER = "maker.ifttt.com"
MOTIONDETECT_PORT = 80

NPCOLOR = {'OFF': (0, 0, 0), 'RED': (255, 0, 0), 'GREEN': (0, 255, 0)}

armed = False
mpu_obj = None
np_obj = None

timer_pollstatus = Timer(0)
timer_motiondetect = Timer(1)

def wifi(ssid, pwd):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(ssid, pwd)
    while not wlan.isconnected():
        time.sleep_ms(1500)
    print(f"Connected to {ssid}")
    print(f"IP Address: {wlan.ifconfig()[0]}\n")     

def init_neopixel():
    global np_obj
    pwr = Pin(PIN_NEOPIXEL_PWR, Pin.OUT)
    pwr.value(1)

    data_pin = Pin(PIN_NEOPIXEL_DATA, Pin.OUT)
    np_obj = neopixel.NeoPixel(data_pin, 1)
    neopixel_set('OFF')

def neopixel_set(color):
    global np_obj
    np_obj[0] = NPCOLOR[color]
    np_obj.write()

def cb_pollstatus(args):
    global armed
    
    sock = socket.socket()
    sock.connect(socket.getaddrinfo(POLLSTATUS_SERVER, POLLSTATUS_PORT)[0][-1])
    
    request = (
        "GET /channels/" + str(POLLSTATUS_CHANNEL_ID) + "/feeds.json?api_key={}&results=1 HTTP/1.1\r\n"
        "Host: " + POLLSTATUS_SERVER + "\r\n"
        "Connection: close\r\n\r\n"
    ).format(POLLSTATUS_KEY)
    sock.send(request)
    
    res = b""
    while True:
        chunk = sock.recv(1024)
        if not chunk: 
            break
        res += chunk
    sock.close()
    # print(res)
    json_body = res[res.find(b'{') : res.rfind(b'}') + 1]
    # print(json_body)
    data = ujson.loads(json_body)
    
    status = data["feeds"][-1]["field1"]
    neopixel_set('GREEN' if status == "ACTIVATE" else 'OFF')
    armed = status == "ACTIVATE"

def cb_motiondetect(args):
    if not armed:
        return
    ax, ay, az = mpu_obj.acceleration()
    # print("cb_motiondetect", ax, ay, az)

    if abs(ax) > X_THRESHOLD or abs(ay) > Y_THRESHOLD or abs(az) > Z_THRESHOLD:
        neopixel_set('RED')
        
        sock = socket.socket()
        sock.connect(socket.getaddrinfo(MOTIONDETECT_SERVER, MOTIONDETECT_PORT)[0][-1])
        payload_data = ujson.dumps({
            "value1": str(ax),
            "value2": str(ay),
            "value3": str(az)
        })
        request = (
            "POST /trigger/{}/json/with/key/{} HTTP/1.1\r\n"
            "HOST: {}\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: {}\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{}"
        ).format(MOTIONDETECT_EVENT, MOTIONDETECT_KEY, MOTIONDETECT_SERVER, len(payload_data), payload_data)
        
        sock.send(request)
        sock.close()

if __name__ == "__main__":
    wifi(SSID, PASSWORD)
    init_neopixel()

    i2c = I2C(0, scl=Pin(PIN_I2C_SCL), sda=Pin(PIN_I2C_SDA), freq=I2C_FREQ)
    mpu_obj = MPU(i2c)
    mpu_obj.calibrate_acceleration()

    timer_pollstatus.init(mode=Timer.PERIODIC, period=POLLSTATUS_INTERVAL, callback=cb_pollstatus)
    timer_motiondetect.init(mode=Timer.PERIODIC, period=MOTIONDETECT_INTERVAL, callback=cb_motiondetect)

    cb_pollstatus(None)