     
 #####################
#  main.py sensors  #
#####################

import os, glob, sys
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import json
from array import *

from smbus import SMBus
from datetime import datetime
import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
Relay1 = 17
Relay2 = 22
GPIO.setup(Relay1, GPIO.OUT)
GPIO.setup(Relay2, GPIO.OUT)

gpio_state = {17: False, 18: False, 22: False, 23: False, 24: False}
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
Broker = '192.168.1.148'

sub_topic_relay1 = "sensors/relay/cmd/R1"    # receive messages on this topic
sub_topic_relay2 = "sensors/relay/cmd/R2"    # receive messages on this topic
pub_topic_relay1 = "sensors/relay/state/R1"
pub_topic_relay2 = "sensors/relay/state/R2"

sub_topic_gpio = "sensors/gpio"   # receive messages on this topic
pub_topic_gpio = "sensors/gpio"   # send messages to this topic
pub_topic_int1 = "sensors/temps/int1"    # send messages to this topic
pub_topic_ext1 = "sensors/temps/ext1"    # send messages to this topic
pub_topic_ext2 = "sensors/temps/ext2"    # send messages to this topic
ds18b20 = 3 # number of temp sensors
base_dir = '/sys/bus/w1/devices/'
devices = glob.glob(base_dir + '28*')
device_file = '/w1_slave'
time.sleep(1)
INTERVAL=15 # Data capture and upload interval in seconds
next_reading = time.time()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, rc, *extra_params):
    print('Connected with result code ' + str(rc))
    # Subscribing to receive RPC requests
    client.subscribe(pub_topic_gpio + '/read/+')
    # client.subscribe(sub_topic_relay + '/read/+')
    client.subscribe("sensors/relay/cmd/R1")
    client.subscribe("sensors/relay/cmd/R2")
    # Sending current GPIO status
    client.publish(pub_topic_gpio, get_gpio_status(), 1)


# The callback for when a PUBLISH message is received from the server.

def on_message(client, userdata, msg):

   if str(msg.topic) == sub_topic_relay1:
     if (msg.payload.decode("utf-8")=='ON'):
           set_gpio_status(Relay1,False)
           print("R1 On")
     if (msg.payload.decode("utf-8")=='OFF'):
           set_gpio_status(Relay1,True)
           print("R1 Off")     
   client.publish(pub_topic_relay1,json.dumps(gpio_state[Relay1]))

   if str(msg.topic) == sub_topic_relay2:
     if (msg.payload.decode("utf-8")=='ON'):
           set_gpio_status(Relay2,False)
           print("R2 On")
     if (msg.payload.decode("utf-8")=='OFF'):
           set_gpio_status(Relay2,True)
           print("R2 Off")
   client.publish(pub_topic_relay2,json.dumps(gpio_state[Relay2]))


def get_gpio_status():
    # Encode GPIOs state to json
    return json.dumps(gpio_state)

def set_gpio_status(pin, status):
    # Output GPIOs state
    GPIO.output(pin, GPIO.HIGH if status else GPIO.LOW)
    # Update GPIOs state
    gpio_state[pin] = status


def read_all():
    i=0
    val=[99.9] * ds18b20
    for device in devices:
        device_dir = device + device_file
        raw_data = read_temp_file(device_dir)
        while raw_data[0].find('YES') == -1:
            sleep(0.1)
            raw_data = read_temp_file(device_dir)
        t_pos = raw_data[1].find('t=')
        if t_pos != -1:
            temp = float(raw_data[1][t_pos+2:]) / 1000
            #val[i] = temp #('%.1f' % round(temp,1))
     
             val[i] = round(temp,1)
            i = i + 1
    return val[0],val[1],val[2]

def read_temp_file(device_file):
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines


def on_publish(mosq, obj, mid):
    print("mid: " + str(mid))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(username='moorviewaddon',password='MarSarPop')
client.connect(Broker, 1883, 60)
client.loop_start()

try:
    while True:
        temp1, temp2, temp3 = read_all()
        client.publish(pub_topic_gpio,json.dumps(get_gpio_status())) # GPIO
        client.publish(pub_topic_ext2,json.dumps(temp3)) # Temp  channels
        client.publish(pub_topic_ext1,json.dumps(temp2)) # Temp  channels
        client.publish(pub_topic_int1,json.dumps(temp1)) # Temp  channels
        client.publish(pub_topic_relay1,json.dumps(gpio_state[Relay1]))
        client.publish(pub_topic_relay2,json.dumps(gpio_state[Relay2]))
        next_reading += INTERVAL
        sleep_time = next_reading-time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
except KeyboardInterrupt:
    pass

client.loop_stop()
client.disconnect()

