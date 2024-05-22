import operator
import threading
import multiprocessing as mp
from ctypes import CDLL
import serial
import DobotDllType as dType
import asyncio
import os
import time
from meross_iot.controller.mixins.electricity import ElectricityMixin
from meross_iot.http_api import MerossHttpClient
from meross_iot.manager import MerossManager
import paho.mqtt.client as mqtt
import csv

row = {}
cube_number = 0
start_timestamp = 0
finish_timestamp = 0
power = 0
current = 0
voltage = 0
power_sample_timestamp = 0
detecting_colour = False
suction_cup = False
arm_moving = False
arm_velocity_ratio = 100
arm_acceleration_ratio = 100
belt_moving = False
belt_speed = 60
cube_weight = 32
block_height_Z = 15

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}


######################################################################
# DOBOT CONTROL MODULE
######################################################################
def move_belt(api):
    # Move the belt from the starting position to the pick position

    global total_time_belt, exec_belt, sleep
    start = time.perf_counter()
    global belt_speed
    # distance the belt moves along the y-axis
    distance = 75000
    STEP_PER_CRICLE = 360.0 / 1.8 * 10.0 * 16.0
    MM_PER_CRICLE = 3.1415926535898 * 36.0
    vel = float(belt_speed) * STEP_PER_CRICLE / MM_PER_CRICLE
    dType.SetEMotorSEx(api, 0, 1, int(vel), int(distance), 1)

    finish = time.perf_counter()
    exec_belt = finish - start
    sleep = exec_belt
    print(f'time to move belt{exec_belt}')


def ColorDetect():
    # This function reads serial messages from the JeVois Camera about the detected colour.
    # Each time a color is detected a counter is increased.
    # The first color that is detected for 40 times (empirically) it is returned as the most frequent detected color.

    # Serial device of JeVois Camera
    serdev = 'COM9'
    start = time.perf_counter()
    dict = {'Red': 0, 'Green': 0, 'Blue': 0}

    with serial.Serial(serdev, 115200, timeout=1) as ser:
        # wait for 100 ms for serial port
        time.sleep(0.1)
        serial.Serial.flush(self=ser)
        serial.Serial.flushInput(self=ser)
        serial.Serial.flushOutput(self=ser)
        print(ser)

        while 1:
            print("loop")
            # Read a whole line and strip any trailing line ending character:
            line = ser.readline()
            print(line)
            line = line.rstrip().decode()
            print(line)

            if line == 'Blue' or line == 'Green' or line == 'Red':
                # add one to the dictionary value that is associated with the color
                dict[line] += 1
                print(dict)

            if dict['Red'] == 40 or dict['Green'] == 40 or dict['Blue'] == 40:
                serial.Serial.close(self=ser)
                decision = max(dict.items(), key=operator.itemgetter(1))[0]
                all_values = dict.values()
                max_value = max(all_values)
                print(f'Colour detected: {decision}')
                print(f'Times detected: {max_value}')
                finish = time.perf_counter()
                total = finish - start
                print("total time: " + str(total))
                return decision

            finish = time.perf_counter()
            # time taken to return the color
            timer = round(finish - start, 2)

            # if no color is returned for 6 seconds return none
            if timer >= 6:
                serial.Serial.close(self=ser)
                print(f'Time elapsed in colour sensor: {timer} second(s)')
                print('No color detected...')
                return None


def dobotControlWithBelt():
    # This function is the Dobot Control Module
    # It is responsible to for the operation of the system
    # It collects information and send commands to all the components of the system

    mqttBroker = "localhost"
    client = mqtt.Client("dobotControlWithBelt")
    client.connect(mqttBroker)

    # Load Dll and get the CDLL object
    api = dType.load()

    # Connect Dobot
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:", CON_STR[state])

    if (state == dType.DobotConnect.DobotConnect_NoError):
        # Clean Command Queued
        dType.SetQueuedCmdClear(api)

        # Start samppling submodule (monitorPose)
        threading.Thread(target=monitorPose, args=(api, client)).start()

        # Async Motion Params Setting
        dType.SetHOMEParams(api, 250, 0, 0, 0, isQueued=1)

        # Parameters:
        # v        - Velocity of the Dobots movements
        # a        - Acceleration of the Dobots movements.
        # isQueued - To queue the command or not.
        global arm_velocity_ratio
        global arm_acceleration_ratio
        dType.SetPTPCommonParams(api, arm_velocity_ratio, arm_acceleration_ratio, isQueued=1)
        dType.SetPTPJointParams(api, arm_velocity_ratio, arm_acceleration_ratio, arm_velocity_ratio,
                                arm_acceleration_ratio, arm_velocity_ratio, arm_acceleration_ratio, arm_velocity_ratio,
                                arm_acceleration_ratio, 1)

        global block_height_Z

        pick_x = 275
        pick_y = 103
        pick_Z_up = block_height_Z + 50

        # Homing - reset position
        client.publish("dobot/operation", "homing")
        client.publish("dobot/operation2", "homing")
        dType.SetHOMECmd(api, temp=0, isQueued=1)
        dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode, pick_x, pick_y, pick_Z_up, 0, 1)

        color = "nothing"
        client.publish("dobot/color", color)

        # Go to pick up from homing
        client.publish("dobot/operation", "go_to_pick_up_from_homing")
        client.publish("dobot/operation2", "go_to_pick_up_from_homing")
        dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

        # Loop until no color is detected
        while color != None:
            client.publish("dobot/moving_arm", "false")
            client.publish("dobot/moving_belt", "true")
            # Move belt
            client.publish("dobot/operation", "moving_belt")
            move_belt(api)
            client.publish("dobot/moving_belt", "false")

            color = None
            client.publish("dobot/moving_arm", "true")
            client.publish("dobot/operation", "go_to_pick_down")
            # joint 3 +
            client.publish("dobot/operation2", "joint 3 +")
            dType.SetPTPCmdEx(api, 4, 20, 51, 22, -20, 1)

            # Enable suction cup
            client.publish("dobot/suction", "true")
            dType.SetEndEffectorSuctionCup(api, 1, 1)

            client.publish("dobot/operation", "go_to_pick_up_from_pick_down")
            # joint 3 -
            client.publish("dobot/operation2", "joint 3 -")
            dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJANGLEMode, 20, 51, 0, -20, 1)

            client.publish("dobot/operation", "go_to_color_detect_up_from_pick_up")
            # joint 1 +
            client.publish("dobot/operation2", "joint 1 +")
            dType.SetPTPCmdEx(api, 4, 65, 51, 0, -20, 1)

            client.publish("dobot/operation", "go_to_color_detect_down")
            # joint 3 +
            client.publish("dobot/operation2", "joint 3 +")
            dType.SetPTPCmdEx(api, 4, 65, 51, 48, -20, 1)
            # Disable suction cup
            client.publish("dobot/suction", "false")
            dType.SetEndEffectorSuctionCup(api, 0, 1)

            client.publish("dobot/operation", "go_to_color_detect_up_from_color_detect_down")

            # joint 3 -
            client.publish("dobot/operation2", "joint 3 -")
            dType.SetPTPCmdEx(api, 4, 65, 51, 0, -20, 1)
            client.publish("dobot/moving_arm", "false")

            client.publish("dobot/operation", "detecting_color")
            client.publish("dobot/operation2", "detecting_color")
            # Detect color
            color = ColorDetect()

            # COLOR DETECT FINISHED
            client.publish("dobot/moving_arm", "true")
            client.publish("dobot/operation", "go_to_color_detect_down")
            # joint 3 +
            client.publish("dobot/operation2", "joint 3 +")
            dType.SetPTPCmdEx(api, 4, 65, 51, 48, -20, 1)

            # Enable suction cup
            client.publish("dobot/suction", "true")
            dType.SetEndEffectorSuctionCup(api, 1, 1)

            client.publish("dobot/operation", "go_to_color_detect_up_from_color_detect_down")
            # joint 3 -
            client.publish("dobot/operation2", "joint 3 -")
            dType.SetPTPCmdEx(api, 4, 65, 51, 0, -20, 1)

            # Detected blue
            if color == 'Blue':
                client.publish("dobot/color", "blue")
                client.publish("dobot/operation", "detected_blue")
                print("COLOR BLUE SUCCESSFULLY DETECTED! :) ")

                client.publish("dobot/operation", "go_to_blue_up")
                # joint 1 +
                client.publish("dobot/operation2", "joint 1 +")
                dType.SetPTPCmdEx(api, 4, 85, 51, 0, -20, 1)

                # Disable suction cup
                client.publish("dobot/suction", "false")
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                client.publish("dobot/moving_arm", "true")
                client.publish("dobot/operation", "go_to_pick_up_from_blue")
                # joint 1 -
                client.publish("dobot/operation2", "joint 1 -")
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

            # Detected green
            elif color == 'Green':
                client.publish("dobot/color", "green")
                client.publish("dobot/operation", "detected_green")
                print("COLOR GREEN SUCCESSFULLY DETECTED! :) ")

                client.publish("dobot/operation", "go_to_green_up")
                # joint 1 +
                client.publish("dobot/operation2", "joint 1 +")
                dType.SetPTPCmdEx(api, 4, 105, 51, 0, -20, 1)

                # Disable suction cup
                client.publish("dobot/suction", "false")
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                client.publish("dobot/moving_arm", "true")
                client.publish("dobot/operation", "go_to_pick_up_from_green")
                # joint 1 -
                client.publish("dobot/operation2", "joint 1 -")
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

            # Detected red
            elif color == 'Red':
                client.publish("dobot/color", "red")
                client.publish("dobot/operation", "detected_red")
                print("COLOR RED SUCCESSFULLY DETECTED! :) ")

                client.publish("dobot/operation", "go_to_red_up")
                # joint 1 +
                client.publish("dobot/operation2", "joint 1 +")
                dType.SetPTPCmdEx(api, 4, 125, 51, 0, -20, 1)

                # Disable suction cup
                client.publish("dobot/suction", "false")
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                client.publish("dobot/moving_arm", "true")
                client.publish("dobot/operation", "go_to_pick_up_from_red")
                # joint 1 -
                client.publish("dobot/operation2", "joint 1 -")
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

            # No color detected
            elif color == None:
                client.publish("dobot/color", "none")
                client.publish("dobot/operation", "detected_none")

                print("NO COLOR DETECTED! :( ")

                # joint 1 -
                client.publish("dobot/operation2", "joint 1 -")
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

                # Disable suction cup
                client.publish("dobot/suction", "false")
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                client.publish("dobot/moving_arm", "false")
                dType.dSleep(500)

    # Disconnect Dobot
    dType.DisconnectDobot(api)


def dobotControlWithBeltWithoutSampling():
    # This function is the Dobot Control Module
    # It is responsible to for the operation of the system
    # It collects information and send commands to all the components of the system

    # Load Dll and get the CDLL object
    api = dType.load()

    # Connect Dobot
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:", CON_STR[state])

    if (state == dType.DobotConnect.DobotConnect_NoError):
        # Clean Command Queued
        dType.SetQueuedCmdClear(api)

        # Async Motion Params Setting
        dType.SetHOMEParams(api, 250, 0, 0, 0, isQueued=1)

        # Parameters:
        # v        - Velocity of the Dobots movements
        # a        - Acceleration of the Dobots movements.
        # isQueued - To queue the command or not.
        global arm_velocity_ratio
        global arm_acceleration_ratio
        dType.SetPTPCommonParams(api, arm_velocity_ratio, arm_acceleration_ratio, isQueued=1)
        dType.SetPTPJointParams(api, arm_velocity_ratio, arm_acceleration_ratio, arm_velocity_ratio,
                                arm_acceleration_ratio, arm_velocity_ratio, arm_acceleration_ratio, arm_velocity_ratio,
                                arm_acceleration_ratio, 1)

        global block_height_Z

        pick_x = 275
        pick_y = 103
        pick_Z_up = block_height_Z + 50

        # Homing - reset position
        dType.SetHOMECmd(api, temp=0, isQueued=1)
        dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode, pick_x, pick_y, pick_Z_up, 0, 1)

        color = "nothing"

        # Go to pick up from homing
        dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

        # Loop until no color is detected
        while color != None:
            # Move belt
            move_belt(api)

            color = None
            # joint 3 +
            dType.SetPTPCmdEx(api, 4, 20, 51, 22, -20, 1)

            # Enable suction cup
            dType.SetEndEffectorSuctionCup(api, 1, 1)

            # joint 3 -
            dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJANGLEMode, 20, 51, 0, -20, 1)

            # joint 1 +
            dType.SetPTPCmdEx(api, 4, 65, 51, 0, -20, 1)

            # joint 3 +
            dType.SetPTPCmdEx(api, 4, 65, 51, 48, -20, 1)
            # Disable suction cup
            dType.SetEndEffectorSuctionCup(api, 0, 1)

            # joint 3 -
            dType.SetPTPCmdEx(api, 4, 65, 51, 0, -20, 1)

            # Detect color
            color = ColorDetect()

            # COLOR DETECT FINISHED
            # joint 3 +
            dType.SetPTPCmdEx(api, 4, 65, 51, 48, -20, 1)

            # Enable suction cup
            dType.SetEndEffectorSuctionCup(api, 1, 1)

            # joint 3 -
            dType.SetPTPCmdEx(api, 4, 65, 51, 0, -20, 1)

            # Detected blue
            if color == 'Blue':
                print("COLOR BLUE SUCCESSFULLY DETECTED! :) ")

                # joint 1 +
                dType.SetPTPCmdEx(api, 4, 85, 51, 0, -20, 1)

                # Disable suction cup
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                # joint 1 -
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

            # Detected green
            elif color == 'Green':
                print("COLOR GREEN SUCCESSFULLY DETECTED! :) ")

                # joint 1 +
                dType.SetPTPCmdEx(api, 4, 105, 51, 0, -20, 1)

                # Disable suction cup
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                # joint 1 -
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

            # Detected red
            elif color == 'Red':
                print("COLOR RED SUCCESSFULLY DETECTED! :) ")

                # joint 1 +
                dType.SetPTPCmdEx(api, 4, 125, 51, 0, -20, 1)

                # Disable suction cup
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                # joint 1 -
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

            # No color detected
            elif color == None:

                print("NO COLOR DETECTED! :( ")

                # joint 1 -
                dType.SetPTPCmdEx(api, 4, 20, 51, 0, -20, 1)

                # Disable suction cup
                dType.SetEndEffectorSuctionCup(api, 0, 1)

                dType.dSleep(500)

    # Disconnect Dobot
    dType.DisconnectDobot(api)


######################################################################


######################################################################
# DOBOT SAMPLING SUBMODULE (POSE DATA)
######################################################################

def monitorPose(api: CDLL, client: mqtt.Client):
    # collect the pose of the robotic arm every 0.1sec and
    # publish it to the appropriate the MQTT topic

    while True:
        time.sleep(0.1)
        pose = dType.GetPose(api)
        # print("pose: " + str(pose))
        client.publish("dobot/pose_x", str(pose[0]))
        client.publish("dobot/pose_y", str(pose[1]))
        client.publish("dobot/pose_z", str(pose[2]))
        client.publish("dobot/pose_rHead", str(pose[3]))
        client.publish("dobot/pose_joint1Angle", str(pose[4]))
        client.publish("dobot/pose_joint2Angle", str(pose[5]))
        client.publish("dobot/pose_joint3Angle", str(pose[6]))
        client.publish("dobot/pose_joint4Angle", str(pose[7]))

######################################################################


######################################################################
# MEROSS SMART PLUG CONSUMPTION MEASUREMENTS
######################################################################

EMAIL = os.environ.get('xxxxxxxx@hotmail.com') or "xxxxxxxx@hotmail.com"
PASSWORD = os.environ.get('xxxxxxxx') or "xxxxxxxx"
BASE_URL = os.environ.get('iotx-eu.meross.com') or "iotx-eu.meross.com"


def startmerossConsumptionMeasurement2():
    if os.name == 'nt':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    loop = asyncio.get_event_loop()
    loop.run_until_complete(merossConsumptionMeasurement2())
    loop.stop()


async def merossConsumptionMeasurement2():
    # collect the power consumption measurement from the meross smart plug
    # every 0.5sec and publish it to the appropriate the MQTT topic

    mqttBroker = "localhost"
    client = mqtt.Client("merossConsumptionMeasurement2")
    client.connect(mqttBroker)

    # Setup the HTTP client API from user-password
    # When choosing the API_BASE_URL env var, choose from one of the above based on your location.
    # Asia-Pacific: "iotx-ap.meross.com"
    # Europe: "iotx-eu.meross.com"
    # US: "iotx-us.meross.com"
    http_api_client = await MerossHttpClient.async_from_user_password(api_base_url=BASE_URL, email=EMAIL,
                                                                      password=PASSWORD)

    # Setup and start the device manager
    manager = MerossManager(http_client=http_api_client)
    await manager.async_init()

    # Retrieve all the devices that implement the electricity mixin
    await manager.async_device_discovery()
    devs = manager.find_devices(device_class=ElectricityMixin)

    if len(devs) < 1:
        print("No electricity-capable device found...")
    else:
        for dev in devs:
            print("Device found: " + dev.name)
            if dev.name == "Thesis2":
                print("Device Selected to monitor: " + dev.name)

                # dev = devs[0]

                # Update device status: this is needed only the first time we connect with this device (or if the connection goes down)
                await dev.async_update()

                # Read the electricity power/voltage/current
                instant_consumption = (await dev.async_get_instant_metrics())
                print(f"Current consumption data: {instant_consumption}")

                for i in range(0, 10000):
                    # cahced_consumption = dev.get_last_sample()
                    # print(f"Cached consumption data: {cahced_consumption}")
                    # Read the electricity power/voltage/current
                    try:
                        instant_consumption = (await dev.async_get_instant_metrics())
                    except Exception:
                        print(
                            f"EXCEPTION!")

                    print(f"Current consumption data: {instant_consumption}, timestamp = {instant_consumption._sample_timestamp}")

                    global power
                    global current
                    global voltage
                    global power_sample_timestamp
                    power_sample_timestamp = instant_consumption.sample_timestamp
                    power = instant_consumption.power
                    current = instant_consumption.current
                    voltage = instant_consumption.voltage

                    client.publish("meross/sample_timestamp", str(instant_consumption.sample_timestamp))
                    client.publish("meross/power", str(instant_consumption.power))
                    client.publish("meross/current", str(instant_consumption.current))
                    client.publish("meross/voltage", str(instant_consumption.voltage))

                    print("TIMESTAMP: ", power_sample_timestamp, "POWER:", power, "(W) CURRENT:", current,
                          "(A) VOLTAGE: ", voltage, "(V)")

                    time.sleep(0.5)

    # Close the manager and logout from http_api
    manager.close()
    await http_api_client.async_logout()

######################################################################


######################################################################
# DATA COLLECTION MODULE (SUBSCRIBER)
######################################################################

def on_message(client, userdata, message):
    # print("SUBSCRIBER RECEIVED: " , "TOPIC: ",  str(message.topic), " PAYLOAD: ", str(message.payload.decode("utf-8")))

    row[message.topic] = message.payload.decode("utf-8")


def subscriber():
    # subscribe to all the appropriate MQTT topics for the data collection
    # this function is responsible to collect all the data sent to the MQTT
    # from the other modules and save them to a csv file

    global cube_weight
    global arm_velocity_ratio
    global arm_acceleration_ratio
    global belt_speed
    global row

    mqttBroker = "localhost"
    client = mqtt.Client("subscriber")
    client.connect(mqttBroker)

    client.loop_start()

    # subscribe to all topics sent from the other components
    client.subscribe("meross/#")
    client.subscribe("dobot/#")
    client.on_message = on_message

    # CSV file
    csvname = "joint_measurements_" + "_armv_" + str(arm_velocity_ratio) + "_arma_" + str(
        arm_acceleration_ratio) + "_belts_" + str(belt_speed) + "_cube_" + str(cube_weight) + "g" + ".csv"
    csvfile = open(csvname, 'w', newline='')
    fieldnames = ['cube/weight',
                  'arm/velocity_ratio', 'arm/acceleration_ratio',
                  'belt/speed',
                  'dobot/operation', 'dobot/color', 'dobot/moving_arm', 'dobot/moving_belt', 'dobot/operation2',
                  'dobot/suction',
                  'meross/sample_timestamp', 'meross/power', 'meross/current', 'meross/voltage',
                  'dobot/pose_x', 'dobot/pose_y', 'dobot/pose_z', 'dobot/pose_rHead', 'dobot/pose_joint1Angle',
                  'dobot/pose_joint2Angle', 'dobot/pose_joint3Angle', 'dobot/pose_joint4Angle']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    # constant parameters of the experiment
    row['cube/weight'] = cube_weight
    row['arm/velocity_ratio'] = arm_velocity_ratio
    row['arm/acceleration_ratio'] = arm_acceleration_ratio
    row['belt/speed'] = belt_speed

    # write the updated row to the csv every 1 second
    while True:
        time.sleep(1)
        print("row: " + str(row))
        writer.writerow(row)

    client.loop_stop()

######################################################################


if __name__ == "__main__":
    mp.freeze_support()
    p0 = mp.Process(target=subscriber)
    p1 = mp.Process(target=startmerossConsumptionMeasurement2)
    p2 = mp.Process(target=dobotControlWithBelt)

    p0.start()
    p1.start()
    p2.start()

    p0.join()
    p1.join()
    p2.join()
