import serial
import binascii
import random
import queue
import numpy as np
import threading
from threading import Thread
from time import sleep
from struct import *
import time
from serial.tools.list_ports import comports
from queue import Queue
from operator import eq
import colorama
from colorama import Fore, Back, Style
from sklearn import neighbors, datasets
from codrone_edu.protocol import *
from codrone_edu.storage import *
from codrone_edu.receiver import *
from codrone_edu.system import *
from codrone_edu.crc import *
import PIL.Image
import PIL.ImageDraw
# from pynput import keyboard
import os
import signal
import pkg_resources


def convertByteArrayToString(dataArray):
    if dataArray == None:
        return ""

    string = ""

    if (isinstance(dataArray, bytes)) or (isinstance(dataArray, bytearray)) or (not isinstance(dataArray, list)):
        for data in dataArray:
            string += "{0:02X} ".format(data)

    return string


def temperature_convert(temp, conversion="F"):
    """
    Converts the given temperature to Fahrenheit or Celsius
    :param temp:current temperature
    :param conversion: (C) Celcius or (F) Fahrenheit
    :return: converted temperature
    """
    if conversion == "F":
        return round((temp * 9 / 5) + 32, 3)
    elif conversion == "C":
        return round((temp - 32) * 5 / 9, 3)
    else:
        print("Conversion must be (F) or (C).")


def get_image_data(image_file_name):
    """
    gets image data when given image file name
    :param image_file_name: the image file name
    :return: list of data resized to fit on controller screen
    """
    img = PIL.Image.open(image_file_name)
    controller_size = (127, 63)
    img = img.resize(controller_size)
    return list(img.getdata())


class Drone:

    # def __on_press(self, key):
    #     """
    #     checks for key press on keyboard
    #     :param key: the key that is pressed
    #     :return: False to stop listener if esc pressed
    #     """
    #     if key == keyboard.Key.esc:
    #         try:
    #             self.emergency_stop()
    #         finally:
    #             os.kill(os.getpid(), signal.SIGTERM)
    #             # stop listener
    #             return False

    # def __listen(self):
    #     """
    #     starts the keyboard listener
    #     :return: None
    #     """
    #     listener = keyboard.Listener(on_press=self.__on_press)
    #     listener.start()

    def __button_event(self, button):
        """
        Stores any button press event with a time stamp
        :param button:
        :return: N/A
        """
        self.button_data[0] = time.time() - self.timeStartProgram
        self.button_data[1] = button.button
        self.button_data[2] = button.event.name

    # BaseFunctions Start

    def __init__(self, flagCheckBackground=True, flagShowErrorMessage=False, flagShowLogMessage=False,
                 flagShowTransferData=False, flagShowReceiveData=False):

        self._serialport = None
        self._bufferQueue = Queue(4096)
        self._bufferHandler = bytearray()
        self._index = 0

        self._thread = None
        self._flagThreadRun = False

        self._receiver = Receiver()

        self._flagCheckBackground = flagCheckBackground

        self._flagShowErrorMessage = flagShowErrorMessage
        self._flagShowLogMessage = flagShowLogMessage
        self._flagShowTransferData = flagShowTransferData
        self._flagShowReceiveData = flagShowReceiveData

        self._eventHandler = EventHandler()
        self._storageHeader = StorageHeader()
        self._storage = Storage()
        self._storageCount = StorageCount()
        self._parser = Parser()

        self._devices = []  # Save a list of discovered devices when you autoconnect
        self._flagDiscover = False  # Indicate if the device is being scanned for autoconnect
        self._flagConnected = False  # Lets you know if you're connected to a device when you connect automatically

        self.timeStartProgram = time.time()  # Program Start Time Recording

        self.systemTimeMonitorData = 0
        self.monitorData = []

        self.altitude_data = [0, 0, 0, 0, 0]
        self.motion_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.state_data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.position_data = [0, 0, 0, 0]
        self.flow_data = [0, 0, 0]
        self.range_data = [0, 0, 0]
        self.joystick_data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.color_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.trim_data = [0, 0, 0, 0, 0]
        self.waypoint_data = []
        self.previous_land = [0, 0]
        self.button_data = [0, 0, 0]
        self.init_press = 0
        self.error_data = [0, 0, 0]

        # set the event handlers for the sensor requests
        self.setEventHandler(DataType.Altitude, self.update_altitude_data)
        self.setEventHandler(DataType.State, self.update_state_data)
        self.setEventHandler(DataType.Motion, self.update_motion_data)
        self.setEventHandler(DataType.Position, self.update_position_data)
        self.setEventHandler(DataType.RawFlow, self.update_flow_data)
        self.setEventHandler(DataType.Range, self.update_range_data)
        self.setEventHandler(DataType.Joystick, self.update_joystick_data)
        self.setEventHandler(DataType.CardColor, self.update_color_data)
        self.setEventHandler(DataType.Trim, self.update_trim_data)
        self.setEventHandler(DataType.Button, self.__button_event)
        self.setEventHandler(DataType.Error, self.update_error_data)

        # fill values for the lists
        self.get_altitude_data()
        self.get_range_data()
        self.get_position_data()
        self.get_flow_data()
        self.get_state_data()
        self.get_motion_data()
        self.get_trim_data()

        # Color classifier
        self.knn = neighbors.KNeighborsClassifier(n_neighbors=9)
        self.labels = []
        self.parent_dir = os.getcwd()

        # Threaded keyboard listener
        # self.__listen()

        for i in range(0, 36):
            self.monitorData.append(i)

        self._control = ControlQuad8()

        colorama.init()
        library_name = "codrone-edu"
        library = pkg_resources.get_distribution(library_name)
        print(f"{library_name} library version {library.version}")
        # this only works when using the release version of the library
        print("Please remove AA batteries while plugged into your computer."
              "\nFor more information go to: robolink.com/edu-controller-issue"
              "\n")

    def __del__(self):

        self.close()

    def get_sensor_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.Altitude)
        time.sleep(delay)
        self.sendRequest(DeviceType.Drone, DataType.Motion)
        time.sleep(delay)
        self.sendRequest(DeviceType.Drone, DataType.Position)
        time.sleep(delay)
        self.sendRequest(DeviceType.Drone, DataType.Range)
        time.sleep(delay)
        self.sendRequest(DeviceType.Drone, DataType.State)
        time.sleep(delay)

        sensor_data_list = []
        sensor_data_list = self.altitude_data + self.motion_data \
                           + self.position_data + self.range_data + self.state_data

        return sensor_data_list

    def update_error_data(self, drone_type):
        """
        ErrorFlagsForSensor -----------------------------------------------------------

        None_                                   = 0x00000000

        Motion_NoAnswer                         = 0x00000001    # Motion 센서 응답 없음
        Motion_WrongValue                       = 0x00000002    # Motion 센서 잘못된 값
        Motion_NotCalibrated                    = 0x00000004    # Gyro Bias 보정이 완료되지 않음
        Motion_Calibrating                      = 0x00000008    # Gyro Bias 보정 중

        Pressure_NoAnswer                       = 0x00000010    # 압력 센서 응답 없음
        Pressure_WrongValue                     = 0x00000020    # 압력 센서 잘못된 값

        RangeGround_NoAnswer                    = 0x00000100    # 바닥 거리 센서 응답 없음
        RangeGround_WrongValue                  = 0x00000200    # 바닥 거리 센서 잘못된 값

        Flow_NoAnswer                           = 0x00001000    # Flow 센서 응답 없음
        Flow_WrongValue                         = 0x00002000    # Flow 잘못된 값
        Flow_CannotRecognizeGroundImage         = 0x00004000    # 바닥 이미지를 인식할 수 없음


        ErrorFlagsForState -------------------------------------------------------------

        None_                                   = 0x00000000

        NotRegistered                           = 0x00000001    # 장치 등록이 안됨
        FlashReadLock_UnLocked                  = 0x00000002    # 플래시 메모리 읽기 Lock이 안 걸림
        BootloaderWriteLock_UnLocked            = 0x00000004    # 부트로더 영역 쓰기 Lock이 안 걸림
        LowBattery                              = 0x00000008    # Low Battery

        TakeoffFailure_CheckPropellerAndMotor   = 0x00000010    # 이륙 실패
        CheckPropellerVibration                 = 0x00000020    # 프로펠러 진동발생
        Attitude_NotStable                      = 0x00000040    # 자세가 많이 기울어져 있거나 뒤집어져 있을때

        CanNotFlip_LowBattery                   = 0x00000100    # 배터리가 30이하
        CanNotFlip_TooHeavy                     = 0x00000200    # 기체가 무거움

        :param drone_type:
        :return:
        """

        self.error_data[0] = time.time() - self.timeStartProgram
        self.error_data[1] = drone_type.errorFlagsForSensor
        self.error_data[2] = drone_type.errorFlagsForState

    def update_altitude_data(self, drone_type):
        """
        temperature	Float32	4 Byte	-	temp.(℃) Celsius
        pressure	Float32	4 Byte	-	pressure (Pascals)
        altitude	Float32	4 Byte	-	Converting pressure to elevation above sea level(meters)
        rangeHeight	Float32	4 Byte	-	Height value output from distance sensor (meters)
        """

        self.altitude_data[0] = time.time() - self.timeStartProgram
        self.altitude_data[1] = drone_type.temperature
        self.altitude_data[2] = drone_type.pressure
        self.altitude_data[3] = drone_type.altitude
        self.altitude_data[4] = drone_type.rangeHeight

    def get_altitude_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.Altitude)
        time.sleep(delay)
        return self.altitude_data

    def get_error_data(self, delay=0.01):
        """
        Will get and print the current error state
        :param delay:
        :return:
        """
        self.sendRequest(DeviceType.Drone, DataType.Error)
        time.sleep(delay)

        error_flag_now = self.error_data[1]
        if error_flag_now & ErrorFlagsForSensor.Motion_Calibrating.value:
            print("Motion_Calibrating")
        elif error_flag_now & ErrorFlagsForSensor.Motion_NoAnswer.value:
            print("Motion_NoAnswer")
        elif error_flag_now & ErrorFlagsForSensor.Motion_WrongValue.value:
            print("Motion_WrongValue")
        elif error_flag_now & ErrorFlagsForSensor.Motion_NotCalibrated.value:
            print("Motion_NotCalibrated")
        elif error_flag_now & ErrorFlagsForSensor.Pressure_NoAnswer.value:
            print("Pressure_NoAnswer")
        elif error_flag_now & ErrorFlagsForSensor.Pressure_WrongValue.value:
            print("Pressure_WrongValue")
        elif error_flag_now & ErrorFlagsForSensor.RangeGround_NoAnswer.value:
            print("RangeGround_NoAnswer")
        elif error_flag_now & ErrorFlagsForSensor.RangeGround_WrongValue.value:
            print("RangeGround_WrongValue")
        elif error_flag_now & ErrorFlagsForSensor.Flow_NoAnswer.value:
            print("Flow_NoAnswer")
        elif error_flag_now & ErrorFlagsForSensor.Flow_CannotRecognizeGroundImage.value:
            print("Flow_CannotRecognizeGroundImage")
        else:
            pass

        return self.error_data

    def get_pressure(self, unit="Pa"):
        '''
        requests from the drone the pressure in pascals
        parameters unit can be either
        Pascals
        Kilopascals
        milliBars
        inches per mercury
        torriceli
        atm
        :return: Returns pressure reading in set unit rounded by 2 decimal places
        '''
        pascals = self.get_altitude_data()[2]
        for i in range(3):
            if pascals == 0:  # check if the value is an error which would be 0
                pascals = self.get_altitude_data()[2]
            else:  # if not 0 then it is an ok value
                break

        if unit == "Pa":
            value = pascals
        elif unit == "KPa" or unit == "kPa" or unit == "kpa":
            kiloPascals = pascals / 1000
            value = kiloPascals
        elif unit == "mB":
            milliBars = pascals / 100
            value = milliBars
        elif unit == "inHg":
            inches_per_mercury = pascals / 3386.39
            value = inches_per_mercury
        elif unit == "torr":
            torr = pascals / 133.322
            value = torr
        elif unit == "atm":
            atm = pascals / 101325
            value = atm
        else:
            print("error: get_pressure() input must be either: Pa KPa mB inHg torr atm, will default to Pascals")
            value = pascals
        value = round(value, 2)
        return value

    def get_elevation(self, unit="m"):
        value = self.get_altitude_data()[3]

        if unit == "m":
            meters = round(value, 2)
            value = meters
        elif unit == "km":
            kilometers = value / 1000
            value = kilometers
        elif unit == "ft":
            feet = value * 3.28084
            value = feet
        elif unit == "mi":
            miles = value / 1609.34
            value = miles
        else:
            print("error: get_elevation() input must be either: m km ft mi")
            return round(value, 2)
        return value

    def set_initial_pressure(self):
        self.init_press = 0
        while self.init_press < 1:
            self.init_press = self.get_pressure()

    def height_from_pressure(self, b=0, m=9.34):
        """
        initial_pressure: the initial pressure taken at the "floor level"
        the drone must start as close to the "ground" as possible
        otherwise the height will be offset this method will
        have lots of variance as the barometer is not
        precise enough for millimeter accuracy.

        :param b: slope intercept in PASCALS
        :param m: slope in millimeters/pascals
        :return: the height in millimeters
        """
        current_pressure = self.get_pressure()
        height_pressure = (self.init_press - current_pressure + b) * m
        height_pressure = round(height_pressure, 2)
        return height_pressure

    def get_temperature(self, unit="C"):
        """
        In celsius by default
        unit == "C" or unit == "c" or unit == "Celsius"
        unit == "F" or unit == "f" or unit == "Fahrenheit"
        unit == "K" or unit == "k" or unit == "Kelvin"
        :return: the temperature in the desired units rounded 2 decimal places
        """
        data = self.get_altitude_data()[1]
        if unit == "C" or unit == "c" or unit == "Celsius":
            data = data
        elif unit == "F" or unit == "f" or unit == "Fahrenheit":
            data = 9 / 5 * data + 32
        elif unit == "K" or unit == "k" or unit == "Kelvin":
            data = data + 273.15
        else:
            data = data
        return round(data, 2)

    def get_drone_temp(self, unit="C"):
        """
        Deprecated function name.
        :param unit:
        :return:
        """
        self.get_temperature(unit=unit)

    def update_range_data(self, drone_type):
        '''
        Uses the time of flight sensor to detect the distance to an object
        Bottom range sensor will not update unless flying.
        Will display -1000 by default

        Front:  millimeters
        Bottom: millimeters
        '''
        self.range_data[0] = time.time() - self.timeStartProgram
        self.range_data[1] = drone_type.front
        self.range_data[2] = drone_type.bottom

    def get_range_data(self, delay=0.01):
        """
        Sends a request for updating the range data then the range data should update and the list will be returned
        does not check if the data has been updated just returns whatever is in the list
        [] Front millimeters
        bottom millimeters
        :param delay:
        :return:
        """
        self.sendRequest(DeviceType.Drone, DataType.Range)
        time.sleep(delay)
        return self.range_data

    def convert_meter(self, meter, conversion="cm"):
        """
        Converts meters to centimeters (cm), millimeters (mm), or inches (in).
        Will return meters if given (m)
        :param meter: The distance in meters
        :param conversion: Conversion handles cm, mm, in, or m
        :return: The distance in unit converted to
        """
        if conversion == "cm":
            return round(meter * 100, 3)

        elif conversion == "in":
            return round(meter * 39.37, 3)

        elif conversion == "mm":
            return round(meter * 1000, 3)

        elif conversion == "m":
            return round(meter, 3)

        else:
            return round(meter, 3)
            print("Second parameter must be (cm), (in), (mm), or (m)")

    def convert_millimeter(self, millimeter, conversion="cm"):
        """
            Converts millimeters to centimeters (cm), meters (m), or inches (in).
            Will return millimeters if given (mm)
            :param millimeter: The distance in millimeters
            :param conversion: Conversion handles cm, mm, in, or m
            :return: The distance in unit converted to
            """
        if conversion == "cm":
            return round(millimeter * 0.1, 3)

        elif conversion == "in":
            return round(millimeter * 0.03937, 3)

        elif conversion == "mm":
            return round(millimeter, 3)

        elif conversion == "m":
            return round(millimeter * 0.001, 3)

        else:
            return round(millimeter, 3)
            print("Second parameter must be (cm), (in), (mm), or (m)")

    def get_front_range(self, unit="cm"):
        """
        :param unit: the unit that the distance will be in "cm"
        :return:
        """
        return self.convert_millimeter(self.get_range_data()[1], unit)

    def get_bottom_range(self, unit="cm"):
        """
        :param unit: the unit that the distance will be in "cm"
        :return:
        """
        return self.convert_millimeter(self.get_range_data()[2], unit)

    def update_color_data(self, drone_type):
        '''
        Reads the current sent over Hue, Saturation, Value, Luminosity
        values from the color sensors.
        There are 2 color sensors one in the front and one in the rear.
        Both positioned at the bottom of the drone

        front sensor: H,S,V,L
        read sensor : H,S,V,L
        Color       : color1, color2
        Card        : color_card
        '''

        self.color_data[0] = time.time() - self.timeStartProgram
        self.color_data[1] = drone_type.hsvl[0][0]
        self.color_data[2] = drone_type.hsvl[0][1]
        self.color_data[3] = drone_type.hsvl[0][2]
        self.color_data[4] = drone_type.hsvl[0][3]
        self.color_data[5] = drone_type.hsvl[1][0]
        self.color_data[6] = drone_type.hsvl[1][1]
        self.color_data[7] = drone_type.hsvl[1][2]
        self.color_data[8] = drone_type.hsvl[1][3]
        self.color_data[9] = drone_type.color[0]
        self.color_data[10] = drone_type.color[1]
        self.color_data[11] = drone_type.card

    def get_color_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.CardColor)
        time.sleep(delay)
        return self.color_data

    def update_position_data(self, drone_type):
        """
        location of the drone
        x	Float32	4 Byte	-	X axis in meters
        y	Float32	4 Byte	-	Y axis in meters
        z	Float32	4 Byte	-	z axis in meters
        """
        self.position_data[0] = time.time() - self.timeStartProgram
        self.position_data[1] = drone_type.x
        self.position_data[2] = drone_type.y
        self.position_data[3] = drone_type.z

    def get_position_data(self, delay=0.01):
        """
        position_data[0] = time.time() - self.timeStartProgram
        position_data[1] = X axis in meters
        position_data[2] = Y axis in meters
        position_data[3] = z axis in meters
        :param delay:
        :return:
        """
        self.sendRequest(DeviceType.Drone, DataType.Position)
        time.sleep(delay)
        return self.position_data

    def get_pos_x(self, unit="cm"):
        """
        x position in centimeters
        :param unit: "cm", "in", "mm", or "m"
        :return: x position in chosen unit (centimeter by default).
        """
        return self.convert_meter(self.get_position_data()[1], unit)

    def get_pos_y(self, unit="cm"):
        """
        y position in centimeters
        :param unit: "cm", "in", "mm", or "m"
        :return: y position in chosen unit (centimeter by default).
        """
        return self.convert_meter(self.get_position_data()[2], unit)

    def get_pos_z(self, unit="cm"):
        """
        z position in centimeters
        :param unit: "cm", "in", "mm", or "m"
        :return: z position in chosen unit (centimeter by default).
        """
        return self.convert_meter(self.get_position_data()[3], unit)

    def get_height(self, unit="cm"):
        """
        height in centimeters
        :param unit: "cm", "in", "mm", or "m"
        :return: height in chosen unit (centimeter by default).
        """
        return self.convert_millimeter(self.get_range_data()[2], unit)

    def update_flow_data(self, drone_type):
        """
        Relative position value calculated by optical flow sensor
        x	Float32	4 Byte	-	X axis(m)
        y	Float32	4 Byte	-	Y axis(m)
        will be in meters
        """
        self.flow_data[0] = time.time() - self.timeStartProgram
        self.flow_data[1] = drone_type.x
        self.flow_data[2] = drone_type.y

    def get_flow_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.RawFlow)
        time.sleep(delay)
        return self.flow_data

    def get_flow_x(self, unit="cm"):
        """
        Relative position value calculated by optical flow sensor
        from the x direction (forward and reverse)
        in centimeters
        :param unit: "cm", "in", "mm", or "m"
        :return: distance in cm
        """
        return self.convert_meter(self.get_flow_data()[1], "cm")

    def get_flow_y(self):
        """
        Relative position value calculated by optical flow sensor
        from the y direction (left and right)
        in centimeters
        :param unit: "cm", "in", "mm", or "m"
        :return: distance in cm
        """
        return self.convert_meter(self.get_flow_data()[1], "cm")

    def update_state_data(self, drone_type):

        """
        variable name	    form	          size	    range	Explanation
        modeSystem	        ModeSystem	      1 Byte	-	    System operating mode
        modeFlight	        ModeFlight	      1 Byte	-	    Flight controller operating mode
        modeControlFlight	ModeControlFlight 1 Byte	-	    flight control mode
        modeMovement	    ModeMovement	  1 Byte	-	    Moving state
        headless	        Headless	      1 Byte	-	    Headless setting status
        sensorOrientation	SensorOrientation 1 Byte	-	    Sensor orientation
        battery	            UInt8	          1 Byte	0~100	Drone battery level
        """
        self.state_data[0] = time.time() - self.timeStartProgram
        self.state_data[1] = drone_type.modeSystem
        self.state_data[2] = drone_type.modeFlight
        self.state_data[3] = drone_type.modeControlFlight
        self.state_data[4] = drone_type.headless
        self.state_data[5] = drone_type.sensorOrientation
        self.state_data[6] = drone_type.battery
        self.state_data[7] = drone_type.modeMovement
        self.state_data[8] = drone_type.controlSpeed

    def get_state_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.State)
        time.sleep(delay)
        return self.state_data

    def get_battery(self):
        """
        Battery level a value from 0 - 100 %
        Based on the battery voltage a guess
        of the battery level is determined.
        :return:
        """
        return self.get_state_data()[6]

    def get_flight_state(self):
        return self.get_state_data()[2]

    def get_control_speed(self):
        """
        :return: control speed of drone
        """
        return self.get_state_data()[8]

    def speed_change(self, speed_level=1):
        """
        Controls the speed of the drone
        default is set to 1
        max is level 3 which makes the drone go very fast.
        :param speed_level: integer from 1 -3
        :return: N/A
        """
        if speed_level > 3:
            speed_level = 3
        elif speed_level < 1:
            speed_level = 1

        self.sendCommand(CommandType.ControlSpeed, int(speed_level))

    def update_motion_data(self, drone_type):

        """
            Variable    Name    Type        Size          Range Unit     Description
        [0] time_elapsed          float                                 seconds
        [1] accelX	    Int16	2 Byte	-1568 ~ 1568 (-156.8 ~ 156.8)	m/s2 x 10	 X
        [2] accelY	    Int16	2 Byte	-1568 ~ 1568 (-156.8 ~ 156.8)	m/s2 x 10	 Y
        [3] accelZ	    Int16	2 Byte	-1568 ~ 1568 (-156.8 ~ 156.8)	m/s2 x 10	 Z
        [4] gyroRoll	Int16	2 Byte	-2000 ~ 2000	degree/second Roll
        [5] gyroPitch	Int16	2 Byte	-2000 ~ 2000	degree/second Pitch
        [6] gyroYaw  	Int16	2 Byte	-2000 ~ 2000	degree/second Yaw
        [7] angleRoll	Int16	2 Byte	-180 ~ 180	degree Roll
        [8] anglePitch	Int16	2 Byte	-180 ~ 180	degree Pitch
        [9] angleYaw	Int16	2 Byte	-180 ~ 180	degree Yaw
        """
        self.motion_data[0] = time.time() - self.timeStartProgram
        self.motion_data[1] = drone_type.accelX
        self.motion_data[2] = drone_type.accelY
        self.motion_data[3] = drone_type.accelZ
        self.motion_data[4] = drone_type.gyroRoll
        self.motion_data[5] = drone_type.gyroPitch
        self.motion_data[6] = drone_type.gyroYaw
        self.motion_data[7] = drone_type.angleRoll
        self.motion_data[8] = drone_type.anglePitch
        self.motion_data[9] = drone_type.angleYaw

    def get_motion_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.Motion)
        time.sleep(delay)
        return self.motion_data

    def get_x_accel(self):
        return self.get_motion_data()[1]

    def get_y_accel(self):
        return self.get_motion_data()[2]

    def get_z_accel(self):
        return self.get_motion_data()[3]

    def get_x_gyro(self):
        return self.get_motion_data()[4]

    def get_angular_speed_x(self):
        return self.get_motion_data()[4]

    def get_y_gyro(self):
        return self.get_motion_data()[5]

    def get_angular_speed_y(self):
        return self.get_motion_data()[5]

    def get_z_gyro(self):
        return self.get_motion_data()[6]

    def get_angular_speed_z(self):
        return self.get_motion_data()[6]

    def get_x_angle(self):
        return self.get_motion_data()[7]

    def get_y_angle(self):
        return self.get_motion_data()[8]

    def get_z_angle(self):
        return self.get_motion_data()[9]

    def update_joystick_data(self, drone_type):
        """
        variable  form	              size	    Range	    Explanation
        x	      Int8	              1 Byte	-100~100	X-axis value
        y	      Int8	              1 Byte	-100~100	Y-axis value
        direction Joystick Direction  1 Byte	-	        joystick direction
        event	  JoystickEvent	      1 Byte	-	        Event
        """
        self.joystick_data[0] = time.time() - self.timeStartProgram
        self.joystick_data[1] = drone_type.left.x
        self.joystick_data[2] = drone_type.left.y
        self.joystick_data[3] = drone_type.left.direction.name
        self.joystick_data[4] = drone_type.left.event.name
        self.joystick_data[5] = drone_type.right.x
        self.joystick_data[6] = drone_type.right.y
        self.joystick_data[7] = drone_type.right.direction.name
        self.joystick_data[8] = drone_type.right.event.name

    def get_joystick_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.Joystick)
        time.sleep(delay)
        return self.joystick_data

    def get_left_joystick_y(self):
        """
        Getter for left joystick Y (vertical) value.
        Range: -100~100   0 is neutral.
        """
        return self.joystick_data[2]

    def get_left_joystick_x(self):
        """
        Getter for left joystick X (horizontal) value.
        Range: -100~100   0 is neutral.
        """
        return self.joystick_data[1]

    def get_right_joystick_y(self):
        """
        Getter for right joystick Y (vertical) value.
        Range: -100~100   0 is neutral.
        """
        return self.joystick_data[6]

    def get_right_joystick_x(self):
        """
        Getter for right joystick X (horizontal) value.
        Range: -100~100   0 is neutral.
        """
        return self.joystick_data[5]

    def get_button_data(self):
        return self.button_data

    def l1_pressed(self):
        """
        Returns true if L1 button is pressed or held
        """
        if self.button_data[1] == 1 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def l2_pressed(self):
        """
        Returns true if L2 button is pressed or held
        """
        if self.button_data[1] == 2 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def r1_pressed(self):
        """
        Returns true if R1 button is pressed or held
        """
        if self.button_data[1] == 4 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def r2_pressed(self):
        """
        Returns true if L1 button is pressed or held
        """
        if self.button_data[1] == 8 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def h_pressed(self):
        """
        Returns true if H button is pressed or held
        """
        if self.button_data[1] == 16 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def power_pressed(self):
        """
        Returns true if power button is pressed or held
        """
        if self.button_data[1] == 32 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def up_arrow_pressed(self):
        """
        Returns true if up arrow button is pressed or held
        """
        if self.button_data[1] == 64 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def left_arrow_pressed(self):
        """
        Returns true if left arrow button is pressed or held
        """
        if self.button_data[1] == 128 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def right_arrow_pressed(self):
        """
        Returns true if right arrow button is pressed or held
        """
        if self.button_data[1] == 256 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def down_arrow_pressed(self):
        """
        Returns true if down arrow button is pressed or held
        """
        if self.button_data[1] == 512 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def s_pressed(self):
        """
        Returns true if S button is pressed or held
        """
        if self.button_data[1] == 1024 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def p_pressed(self):
        """
        Returns true if P button is pressed or held
        """
        if self.button_data[1] == 2048 and (self.button_data[2] == 'Press' or self.button_data[2] == 'Down'):
            return True
        return False

    def update_trim_data(self, drone_type):
        """
        Updates and reads the trim values for
        roll, pitch, yaw, and throttle
        that are set on the drones internal memory.
        :param drone_type:
        :return: N/A
        """
        self.trim_data[0] = time.time() - self.timeStartProgram
        self.trim_data[1] = drone_type.roll
        self.trim_data[2] = drone_type.pitch
        self.trim_data[3] = drone_type.yaw
        self.trim_data[4] = drone_type.throttle

    def get_trim_data(self, delay=0.01):
        self.sendRequest(DeviceType.Drone, DataType.Trim)
        time.sleep(delay)
        return self.trim_data

    # def go(self, direction, duration=1, power=50):
    #     """
    #     Fly in a given direction for the given duration and power.
    #     :param direction: string which can be one of the following: FORWARD, BACKWARD, LEFT, RIGHT, UP, and DOWN
    #     :param duration: int duration of the flight motion in seconds. If undefined, defaults to 1 seconds.
    #     :param power: int the power at which the drone flies forward. Takes a value from 0 to 100. Defaults to 50 if not defined.
    #     """
    #     try:
    #         self.set_roll(0)
    #         self.set_pitch(0)
    #         self.set_throttle(0)
    #
    #         direction = direction.upper()
    #         if power > 100:
    #             power = 100
    #         elif power < 0:
    #             power = 0
    #
    #         if direction == "FORWARD":
    #             self.set_pitch(power)
    #             self.move(duration)
    #         elif direction == "BACKWARD":
    #             self.set_pitch(power * -1)
    #             self.move(duration)
    #         elif direction == "LEFT":
    #             self.set_roll(power)
    #             self.move(duration)
    #         elif direction == "RIGHT":
    #             self.set_roll(power * -1)
    #             self.move(duration)
    #         elif direction == "UP":
    #             self.set_throttle(power)
    #             self.move(duration)
    #         elif direction == "DOWN":
    #             self.set_throttle(power * -1)
    #             self.move(duration)
    #     except:
    #         print("Incorrect arguments. Please check you parameters.")
    #         return

    def go(self, roll, pitch, yaw, throttle, duration):
        """
                Sends roll, pitch, yaw, throttle values continuously to the drone for duration (seconds)

                :param roll: int from -100-100
                :param pitch: int from -100-100
                :param yaw: int from -100-100
                :param throttle: int from -100-100
                :param timeMs: int from -100-1000000 in milliseconds
                :return: sendControl()
                """
        self.sendControlWhile(roll, pitch, yaw, throttle, duration * 1000)

    def _receiving(self):
        while self._flagThreadRun:

            self._bufferQueue.put(self._serialport.read())

            # Automatic update of data when incoming data background check is enabled
            if self._flagCheckBackground:
                while self.check() != DataType.None_:
                    pass

            # sleep(0.001)

    def isOpen(self):
        if self._serialport is not None:
            return self._serialport.isOpen()
        else:
            return False

    def isConnected(self):
        if not self.isOpen():
            return False
        else:
            return self._flagConnected

    def open(self, portname=None):
        """
        Open a serial port to the controller on a baud rate of 57600
        Checks the VID vendor ID 1155 for the CoDrone EDU controller
        in order to verify the correct device.

        Sends a battery request in order to verify a
        connection to the drone and displays the battery level

        :param portname: usb port path
        :return: Boolean (True if successful connection and false if not)
        """
        cde_controller_vid = 1155

        try:
            ser = serial.Serial()  # open first serial port
            ser.close()
        except:
            print("Serial library not installed")
            self.close()
            exit()
            return False

        if portname is None:
            nodes = comports()
            size = len(nodes)

            for item in nodes:
                if item.vid == cde_controller_vid:
                    portname = item.device
                    print("Found CoDrone EDU controller. ", portname)
                    break
        try:

            print("Connecting to CoDrone EDU controller.")
            self._serialport = serial.Serial(
                port=portname,
                baudrate=57600)

            if self.isOpen():

                self._flagThreadRun = True
                self._thread = Thread(target=self._receiving, args=(), daemon=True)
                self._thread.start()
                self._printLog("Connected.({0})".format(portname))

            else:

                self._printError("Could not connect to device.")
                print("Serial port could not open. Check the microUSB cable and port. ")
                self.close()
                exit()
                return False

        # TODO: Fix this bare except
        except:
            self._printError("Could not connect to device.")
            print("Could not connect to CoDrone EDU controller.")
            self.close()
            exit()
            return False
        # check about 10 times
        for i in range(10):
            state = self.get_state_data()
            state_flight = state[2]
            if state_flight is ModeFlight.Ready:
                break
            else:
                time.sleep(0.1)

        if state_flight is ModeFlight.Ready:
            print("Connected to CoDrone EDU")
            battery = state[6]
            print("Battery =", battery, "%")
            # set the speed to medium level
            self.speed_change(speed_level=2)
            for i in range(10):
                # disable the previous YPRT commands
                self.sendControl(0, 0, 0, 0)
                time.sleep(0.1)
        else:
            print("Could not connect to CoDrone EDU.")
            print("Check that the controller and drone are on and paired.")
            print("Checkout this lesson on how to pair https://learn.robolink.com/lesson/0-2-power-and-charging/")
            # print("Exiting program")
            # self.close()
            # exit()

        return True

    def close(self):
        # log output
        if self.isOpen():
            self._printLog("Closing serial port.")

        self._printLog("Thread Flag False.")

        if self._flagThreadRun:
            self._flagThreadRun = False
            sleep(0.1)

        self._printLog("Thread Join.")

        if self._thread is not None:
            self._thread.join(timeout=1)

        self._printLog("Port Close.")

        if self.isOpen():
            self._serialport.close()
            sleep(0.2)

    def makeTransferDataArray(self, header, data):
        if (header is None) or (data is None):
            return None

        if not isinstance(header, Header):
            return None

        if isinstance(data, ISerializable):
            data = data.toArray()

        crc16 = CRC16.calc(header.toArray(), 0)
        crc16 = CRC16.calc(data, crc16)

        dataArray = bytearray()
        dataArray.extend((0x0A, 0x55))
        dataArray.extend(header.toArray())
        dataArray.extend(data)
        dataArray.extend(pack('H', crc16))

        return dataArray

    def transfer(self, header, data):
        if not self.isOpen():
            return

        dataArray = self.makeTransferDataArray(header, data)

        self._serialport.write(dataArray)

        # send data output
        self._printTransferData(dataArray)

        return dataArray

    def check(self):
        while not self._bufferQueue.empty():
            dataArray = self._bufferQueue.get_nowait()
            self._bufferQueue.task_done()

            if (dataArray is not None) and (len(dataArray) > 0):
                # receive data output
                self._printReceiveData(dataArray)

                self._bufferHandler.extend(dataArray)

        while len(self._bufferHandler) > 0:
            stateLoading = self._receiver.call(self._bufferHandler.pop(0))

            # error output
            if stateLoading == StateLoading.Failure:
                # Incoming data output (skipped)
                self._printReceiveDataEnd()

                # Error message output
                self._printError(self._receiver.message)

            # log output
            if stateLoading == StateLoading.Loaded:
                # Incoming data output (skipped)
                self._printReceiveDataEnd()

                # Log output
                self._printLog(self._receiver.message)

            if self._receiver.state == StateLoading.Loaded:
                self._handler(self._receiver.header, self._receiver.data)
                return self._receiver.header.dataType

        return DataType.None_

    def checkDetail(self):
        while not self._bufferQueue.empty():
            dataArray = self._bufferQueue.get_nowait()
            self._bufferQueue.task_done()

            if (dataArray is not None) and (len(dataArray) > 0):
                # Receive data output
                self._printReceiveData(dataArray)

                self._bufferHandler.extend(dataArray)

        while len(self._bufferHandler) > 0:
            stateLoading = self._receiver.call(self._bufferHandler.pop(0))

            # Error output
            if stateLoading == StateLoading.Failure:
                # Incoming data output (skipped)
                self._printReceiveDataEnd()

                # Error message output
                self._printError(self._receiver.message)

            # Log output
            if stateLoading == StateLoading.Loaded:
                # Incoming data output (skipped)
                self._printReceiveDataEnd()

                # Log output
                self._printLog(self._receiver.message)

            if self._receiver.state == StateLoading.Loaded:
                self._handler(self._receiver.header, self._receiver.data)
                return self._receiver.header, self._receiver.data

        return None, None

    def _handler(self, header, dataArray):

        # Save incoming data
        self._runHandler(header, dataArray)

        # Run a callback event
        self._runEventHandler(header.dataType)

        # Monitor data processing
        self._runHandlerForMonitor(header, dataArray)

        # Verify data processing complete
        self._receiver.checked()

        return header.dataType

    def _runHandler(self, header, dataArray):

        # General data processing
        if self._parser.d[header.dataType] is not None:
            self._storageHeader.d[header.dataType] = header
            self._storage.d[header.dataType] = self._parser.d[header.dataType](dataArray)
            self._storageCount.d[header.dataType] += 1

    def _runEventHandler(self, dataType):
        if (isinstance(dataType, DataType)) and (self._eventHandler.d[dataType] is not None) and (
                self._storage.d[dataType] is not None):
            return self._eventHandler.d[dataType](self._storage.d[dataType])
        else:
            return None

    def _runHandlerForMonitor(self, header, dataArray):

        # Monitor data processing
        # Parse the received data self.monitorData[] Putting data in an array
        if header.dataType == DataType.Monitor:

            monitorHeaderType = MonitorHeaderType(dataArray[0])

            if monitorHeaderType == MonitorHeaderType.Monitor0:

                monitor0 = Monitor0.parse(dataArray[1:1 + Monitor0.getSize()])

                if monitor0.monitorDataType == MonitorDataType.F32:

                    dataCount = (dataArray.len() - 1 - Monitor0.getSize()) / 4

                    for i in range(0, dataCount):

                        if monitor0.index + i < len(self.monitorData):
                            index = 1 + Monitor0.getSize() + (i * 4)
                            self.monitorData[monitor0.index + i], = unpack('<f', dataArray[index:index + 4])

            elif monitorHeaderType == MonitorHeaderType.Monitor4:

                monitor4 = Monitor4.parse(dataArray[1:1 + Monitor4.getSize()])

                if monitor4.monitorDataType == MonitorDataType.F32:

                    self.systemTimeMonitorData = monitor4.systemTime

                    dataCount = (dataArray.len() - 1 - Monitor4.getSize()) / 4

                    for i in range(0, dataCount):

                        if monitor4.index + i < len(self.monitorData):
                            index = 1 + Monitor4.getSize() + (i * 4)
                            self.monitorData[monitor4.index + i], = unpack('<f', dataArray[index:index + 4])

            elif monitorHeaderType == MonitorHeaderType.Monitor8:

                monitor8 = Monitor8.parse(dataArray[1:1 + Monitor8.getSize()])

                if monitor8.monitorDataType == MonitorDataType.F32:

                    self.systemTimeMonitorData = monitor8.systemTime

                    dataCount = (dataArray.len() - 1 - Monitor8.getSize()) / 4

                    for i in range(0, dataCount):

                        if monitor8.index + i < len(self.monitorData):
                            index = 1 + Monitor8.getSize() + (i * 4)
                            self.monitorData[monitor8.index + i], = unpack('<f', dataArray[index:index + 4])

    def setEventHandler(self, dataType, eventHandler):

        if not isinstance(dataType, DataType):
            return

        self._eventHandler.d[dataType] = eventHandler

    def getHeader(self, dataType):

        if not isinstance(dataType, DataType):
            return None

        return self._storageHeader.d[dataType]

    def getData(self, dataType):

        if not isinstance(dataType, DataType):
            return None

        return self._storage.d[dataType]

    def getCount(self, dataType):

        if not isinstance(dataType, DataType):
            return None

        return self._storageCount.d[dataType]

    def _printLog(self, message):

        # Log output
        if self._flagShowLogMessage and message is not None:
            print(Fore.GREEN + "[{0:10.03f}] {1}".format((time.time() - self.timeStartProgram),
                                                         message) + Style.RESET_ALL)

    def _printError(self, message):

        # Error message output
        if self._flagShowErrorMessage and message is not None:
            print(
                Fore.RED + "[{0:10.03f}] {1}".format((time.time() - self.timeStartProgram), message) + Style.RESET_ALL)

    def _printTransferData(self, dataArray):

        # Send data output
        if self._flagShowTransferData and (dataArray is not None) and (len(dataArray) > 0):
            print(Back.YELLOW + Fore.BLACK + convertByteArrayToString(dataArray) + Style.RESET_ALL)

    def _printReceiveData(self, dataArray):

        # Receive data output
        if self._flagShowReceiveData and (dataArray is not None) and (len(dataArray) > 0):
            print(Back.CYAN + Fore.BLACK + convertByteArrayToString(dataArray) + Style.RESET_ALL, end='')

    def _printReceiveDataEnd(self):

        # Incoming data output (skipped)
        if self._flagShowReceiveData:
            print("")

    # BaseFunctions End

    # Common Start

    def sendPing(self, deviceType):

        if not isinstance(deviceType, DeviceType):
            return None

        header = Header()

        header.dataType = DataType.Ping
        header.length = Ping.getSize()
        header.from_ = DeviceType.Base
        header.to_ = deviceType

        data = Ping()

        data.systemTime = 0

        return self.transfer(header, data)

    def sendRequest(self, deviceType, dataType):

        if (not isinstance(deviceType, DeviceType)) or (not isinstance(dataType, DataType)):
            return None

        header = Header()

        header.dataType = DataType.Request
        header.length = Request.getSize()
        header.from_ = DeviceType.Base
        header.to_ = deviceType

        data = Request()

        data.dataType = dataType

        return self.transfer(header, data)

    def sendPairing(self, deviceType, address0, address1, address2, scramble, channel0, channel1, channel2, channel3):

        if ((not isinstance(deviceType, DeviceType)) or
                (not isinstance(address0, int)) or
                (not isinstance(address1, int)) or
                (not isinstance(address2, int)) or
                (not isinstance(scramble, int)) or
                (not isinstance(channel0, int)) or
                (not isinstance(channel1, int)) or
                (not isinstance(channel2, int)) or
                (not isinstance(channel3, int))):
            return None

        header = Header()

        header.dataType = DataType.Pairing
        header.length = Pairing.getSize()
        header.from_ = DeviceType.Base
        header.to_ = deviceType

        data = Pairing()

        data.address0 = address0
        data.address1 = address1
        data.address2 = address2
        data.scramble = scramble
        data.channel0 = channel0
        data.channel1 = channel1
        data.channel2 = channel2
        data.channel3 = channel3

        return self.transfer(header, data)

    # Common Start

    # Control Start
    def pair(self, portname=None):
        self.open(portname)

    def disconnect(self):
        self.close()

    def takeoff(self):

        self.reset_move()
        self.sendTakeOff()

        timeout = 4
        init_time = time.time()
        time_elapsed = time.time() - init_time
        while time_elapsed < timeout:
            time_elapsed = time.time() - init_time
            state = self.get_state_data()
            state_flight = state[2]
            if state_flight is ModeFlight.TakeOff:
                break
            else:
                self.sendTakeOff()
                time.sleep(0.01)

        time.sleep(4)

    def land(self):
        """
        Sends a command to land the drone gently.

        :return: None
        """
        self.reset_move()
        # reset the land coordinate back to zero
        self.previous_land[0] = self.previous_land[0] + self.get_position_data()[1]
        self.previous_land[1] = self.previous_land[1] + self.get_position_data()[2]
        self.sendLanding()

        timeout = 4
        init_time = time.time()
        time_elapsed = time.time() - init_time
        while time_elapsed < timeout:
            time_elapsed = time.time() - init_time
            state = self.get_state_data()
            state_flight = state[2]
            if state_flight is ModeFlight.Landing:
                break
            else:
                self.sendLanding()
                time.sleep(0.01)

        time.sleep(4)

    def emergency_stop(self):
        """
        Sends a command to stop all motors immediately.

        :return: None
        """
        self.reset_move()
        self.sendStop()

    def stop_motors(self):
        """
        same as emergency stop just different name
        :return:
        """
        self.emergency_stop()

    def hover(self, duration=0.01):
        """
        Hovers the drone in place for a duration of time.

        :param duration: number of seconds to perform the hover command
        TODO: Make this command use the sensors to attempt to stay at that position
        :return: None
        """
        self.sendControl(0, 0, 0, 0)
        time.sleep(duration)

    # Movement control

    def reset_move(self, attempts=3):
        """
        Resets the values of roll, pitch, yaw, and throttle to 0.

        :param attempts: number of times hover() command is sent
        :return: None
        """

        # make sure the drone doesnt have any previous YPRT
        for i in range(attempts):
            self.hover()

    def set_roll(self, power):
        """
        Sets the roll variable for flight movement.

        :param power: int from -100-100
        :return: None
        """
        self._control.roll = int(power)

    def set_pitch(self, power):
        """
        Sets the pitch variable for flight movement.

        :param power: int from 100-100
        :return: None
        """
        self._control.pitch = int(power)

    def set_yaw(self, power):
        """
        Sets the yaw variable for flight movement.

        :param power: int from -100-100
        :return: None
        """
        self._control.yaw = int(power)

    def set_throttle(self, power):
        """
        Sets the yaw variable for flight movement.

        :param power: int from -100-100
        :return: None
        """
        self._control.throttle = int(power)

    def move(self, duration=None):
        """
        Used with set_roll, set_pitch, set_yaw, set_throttle commands.
        Sends flight movement values to the drone.

        :param duration: Number of seconds to perform the action
        :return: None
        """
        if duration is None:
            self.sendControl(*vars(self._control).values())
            time.sleep(0.003)

        else:
            milliseconds = int(duration * 1000)
            # there is a while loop inside of the send control method.
            self.sendControlWhile(*vars(self._control).values(), milliseconds)

    def print_move_values(self):
        """
        Prints current values of roll, pitch, yaw, and throttle.

        :return: None
        """
        print(*vars(self._control).values())

    def turn(self, seconds=None, power=50):
        """
        Turns the drone in the yaw direction
        :param seconds: Number of seconds to perform the turn
        :param power: int from -100 - 100
        :return: None
        """
        if seconds is None:
            self.sendControlWhile(0, 0, -power, 0, 0.2)
            self.hover(0.5)
        else:
            seconds = seconds * 1000
            self.sendControlWhile(0, 0, -power, 0, seconds)
            self.hover(0.5)

    def set_waypoint(self):
        """Saves current position data to waypoint_data list
        :return: current position data
        """
        self.get_position_data()
        x = self.get_position_data()[1]
        y = self.get_position_data()[2]
        z = self.get_position_data()[3]
        temp = [x, y, z]
        return self.waypoint_data.append(temp)

    def sendTakeOff(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.TakeOff.value

        return self.transfer(header, data)

    def sendLanding(self):

        self._control.roll = 0
        self._control.pitch = 0
        self._control.yaw = 0
        self._control.throttle = 0
        self.move()

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.Landing.value

        return self.transfer(header, data)

    def sendStop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.Stop
        data.option = 0

        return self.transfer(header, data)

    def sendControl(self, roll, pitch, yaw, throttle):
        '''
        Sends roll, pitch, yaw, throttle values to the drone.

        :param roll: int from -100-100
        :param pitch: int from -100-100
        :param yaw: int from -100-100
        :param throttle: int from -100-100
        :return: transfer()
        '''

        if ((not isinstance(roll, int)) or (not isinstance(pitch, int)) or (not isinstance(yaw, int)) or (
                not isinstance(throttle, int))):
            return None

        header = Header()

        header.dataType = DataType.Control
        header.length = ControlQuad8.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        self._control.roll = roll
        self._control.pitch = pitch
        self._control.yaw = yaw
        self._control.throttle = throttle

        return self.transfer(header, self._control)

    def sendControlWhile(self, roll, pitch, yaw, throttle, timeMs):
        """
        Sends roll, pitch, yaw, throttle values continuously to the drone for timeMs

        :param roll: int from -100-100
        :param pitch: int from -100-100
        :param yaw: int from -100-100
        :param throttle: int from -100-100
        :param timeMs: int from -100-1000000 in milliseconds

        """

        if ((not isinstance(roll, int)) or
                (not isinstance(pitch, int)) or
                (not isinstance(yaw, int)) or
                (not isinstance(throttle, int))):
            return None

        time_sec = timeMs / 1000
        time_start = time.perf_counter()

        while (time.perf_counter() - time_start) < time_sec:
            self.sendControl(roll, pitch, yaw, throttle)
            time.sleep(0.003)

    def sendControlPosition16(self, positionX, positionY, positionZ, velocity, heading, rotationalVelocity):
        """
        drone movement command

        Instead of using 2 byte integers for all variables,
         we apply x10 to the values of position and velocity.

        :param positionX: Int16	-100 ~ 100(-10.0 ~ 10.0)	meter x 10	Front (+), Back (-)
        :param positionY: Int16	-100 ~ 100(-10.0 ~ 10.0)	meter x 10	Left(+), Right(-)
        :param positionZ: Int16	-100 ~ 100(-10.0 ~ 10.0)	meter x 10	Up (+), Down (-)
        :param velocity: Int16	5 ~ 20(0.5 ~ 2.0)	m/s x 10	position movement speed
        :param heading: Int16	-360 ~ 360	degree	Turn left (+), turn right (-)
        :param rotationalVelocity: Int16	10 ~ 360	degree/s	left and right rotation speed
        :return:
        """

        if ((not isinstance(positionX, int)) or
                (not isinstance(positionY, int)) or
                (not isinstance(positionZ, int)) or
                (not isinstance(velocity, int))):
            return None

        if ((not isinstance(heading, int)) or
                (not isinstance(rotationalVelocity, int))):
            return None

        header = Header()

        header.dataType = DataType.Control
        header.length = ControlPosition16.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = ControlPosition16()

        data.positionX = positionX
        data.positionY = positionY
        data.positionZ = positionZ
        data.velocity = velocity
        data.heading = heading
        data.rotationalVelocity = rotationalVelocity

        return self.transfer(header, data)

    def sendControlPosition(self, positionX, positionY, positionZ, velocity, heading, rotationalVelocity):
        """
        drone movement command

        Use real values for position and velocity, and
        integer values for heading and rotational Velocity.
        :param positionX: float	-10.0 ~ 10.0	meter	Front (+), Back (-)
        :param positionY: float	-10.0 ~ 10.0	meter	Left(+), Right(-)
        :param positionZ: float	-10.0 ~ 10.0	meter	Up (+), Down (-)
        :param velocity: float	0.5 ~ 2.0	m/s	position movement speed
        :param heading: Int16	-360 ~ 360	degree	Turn left (+), turn right (-)
        :param rotationalVelocity:
        :return: Int16	10 ~ 360	degree/s	left and right rotation speed
        """

        if not (isinstance(positionX, float) or isinstance(positionX, int)):
            return None

        if not (isinstance(positionY, float) or isinstance(positionY, int)):
            return None

        if not (isinstance(positionZ, float) or isinstance(positionZ, int)):
            return None

        if not (isinstance(velocity, float) or isinstance(velocity, int)):
            return None

        if (not isinstance(heading, int)) or (not isinstance(rotationalVelocity, int)):
            return None

        header = Header()

        header.dataType = DataType.Control
        header.length = ControlPosition.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = ControlPosition()

        data.positionX = float(positionX)
        data.positionY = float(positionY)
        data.positionZ = float(positionZ)
        data.velocity = float(velocity)
        data.heading = heading
        data.rotationalVelocity = rotationalVelocity

        return self.transfer(header, data)

    def move_forward(self, distance, units="cm", speed=1.0):
        """
        :param distance: the numerical value of the value to move
        :param units: can either be in inches, centimeters, meters, feet.
        :param speed: default 1 meter per second. Max is 2 meters/second.
        :return: N/A
        """
        if units == "cm":
            distance_meters = distance / 100
        elif units == "ft":
            distance_meters = distance / 3.28084
        elif units == "in":
            distance_meters = distance / 39.37
        elif units == "m":
            distance_meters = distance * 1

        # cap the speed
        if speed > 2:
            speed = 2
        elif speed < 0:
            speed = 0

        # read the current position and move relative to
        # position
        data = self.get_position_data()
        x = data[1]
        y = data[2]
        z = data[3]
        self.send_absolute_position(positionX=x + distance_meters,
                                    positionY=y,
                                    positionZ=z,
                                    velocity=speed,
                                    heading=0,
                                    rotationalVelocity=0)
        time.sleep(0.001)

    def move_backward(self, distance, units="cm", speed=1.0):
        """
        :param distance: the numerical value of the value to move
        :param units: can either be in inches, centimeters, meters, feet.
        :param speed: default 1 meter per second. Max is 2 meters/second.
        :return: N/A
        """
        if units == "cm":
            distance_meters = distance / 100
        elif units == "ft":
            distance_meters = distance / 3.28084
        elif units == "in":
            distance_meters = distance / 39.37
        elif units == "m":
            distance_meters = distance * 1

        # cap the speed
        if speed > 2:
            speed = 2
        elif speed < 0:
            speed = 0
        data = self.get_position_data()
        x = data[1]
        y = data[2]
        z = data[3]
        self.send_absolute_position(positionX=x - distance_meters,
                                    positionY=y,
                                    positionZ=z,
                                    velocity=speed,
                                    heading=0,
                                    rotationalVelocity=0)
        time.sleep(0.001)

    def move_left(self, distance, units="cm", speed=1.0):
        """
        :param distance: the numerical value of the value to move
        :param units: can either be in inches, centimeters, meters, feet.
        :param speed: default 1 meter per second. Max is 2 meters/second.
        :return: N/A
        """
        if units == "cm":
            distance_meters = distance / 100
        elif units == "ft":
            distance_meters = distance / 3.28084
        elif units == "in":
            distance_meters = distance / 39.37
        elif units == "m":
            distance_meters = distance * 1

        # cap the speed
        if speed > 2:
            speed = 2
        elif speed < 0:
            speed = 0

        data = self.get_position_data()
        x = data[1]
        y = data[2]
        z = data[3]
        self.send_absolute_position(positionX=x,
                                    positionY=y + distance_meters,
                                    positionZ=z,
                                    velocity=speed,
                                    heading=0,
                                    rotationalVelocity=0)
        time.sleep(0.001)

    def move_right(self, distance, units="cm", speed=1.0):
        """
        :param distance: the numerical value of the value to move
        :param units: can either be in inches, centimeters, meters, feet.
        :param speed: default 1 meter per second. Max is 2 meters/second.
        :return: N/A
        """
        if units == "cm":
            distance_meters = distance / 100
        elif units == "ft":
            distance_meters = distance / 3.28084
        elif units == "in":
            distance_meters = distance / 39.37
        elif units == "m":
            distance_meters = distance * 1

        # cap the speed
        if speed > 2:
            speed = 2
        elif speed < 0:
            speed = 0
        data = self.get_position_data()
        x = data[1]
        y = data[2]
        z = data[3]
        self.send_absolute_position(positionX=x,
                                    positionY=y - distance_meters,
                                    positionZ=z,
                                    velocity=speed,
                                    heading=0,
                                    rotationalVelocity=0)
        time.sleep(0.001)

    def send_absolute_position(self, positionX, positionY, positionZ, velocity, heading, rotationalVelocity):
        """
        drone movement command

        Use real values for position and velocity, and
        integer values for heading and rotational Velocity.
        :param positionX: float	-10.0 ~ 10.0	meter	Front (+), Back (-)
        :param positionY: float	-10.0 ~ 10.0	meter	Left(+), Right(-)
        :param positionZ: float	-10.0 ~ 10.0	meter	Up (+), Down (-)
        :param velocity: float	0.5 ~ 2.0	m/s	position movement speed
        :param heading: Int16	-360 ~ 360	degree	Turn left (+), turn right (-)
        :param rotationalVelocity: Int16	10 ~ 360	degree/s	left and right rotation speed
        :return: None
        """

        if not (isinstance(positionX, float) or isinstance(positionX, int)):
            return None

        if not (isinstance(positionY, float) or isinstance(positionY, int)):
            return None

        if not (isinstance(positionZ, float) or isinstance(positionZ, int)):
            return None

        if not (isinstance(velocity, float) or isinstance(velocity, int)):
            return None

        if (not isinstance(heading, int)) or (not isinstance(rotationalVelocity, int)):
            return None

        from math import sqrt

        header = Header()

        header.dataType = DataType.Control
        header.length = ControlPosition.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = ControlPosition()

        pos_data = self.get_position_data()
        # calculate the deltas in position needed to fly in those axis.

        data.positionX = float(positionX) - (self.previous_land[0] + pos_data[1])
        data.positionY = float(positionY) - (self.previous_land[1] + pos_data[2])
        data.positionZ = float(positionZ) - pos_data[3]
        data.velocity = float(velocity)
        data.heading = heading
        data.rotationalVelocity = rotationalVelocity

        self.transfer(header, data)

    def goto_waypoint(self, waypoint, velocity):
        """
        drone movement command

        Use real values for position and velocity, and
        integer values for heading and rotational Velocity.
        :param positionX: float	-10.0 ~ 10.0	meter	Front (+), Back (-)
        :param positionY: float	-10.0 ~ 10.0	meter	Left(+), Right(-)
        :param positionZ: float	-10.0 ~ 10.0	meter	Up (+), Down (-)
        :param velocity: float	0.5 ~ 2.0	m/s	position movement speed
        :param heading: Int16	-360 ~ 360	degree	Turn left (+), turn right (-)
        :param rotationalVelocity: Int16	10 ~ 360	degree/s	left and right rotation speed
        :return: None
        """

        if not (isinstance(waypoint[0], float) or isinstance(waypoint[0], int)):
            return None

        if not (isinstance(waypoint[1], float) or isinstance(waypoint[1], int)):
            return None

        if not (isinstance(waypoint[2], float) or isinstance(waypoint[2], int)):
            return None

        if not (isinstance(velocity, float) or isinstance(velocity, int)):
            return None

        from math import sqrt

        header = Header()

        header.dataType = DataType.Control
        header.length = ControlPosition.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = ControlPosition()

        data.positionX = float(waypoint[0]) - (self.previous_land[0] + self.get_position_data()[1])
        data.positionY = float(waypoint[1]) - (self.previous_land[1] + self.get_position_data()[2])
        data.positionZ = data.positionZ
        data.velocity = float(velocity)
        data.heading = 0
        data.rotationalVelocity = 0

        temp = sqrt(data.positionX ** 2 + data.positionY ** 2)
        wait = (temp / velocity) + 1
        self.transfer(header, data)
        time.sleep(wait)

    # Control End
    def percent_error(self, desired, current):
        """
        Calculates the percent error of two values.

        :param desired: numerical value
        :param current: numerical value
        :return: positive or negative value of error percent
        """

        error_percent = (current - desired)

        # cap the value between -100% and 100%
        if error_percent > 100:
            error_percent = 100
        elif error_percent < -100:
            error_percent = -100

        return error_percent

    def turn_degree(self, degree, timeout=3, p_value=10):
        """
        Turns right or left with absolute referemce frame to
        drone's initial heading.
        Positive degrees turn to right and
        negative degrees turn to the left.

        :param degree: integer from -180->180 degrees
        :param timeout: duration in seconds that drone will try to turn
        :param p_value: the gain of the proportional controller,
        if this increased CDE will turn quicker, the smaller the slower.
        examples values 0.5 -> 1.5
        :return: None
        """

        # make sure you arent moving
        self.hover(0.01)
        init_time = time.time()
        time_elapsed = time.time() - init_time
        init_angle = self.get_z_angle()
        desired_angle = degree

        if desired_angle >= 180:
            desired_angle = 180
        elif desired_angle <= -180:
            desired_angle = -180

        while time_elapsed < timeout:
            time_elapsed = time.time() - init_time
            current_angle = self.get_z_angle()
            # find the distance to the desired angle
            degree_diff = desired_angle - current_angle

            # save the first distance turning in one direction
            degree_dist_1 = abs(degree_diff)

            # now we find what direction to turn
            # sign will be either +1 or -1
            if degree_dist_1 > 0:
                sign = degree_diff / degree_dist_1
            else:
                sign = 1

            # now save the second distance going
            # the opposite direction turning
            degree_dist_2 = 360 - degree_dist_1

            # find which one is the shorter path
            if degree_dist_1 <= degree_dist_2:
                # normalize the degree distance so it goes from 0-100%
                # where 100% would be 360 degree distance
                error_percent = int(degree_dist_1 / 360 * 100)
                error_percent = int(error_percent * p_value)
                # cap the value between -100% and 100%
                if error_percent > 100:
                    error_percent = 100
                elif error_percent < -100:
                    error_percent = -100
                # now we can use the sign to determine the turning direction
                speed = int(sign * error_percent)
                self.sendControl(0, 0, speed, 0)
                time.sleep(0.005)
                # print( error_percent, " ,", current_angle," ,", sign," ,", degree_dist_1,
                #      " ,", degree_dist_2, " ,", speed, " ,", time.time(), " ,", desired_angle)


            elif degree_dist_2 < degree_dist_1:
                error_percent = int(degree_dist_2 / 360 * 100)
                error_percent = int(error_percent * p_value)
                # cap the value between -100% and 100%
                if error_percent > 100:
                    error_percent = 100
                elif error_percent < -100:
                    error_percent = -100
                speed = int(-1 * sign * error_percent)
                self.sendControl(0, 0, speed, 0)
                time.sleep(0.005)
                # print( error_percent, " ,", current_angle," ,", sign," ,", degree_dist_1,
                #       " ,", degree_dist_2, " ,", speed, " ,", time.time(), " ,", desired_angle)

        # stop any movement just in case
        self.hover(0.05)

    def turn_left(self, degree=90, timeout=3):
        # make sure it is an int and a positive value
        degree = int(abs(degree))
        # cap the max value to turn to 180
        if degree >= 180:
            degree = 179

        current_degree = self.get_z_angle()
        # positive degrees are to the left
        des_degree = degree + current_degree
        if des_degree > 180:
            new_degree = -(360 - des_degree)
            self.turn_degree(new_degree, timeout=timeout)
        else:
            self.turn_degree(des_degree, timeout=timeout)

    def turn_right(self, degree=90, timeout=3):
        # make sure it is an int and a positive value
        degree = int(abs(degree))
        # cap the max value to turn to 180
        if degree >= 180:
            degree = 179

        current_degree = self.get_z_angle()
        # positive degrees are to the left
        des_degree = current_degree - degree
        if des_degree < -180:
            new_degree = (360 - des_degree)
            self.turn_degree(new_degree, timeout=timeout)
        else:
            self.turn_degree(des_degree, timeout=timeout)

    # Flight Sequences Start
    def avoid_wall(self, timeout=2, distance=70):
        """
        A looped method that makes the drone fly forward until it reaches
        a desired distance.
        The range of front sensor is from 0cm-100cm

        :param timeout:  duration in seconds that function will run
        :param distance: distance in cm the drone will stop in front of object
        :return:
        """
        threshold = 20
        p_value = 0.4
        counter = 0

        init_time = time.time()
        time_elapsed = time.time() - init_time
        prev_distance = 0
        change_in_distance = 0
        while time_elapsed < timeout:
            time_elapsed = time.time() - init_time
            current_distance = self.get_front_range("cm")
            change_in_distance = prev_distance - current_distance
            prev_distance = current_distance
            data_now = self.get_flow_data()
            error_percent = self.percent_error(desired=distance, current=current_distance)
            # speed can range from -100 to 100
            speed = int(error_percent * p_value)
            # print(data_now[0], ",", current_distance, ",",change_in_distance, ",",data_now[1], ",",data_now[2])

            if current_distance > distance + threshold or current_distance < distance - threshold:
                self.sendControl(0, speed, 0, 0)
                time.sleep(0.005)
            else:
                self.hover()
                counter = counter + 1
                if counter == 20:
                    break
        self.hover()

    def keep_distance(self, timeout=2, distance=50):
        """
        A looped method that makes the drone fly forward until it reaches
        a desired distance. The drone will keep that distance.
        The range of front sensor is from 0cm-100cm

        :param timeout: duration in seconds that function will run
        :param distance: distance in cm the drone will maintain in front of object
        :return:
        """
        threshold = 10
        p_value = 0.4

        init_time = time.time()
        time_elapsed = time.time() - init_time
        while time_elapsed < timeout:
            time_elapsed = time.time() - init_time
            current_distance = self.get_front_range("cm")
            error_percent = self.percent_error(desired=distance, current=current_distance)
            # speed can range from -100 to 100
            speed = int(error_percent * p_value)

            if current_distance > distance + threshold or current_distance < distance - threshold:
                self.sendControl(0, speed, 0, 0)
                time.sleep(0.005)
            else:
                self.hover()

    def detect_wall(self, distance=50):
        """
        Returns True when a distance below the threshold is reached.
        The range of front sensor is from 0cm-100cm

        :param distance: threshold in millimeters that returns True
        :return: Boolean
        """
        current_distance = self.get_front_range("cm")

        if current_distance < distance:
            return True
        else:
            return False

    def flip(self, direction="back"):
        """
        Calls sendFlip() command to flip the drone in desired direction.
        Options are: "front", "back", "left", and "right"

        :param string that determines flip direction
        :return: None
        """

        state = self.get_state_data()
        battery = state[6]
        if battery < 50:
            print("Battery too low for flip. Battery =", battery, "%.")
            self.controller_buzzer(587, 100)
            self.controller_buzzer(554, 100)
            self.controller_buzzer(523, 100)
            self.controller_buzzer(494, 150)
            return

        if direction == "back":
            mode = FlightEvent.FlipRear
        elif direction == "front":
            mode = FlightEvent.FlipFront
        elif direction == "right":
            mode = FlightEvent.FlipRight
        elif direction == "left":
            mode = FlightEvent.FlipLeft
        else:
            print("Invalid flip direction.")
            return

        self.sendFlip(mode)

    def sendFlip(self, mode):

        header = Header()
        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()
        data.commandType = CommandType.FlightEvent
        data.option = mode.value
        return self.transfer(header, data)

    def square(self, speed=60, seconds=1, direction=1):
        """
        Flies the drone in the shape of a square. Defaults to the right.

        :param speed: integer from 0 to 100
        :param seconds:  integer that describes the duration of each side
        :param direction: integer, -1 or 1 that determines direction.
        :return:
        """

        power = int(speed)
        duration = int(seconds * 1000)

        self.sendControlWhile(0, power, 0, 0, duration)  # Pitch
        self.sendControlWhile(0, -power, 0, 0, 50)

        self.sendControlWhile(power * direction, 0, 0, 0, duration)  # roll
        self.sendControlWhile(-power * direction, 0, 0, 0, 50)

        self.sendControlWhile(0, -power, 0, 0, duration)  # -Pitch
        self.sendControlWhile(0, power, 0, 0, 50)

        self.sendControlWhile(-power * direction, 0, 0, 0, duration)  # Roll
        self.sendControlWhile(power * direction, 0, 0, 0, 50)

    def triangle(self, speed=60, seconds=1, direction=1):
        """
        Flies the drone in the shape of a triangle. Defaults to the right.

        :param speed: integer from 0 to 100
        :param seconds:  integer that describes the duration of each side
        :param direction: integer, -1 or 1 that determines direction.
        :return:
                """
        power = int(speed)
        duration = int(seconds * 1000)

        self.sendControlWhile(power * direction, power, 0, 0, duration)  # Pitch
        self.sendControlWhile(-power * direction, -power, 0, 0, 50)

        self.sendControlWhile(power * direction, -power, 0, 0, duration)  # -Pitch
        self.sendControlWhile(-power * direction, power, 0, 0, 50)

        self.sendControlWhile(-power * direction, 0, 0, 0, duration)  # Roll
        self.sendControlWhile(power * direction, 0, 0, 0, 50)

    def triangle_turn(self, speed=60, seconds=2, direction=1):
        """
        Flies the drone in the shape of a triangle by changing yaw. Defaults to the right.

        :param speed: integer from 0 to 100
        :param seconds:  integer that describes the duration of each side
        :param direction: integer, -1 or 1 that determines direction.
        :return:
        """
        # TODO Check this
        power = int(speed)
        duration = int(seconds * 1000)
        self.sendControlWhile(power * direction, power, 0, 0, duration)
        self.sendControlWhile(power * direction, -power, 0, 0, duration)
        self.sendControlWhile(-power * direction, 0, 0, 0, duration)

    def spiral(self, speed=50, seconds=5, direction=1):
        """
        Flies the drone in a downward spiral for a specified duration. Defaults to the right.

        :param speed: integer from 0 to 100
        :param seconds:  integer that describes the duration of the movement
        :param direction: integer, -1 or 1 that determines direction.
        :return:
        """
        power = int(speed)
        self.sendControl(0, power, 100 * -direction, -power)
        time.sleep(seconds)

    def circle(self, speed=75, direction=1):
        """
       Flies the drone in a circular turn. Defaults to the right.

       :param speed: integer from 0 to 100
       :param direction: integer, -1 or 1 that determines direction.
       :return:
       """
        # TODO Fix this later with gyro
        self.sendControl(0, speed, direction * speed, 0)
        time.sleep(5)

    def circle_turn(self, speed=30, seconds=1, direction=1):
        pitch = int(speed)
        roll = 0;
        for i in range(4):
            self.sendControlWhile(roll, pitch, 0, 0, 400)
            roll = roll + 10
            pitch = pitch - 10
        for i in range(4):
            self.sendControlWhile(roll, pitch, 0, 0, 400)
            roll = roll - 10
            pitch = pitch - 10
        for i in range(4):
            self.sendControlWhile(roll, pitch, 0, 0, 400)
            roll = roll - 10
            pitch = pitch + 10
        for i in range(4):
            self.sendControlWhile(roll, pitch, 0, 0, 400)
            roll = roll + 10
            pitch = pitch + 10

    def sway(self, speed=30, seconds=2, direction=1):
        """
        Moves the drone left and right twice. Defaults to start to the left

        :param speed: integer from 0 to 100
        :param seconds:  integer that describes the duration of the movement
        :param direction: integer, -1 or 1 that determines direction.
        :return:
        """
        power = int(speed)
        duration = int(seconds * 1000)
        for i in range(2):
            self.sendControlWhile(-power * direction, 0, 0, 0, duration)
            self.sendControlWhile(power * direction, 0, 0, 0, duration)

    # Flight Sequences End

    # Setup Start

    def sendCommand(self, commandType, option=0):
        """
        Used to send commands to the drone.
        The option must contain either a value value
         of each format or a numeric value.
        https://dev.byrobot.co.kr/documents/kr/products/e_drone/library/python/e_drone/04_protocol/#CommandType
        :param commandType: CommandType	command type
        :param option: 	ModeControlFlight	option
                        FlightEvent
                        Headless
                        Trim
                        UInt8
        :return: transfer()
        """

        if ((not isinstance(commandType, CommandType)) or
                (not isinstance(option, int))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = commandType
        data.option = option

        return self.transfer(header, data)

    # Sounds

    def start_drone_buzzer(self, note):
        """
       starts buzzer indefinitely.

        :param note: integer frequency or Note object
        :param duration: duration of the note in milliseconds
        :return: None
        """

        if isinstance(note, int):
            mode = BuzzerMode.Hz
            note_value = note

        elif isinstance(note, Note):
            mode = BuzzerMode.Scale
            note_value = note.value

        else:
            print("Input must be Note or integer.")
            return

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Tester
        header.to_ = DeviceType.Drone

        data = Buzzer()

        data.mode = mode
        data.value = note_value
        data.time = 65535

        self.transfer(header, data)

    def stop_drone_buzzer(self):
        """
        stops buzzer.

        :return: None
        """
        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Tester
        header.to_ = DeviceType.Drone

        data = Buzzer()

        data.mode = BuzzerMode.Mute
        data.value = BuzzerMode.Mute.value
        data.time = 1

        return self.transfer(header, data)

    def start_controller_buzzer(self, note):
        """
       starts buzzer indefinitely.

        :param note: integer frequency or Note object
        :param duration: duration of the note in milliseconds
        :return: None
        """

        if isinstance(note, int):
            mode = BuzzerMode.Hz
            note_value = note

        elif isinstance(note, Note):
            mode = BuzzerMode.Scale
            note_value = note.value

        else:
            print("Input must be Note or integer.")
            return

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Tester
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = mode
        data.value = note_value
        data.time = 65535

        self.transfer(header, data)

    def stop_controller_buzzer(self):
        """
        stops buzzer.

        :return: None
        """
        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Tester
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = BuzzerMode.Mute
        data.value = BuzzerMode.Mute.value
        data.time = 1

        return self.transfer(header, data)

    def controller_buzzer(self, note, duration):
        """
        Plays a note using the controller's buzzer.

        :param note: integer frequency or Note object
        :param duration: duration of the note in milliseconds
        :return: None
        """

        if isinstance(note, int):
            mode = BuzzerMode.Hz

        elif isinstance(note, Note):
            mode = BuzzerMode.Scale
            note = note.value

        else:
            print("Input must be Note or integer.")
            return

        self.sendBuzzer(mode, note, duration)
        time.sleep(duration / 1000)
        self.sendBuzzerMute(0.01)

    def drone_buzzer(self, note, duration):
        """
        Plays a note using the drone's buzzer.

        :param note: integer frequency or Note object
        :param duration: duration of the note in milliseconds
        :return: None
        """
        if isinstance(note, int):
            mode = BuzzerMode.Hz
            note_value = note

        elif isinstance(note, Note):
            mode = BuzzerMode.Scale
            note_value = note.value

        else:
            print("Input must be Note or integer.")
            return

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Tester
        header.to_ = DeviceType.Drone

        data = Buzzer()

        data.mode = mode
        data.value = note_value
        data.time = duration

        self.transfer(header, data)
        time.sleep(duration / 1000)
        self.sendBuzzerMute(0.01)

    # Lights

    def set_drone_LED(self, r, g, b, brightness):
        """
        Changes the drone LED to a specified color using RGB values.

        :param r: integer from 0-255
        :param g: integer from 0-255
        :param b: integer from 0-255
        :param brightness: integer from 0-255
        :return: None
        """

        self.sendLightDefaultColor(LightModeDrone.BodyHold, brightness, r, g, b)
        time.sleep(0.005)

    def set_controller_LED(self, r, g, b, brightness):
        """
        Changes the controller LED to a specified color using RGB values.

        :param r: integer from 0-255
        :param g: integer from 0-255
        :param b: integer from 0-255
        :param brightness: integer from 0-255
        :return: None
        """
        self.sendLightDefaultColor(LightModeController.BodyHold, brightness, r, g, b)
        time.sleep(0.005)

    def drone_LED_off(self):
        """
        Turns off the drone LED.

        :return: None
        """

        self.sendLightDefaultColor(LightModeDrone.BodyHold, 0, 0, 0, 0)
        time.sleep(0.005)

    def controller_LED_off(self):
        """
        Turns off the controller LED.

        :return: None
        """
        self.sendLightDefaultColor(LightModeController.BodyHold, 0, 0, 0, 0)
        time.sleep(0.005)

    def sendCommandLightEvent(self, commandType, option, lightEvent, interval, repeat):
        """
        Command + LED Event
        Used to send commands to the drone.
        The option must contain either a value value of each format or a numeric value.

        :param commandType: CommandType	command type
        :param option:
        ModeControlFlight	option
        FlightEvent
        Headless
        Trim
        UInt8
        :param lightEvent: UInt8	LED operating mode
        :param interval: 0 ~ 65535	Internal brightness control function call cycle
        :param repeat: 0 ~ 255	number of repetitions
        :return: transfer()
        """

        if ((not isinstance(commandType, CommandType)) or
                (not isinstance(option, int)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = CommandLightEvent.getSize()
        header.from_ = DeviceType.Base

        data = CommandLightEvent()

        if isinstance(lightEvent, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, LightModeController):
            header.to_ = DeviceType.Controller
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, int):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent

        else:
            return None

        data.command.commandType = commandType
        data.command.option = option

        data.event.interval = interval
        data.event.repeat = repeat

        return self.transfer(header, data)

    def sendCommandLightEventColor(self, commandType, option, lightEvent, interval, repeat, r, g, b):

        if ((not isinstance(commandType, CommandType)) or
                (not isinstance(option, int)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = CommandLightEventColor.getSize()
        header.from_ = DeviceType.Base

        data = CommandLightEventColor()

        if isinstance(lightEvent, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, LightModeController):
            header.to_ = DeviceType.Controller
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, int):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent

        else:
            return None

        data.command.commandType = commandType
        data.command.option = option

        data.event.interval = interval
        data.event.repeat = repeat

        data.color.r = r
        data.color.g = g
        data.color.b = b

        return self.transfer(header, data)

    def sendCommandLightEventColors(self, commandType, option, lightEvent, interval, repeat, colors):

        if ((not isinstance(commandType, CommandType)) or
                (not isinstance(option, int)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(colors, Colors))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = CommandLightEventColors.getSize()
        header.from_ = DeviceType.Base

        data = CommandLightEventColors()

        if isinstance(lightEvent, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, LightModeController):
            header.to_ = DeviceType.Controller
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, int):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent

        else:
            return None

        data.command.commandType = commandType
        data.command.option = option

        data.event.interval = interval
        data.event.repeat = repeat

        data.colors = colors

        return self.transfer(header, data)

    def sendModeControlFlight(self, modeControlFlight):

        if not isinstance(modeControlFlight, ModeControlFlight):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.ModeControlFlight
        data.option = modeControlFlight.value

        return self.transfer(header, data)

    def sendHeadless(self, headless):

        if not isinstance(headless, Headless):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.Headless
        data.option = headless.value

        return self.transfer(header, data)

    def reset_sensor(self):
        """
        This method will reset the gyroscope and accelerometer
        and calibrate it.

        While this program runs the drone must be on a leveled flat surface
        in order for the Gyroscope and Accelerometer to be cleared.
        If the calibration is done while the drone is moving
        the drone will have incorrect calibration values
        resulting in incorrect gyro angles and accelerometer values

        :return:
        """
        # send RF command to clear the bias and initiate the calibration
        self.sendClearBias()
        time.sleep(0.01)

        print("Lay the drone on a flat surface & "
              "Do not move CoDrone EDU")
        time.sleep(1)
        while True:
            self.get_error_data()
            error_flag_now = self.error_data[1]
            if (error_flag_now & ErrorFlagsForSensor.Motion_Calibrating.value) == 0:
                break
                time.sleep(0.5)
        print("Done calibrating.")

    def set_trim(self, roll, pitch):
        """
        Sets the drone trim values for roll, pitch, yaw, and throttle.

        :param roll: integer from -100-100
        :param pitch: integer from -100-100
        :return: None
        """

        roll = int(roll)
        pitch = int(pitch)
        self.sendTrim(roll, pitch, 0, 0)
        time.sleep(0.005)

    def reset_trim(self):
        """
        Resets all of the trim values to 0.

        :return: None
        """
        self.sendTrim(0, 0, 0, 0)
        time.sleep(0.005)

    def get_trim(self):
        """
        Returns current trim values.

        :return: None
        """

        trim = self.get_trim_data()[1:3]
        time.sleep(0.005)
        return trim

    def sendTrim(self, roll, pitch, yaw, throttle):

        if ((not isinstance(roll, int)) or (not isinstance(pitch, int)) or (not isinstance(yaw, int)) or (
                not isinstance(throttle, int))):
            return None

        header = Header()

        header.dataType = DataType.Trim
        header.length = Trim.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Trim()

        data.roll = roll
        data.pitch = pitch
        data.yaw = yaw
        data.throttle = throttle

        return self.transfer(header, data)

    def sendWeight(self, weight):

        header = Header()

        header.dataType = DataType.Weight
        header.length = Weight.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Weight()

        data.weight = weight

        return self.transfer(header, data)

    def sendLostConnection(self, timeNeutral, timeLanding, timeStop):

        header = Header()

        header.dataType = DataType.LostConnection
        header.length = LostConnection.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = LostConnection()

        data.timeNeutral = timeNeutral
        data.timeLanding = timeLanding
        data.timeStop = timeStop

        return self.transfer(header, data)

    def sendFlightEvent(self, flightEvent):

        if ((not isinstance(flightEvent, FlightEvent))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = flightEvent.value

        return self.transfer(header, data)

    def sendClearBias(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.ClearBias
        data.option = 0

        return self.transfer(header, data)

    def sendClearTrim(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Tester
        header.to_ = DeviceType.Drone

        data = Command()

        data.commandType = CommandType.ClearTrim
        data.option = 0

        return self.transfer(header, data)

    def sendSetDefault(self, deviceType):

        if ((not isinstance(deviceType, DeviceType))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = deviceType

        data = Command()

        data.commandType = CommandType.SetDefault
        data.option = 0

        return self.transfer(header, data)

    def sendBacklight(self, flagPower):

        if ((not isinstance(flagPower, bool))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Command()

        data.commandType = CommandType.Backlight
        data.option = int(flagPower)

        return self.transfer(header, data)

    # Setup End

    # Device Start

    def sendMotor(self, motor0, motor1, motor2, motor3):

        if ((not isinstance(motor0, int)) or
                (not isinstance(motor1, int)) or
                (not isinstance(motor2, int)) or
                (not isinstance(motor3, int))):
            return None

        header = Header()

        header.dataType = DataType.Motor
        header.length = Motor.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = Motor()

        data.motor[0].value = motor0
        data.motor[1].value = motor1
        data.motor[2].value = motor2
        data.motor[3].value = motor3

        return self.transfer(header, data)

    def set_motor_speed(self, front_right=0, back_right=0, back_left=0, front_left=0, time_delay=0.001):
        """
        for controlling the invidual motor speeds
        0-1000 integer

        :return:
        """

        self.sendMotor(int(front_right), int(back_right), int(back_left), int(front_left))
        sleep(time_delay)

    def sendMotorSingle(self, target, value):

        if ((not isinstance(target, int)) or
                (not isinstance(value, int))):
            return None

        header = Header()

        header.dataType = DataType.MotorSingle
        header.length = MotorSingle.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Drone

        data = MotorSingle()

        data.target = target
        data.value = value

        return self.transfer(header, data)

    # Device End

    # Light Start

    def sendLightManual(self, deviceType, flags, brightness):

        if ((not isinstance(deviceType, DeviceType)) or
                (not isinstance(flags, int)) or
                (not isinstance(brightness, int))):
            return None

        header = Header()

        header.dataType = DataType.LightManual
        header.length = LightManual.getSize()
        header.from_ = DeviceType.Base
        header.to_ = deviceType

        data = LightManual()

        data.flags = flags
        data.brightness = brightness

        return self.transfer(header, data)

    def sendLightModeColor(self, lightMode, interval, r, g, b):

        if ((not isinstance(lightMode, int)) or
                (not isinstance(interval, int)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int))):
            return None

        header = Header()

        header.dataType = DataType.LightMode
        header.length = LightModeColor.getSize()
        header.from_ = DeviceType.Base

        data = LightModeColor()

        if isinstance(lightMode, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.mode.mode = lightMode.value

        elif isinstance(lightMode, LightModeController):
            header.to_ = DeviceType.Controller
            data.mode.mode = lightMode.value

        elif isinstance(lightMode, int):
            header.to_ = DeviceType.Drone
            data.mode.mode = lightMode

        else:
            return None

        data.mode.interval = interval

        data.color.r = r
        data.color.g = g
        data.color.b = b

        return self.transfer(header, data)

    def sendLightModeColors(self, lightMode, interval, colors):

        if ((not isinstance(interval, int)) or
                (not isinstance(colors, Colors))):
            return None

        header = Header()

        header.dataType = DataType.LightMode
        header.length = LightModeColors.getSize()
        header.from_ = DeviceType.Base

        data = LightModeColors()

        if isinstance(lightMode, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.mode.mode = lightMode.value

        elif isinstance(lightMode, LightModeController):
            header.to_ = DeviceType.Controller
            data.mode.mode = lightMode.value

        elif isinstance(lightMode, int):
            header.to_ = DeviceType.Drone
            data.mode.mode = lightMode

        else:
            return None

        data.mode.interval = interval
        data.colors = colors

        return self.transfer(header, data)

    def sendLightEventColor(self, lightEvent, interval, repeat, r, g, b):

        if ((not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEvent
        header.length = LightEventColor.getSize()
        header.from_ = DeviceType.Base

        data = LightEventColor()

        if isinstance(lightEvent, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, LightModeController):
            header.to_ = DeviceType.Controller
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, int):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent

        else:
            return None

        data.event.interval = interval
        data.event.repeat = repeat

        data.color.r = r
        data.color.g = g
        data.color.b = b

        return self.transfer(header, data)

    def sendLightEventColors(self, lightEvent, interval, repeat, colors):

        if ((not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(colors, Colors))):
            return None

        header = Header()

        header.dataType = DataType.LightEvent
        header.length = LightEventColors.getSize()
        header.from_ = DeviceType.Base

        data = LightEventColors()

        if isinstance(lightEvent, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, LightModeController):
            header.to_ = DeviceType.Controller
            data.event.event = lightEvent.value

        elif isinstance(lightEvent, int):
            header.to_ = DeviceType.Drone
            data.event.event = lightEvent

        else:
            return None

        data.event.interval = interval
        data.event.repeat = repeat

        data.colors = colors

        return self.transfer(header, data)

    def sendLightDefaultColor(self, lightMode, interval, r, g, b):

        if ((not isinstance(interval, int)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int))):
            return None

        header = Header()

        header.dataType = DataType.LightDefault
        header.length = LightModeColor.getSize()
        header.from_ = DeviceType.Base

        data = LightModeColor()

        if isinstance(lightMode, LightModeDrone):
            header.to_ = DeviceType.Drone
            data.mode.mode = lightMode.value

        elif isinstance(lightMode, LightModeController):
            header.to_ = DeviceType.Controller
            data.mode.mode = lightMode.value

        elif isinstance(lightMode, int):
            header.to_ = DeviceType.Drone
            data.mode.mode = lightMode

        else:
            return None

        data.mode.interval = interval

        data.color.r = r
        data.color.g = g
        data.color.b = b

        return self.transfer(header, data)

    # Light End

    # Color Start

    def get_colors(self):
        """
        Access the color data using the default ByRobot
        color prediction
        color 1 is the front sensor
        color 2 is the back sensor
        """
        color1 = self.get_color_data()[9].name
        color2 = self.get_color_data()[10].name
        colors = [color1, color2]
        return colors

    # this functions returns a string (red, blue, magenta..)
    def get_front_color(self):
        return self.get_color_data()[9].name

    def get_back_color(self):
        return self.get_color_data()[10].name

    def predict_colors(self, color_data):
        try:
            prediction_front = self.knn.predict([[color_data[1], color_data[2], color_data[3], color_data[4]]])
            prediction_back = self.knn.predict([[color_data[5], color_data[6], color_data[7], color_data[8]]])
            prediction = [prediction_front[0], prediction_back[0]]
            return prediction
        except:
            print("Error: A classifier has not been loaded. Call drone.load_classifier() and try again.")
            self.close()
            exit()

    def load_classifier(self, dataset=None, show_graph=False):

        # TODO Check first if all text tiles in the dataset have the same number of data points 0.6

        if dataset is None:  # path to default data inside of cde lib
            lib_dir = os.path.dirname(os.path.abspath(__file__))
            path = lib_dir + "/data/"

        else:
            path = os.path.join(self.parent_dir, dataset)  # user defined data

            if not os.path.isdir(path):
                print("Error: Cannot load classifier. Dataset " + dataset + " does not exist.")
                print("Use the new_color_data() method to add data.")
                self.close()
                exit()

            else:
                folder = os.listdir(path)

                if len(folder) == 0:
                    print("Error: Cannot load classifier. Dataset " + dataset + " is empty.")
                    print("Use the new_color_data() method to add data.")
                    self.close()
                    exit()

        all_data = []
        all_labels = []
        sample_size = None
        prev_sample_size = None
        samp_size_list = []
        sizes_different = False

        for filename in os.listdir(path):
            # print(filename)
            data = np.loadtxt(path + '/' + filename)
            all_data.append(data)

            sample_size = len(data)
            samp_size_list.append([sample_size, filename])

            if prev_sample_size is None:
                prev_sample_size = sample_size

            if prev_sample_size != sample_size:
                sizes_different = True

            prev_sample_size = sample_size
            all_labels.append(filename.strip('.txt'))

        if len(all_labels) < 3:
            print("Error: The data set must have at least 3 labels to call load_classifier().")
            print(f'Your data set "{dataset}" has {len(all_labels)} label(s)')

        if sizes_different:
            print("Error one or more files has a different sample size.")
            for sizes in samp_size_list:
                print("samples ", sizes[0], " filename ", sizes[1])

        x = []  # Hue
        y = []  # Saturation
        z = []  # Value
        w = []  # Luminosity
        labels_list = []

        for color in range(len(all_labels)):

            for i in range(len(all_data)):
                labels_list.append(all_labels[color])
                # use data from both sensors front
                x.append(all_data[color][i][1])  # Hue
                y.append(all_data[color][i][2])  # Saturation
                z.append(all_data[color][i][3])  # Value
                w.append(all_data[color][i][4])  # Luminosity

        x_data = []
        for i in range(len(x)):
            x_data.append([x[i], y[i], z[i], w[i]])
        y_data = labels_list
        self.knn.fit(x_data, y_data)

        if show_graph:
            from mpl_toolkits import mplot3d
            import matplotlib.pyplot as plt
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.scatter3D(x, y, z, c=z)
            plt.show()
        return all_data

    def print_num_data(self, label, dataset):

        folder = dataset
        parent_dir = os.getcwd()
        path = os.path.join(parent_dir, folder)
        filename = path + '/' + label + '.txt'
        file_exists = os.path.exists(filename)

        if not file_exists:
            print("Error: Cannot count data. Folder and file do not exist. Use new_color_data()")
            return

        data = np.loadtxt(filename)
        return len(data)

    def append_color_data(self, label, data, dataset):

        """
          This function will append data to an already existing label in a dataset.
          If the file doesn't exist, then it will print an error.

          :param label: String label name that will be used for the filename
          :param data: List of HSV data samples
          :param dataset: String folder name where the text file will be stored.
          :return: None
          """

        folder = dataset
        parent_dir = os.getcwd()
        path = os.path.join(parent_dir, folder)
        filename = path + '/' + label + '.txt'
        file_exists = os.path.exists(filename)

        if not os.path.isdir(path) or not file_exists:
            print("Error: Cannot append data. Folder and file do not exist. Use new_color_data()")
            return

        print("Appending data to " + label + "...")

        new_data = data  # new data we want to add
        con_data = np.array(new_data)  # convert it first to np array
        old_data = np.loadtxt(filename)  # load existing data
        all_data = np.concatenate((old_data, con_data))  # add the data
        np.savetxt(filename, all_data)  # save the new combined data

    def new_color_data(self, label, data, dataset):
        """
        This function creates a new textfile label.txt in a dataset folder.

        :param label: String label name that will be used for the filename
        :param data: List of HSV data samples
        :param dataset: String folder name where the text file will be stored.
        :return: None
        """

        folder = dataset
        parent_dir = os.getcwd()
        path = os.path.join(parent_dir, folder)

        if not os.path.isdir(path):
            # print("Creating new dataset.")
            os.makedirs(path)
        print("Adding " + label + " to", dataset)
        filename = label + ".txt"
        np.savetxt(path + '/' + filename, data)

    # Color End

    # Display Start

    def sendDisplayClearAll(self, pixel=DisplayPixel.White):

        if (not isinstance(pixel, DisplayPixel)):
            return None

        header = Header()

        header.dataType = DataType.DisplayClear
        header.length = DisplayClearAll.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayClearAll()

        data.pixel = pixel

        return self.transfer(header, data)

    def sendDisplayClear(self, x, y, width, height, pixel=DisplayPixel.White):

        if (not isinstance(pixel, DisplayPixel)):
            return None

        header = Header()

        header.dataType = DataType.DisplayClear
        header.length = DisplayClear.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayClear()

        data.x = x
        data.y = y
        data.width = width
        data.height = height
        data.pixel = pixel

        return self.transfer(header, data)

    def sendDisplayInvert(self, x, y, width, height):

        header = Header()

        header.dataType = DataType.DisplayInvert
        header.length = DisplayInvert.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayInvert()

        data.x = x
        data.y = y
        data.width = width
        data.height = height

        return self.transfer(header, data)

    def sendDisplayDrawPoint(self, x, y, pixel=DisplayPixel.Black):

        if (not isinstance(pixel, DisplayPixel)):
            return None

        header = Header()

        header.dataType = DataType.DisplayDrawPoint
        header.length = DisplayDrawPoint.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayDrawPoint()

        data.x = x
        data.y = y
        data.pixel = pixel

        return self.transfer(header, data)

    def sendDisplayDrawLine(self, x1, y1, x2, y2, pixel=DisplayPixel.Black, line=DisplayLine.Solid):

        if ((not isinstance(pixel, DisplayPixel)) or (not isinstance(line, DisplayLine))):
            return None

        header = Header()

        header.dataType = DataType.DisplayDrawLine
        header.length = DisplayDrawLine.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayDrawLine()

        data.x1 = x1
        data.y1 = y1
        data.x2 = x2
        data.y2 = y2
        data.pixel = pixel
        data.line = line

        return self.transfer(header, data)

    def sendDisplayDrawRect(self, x, y, width, height, pixel=DisplayPixel.Black, flagFill=False,
                            line=DisplayLine.Solid):

        if ((not isinstance(pixel, DisplayPixel)) or (not isinstance(flagFill, bool)) or (
                not isinstance(line, DisplayLine))):
            return None

        header = Header()

        header.dataType = DataType.DisplayDrawRect
        header.length = DisplayDrawRect.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayDrawRect()

        data.x = x
        data.y = y
        data.width = width
        data.height = height
        data.pixel = pixel
        data.flagFill = flagFill
        data.line = line

        return self.transfer(header, data)

    def sendDisplayDrawCircle(self, x, y, radius, pixel=DisplayPixel.Black, flagFill=True):

        if ((not isinstance(pixel, DisplayPixel)) or (not isinstance(flagFill, bool))):
            return None

        header = Header()

        header.dataType = DataType.DisplayDrawCircle
        header.length = DisplayDrawCircle.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayDrawCircle()

        data.x = x
        data.y = y
        data.radius = radius
        data.pixel = pixel
        data.flagFill = flagFill

        return self.transfer(header, data)

    def sendDisplayDrawString(self, x, y, message, font=DisplayFont.LiberationMono5x8, pixel=DisplayPixel.Black):

        if ((not isinstance(font, DisplayFont)) or (not isinstance(pixel, DisplayPixel)) or (
                not isinstance(message, str))):
            return None

        header = Header()

        header.dataType = DataType.DisplayDrawString
        header.length = DisplayDrawString.getSize() + len(message)
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayDrawString()

        data.x = x
        data.y = y
        data.font = font
        data.pixel = pixel
        data.message = message

        return self.transfer(header, data)

    def sendDisplayDrawStringAlign(self, x_start, x_end, y, message, align=DisplayAlign.Center,
                                   font=DisplayFont.LiberationMono5x8, pixel=DisplayPixel.Black):

        if ((not isinstance(align, DisplayAlign)) or (not isinstance(font, DisplayFont)) or (
                not isinstance(pixel, DisplayPixel)) or (not isinstance(message, str))):
            return None

        header = Header()

        header.dataType = DataType.DisplayDrawStringAlign
        header.length = DisplayDrawStringAlign.getSize() + len(message)
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = DisplayDrawStringAlign()

        data.x_start = x_start
        data.x_end = x_end
        data.y = y
        data.align = align
        data.font = font
        data.pixel = pixel
        data.message = message

        return self.transfer(header, data)

    def controller_create_canvas(self):
        """
        Creates a clean canvas for drawing
        :return: image object
        """
        image = PIL.Image.new("RGB", (127, 63), color=-1)
        return image

    def controller_preview_canvas(self, image):
        """
        Pops up a window of the current canvas
        :param image: the image
        :return: nothing
        """
        image.show()

    def controller_draw_canvas(self, image):
        """
        Draws custom image canvas onto the controller screen
        :param image: image to be drawn
        :return: nothing
        """
        img = list(image.getdata())
        self.controller_draw_image(img)

    def controller_draw_line(self, x1, y1, x2, y2, pixel=DisplayPixel.Black, line_type=DisplayLine.Solid):
        """
        (x1,y1) \
                 \
                  \
                   \ (x2,y2)
        draws a line between points (x1, y1) and (x2, y2)
        :param x1: point 1 x coordinate
        :param y1: point 1 y coordinate
        :param x2: point 2 x coordinate
        :param y2: point 2 y coordinate
        :param pixel: DisplayPixel type.
        :param line_type: type of line drawn
        :return: nothing
        """
        self.sendDisplayDrawLine(x1, y1, x2, y2, pixel, line_type)

    def controller_draw_rectangle(self, x, y, width, height, pixel=DisplayPixel.Black, fill=False,
                                  line_type=DisplayLine.Solid):
        """
                   width
        (x,y)|---------------|
             |               | height
             |_______________|

        draws a rectangle onto the controller screen starting from point (x,y) and extends to
        given height and width
        :param x: top left corner x coordinate
        :param y: top left corner y coordinate
        :param width: width of rectangle
        :param height: height of rectangle
        :param pixel: DisplayPixel type
        :param fill: False will be white, True will be black
        :param line_type: type of line drawn
        :return: nothing
        """
        self.sendDisplayDrawRect(x, y, width, height, pixel, fill, line_type)

    def controller_draw_square(self, x, y, width, pixel=DisplayPixel.Black, fill=False, line_type=DisplayLine.Solid):
        """
               width
        (x,y)|------|
             |      | width
             |______|
        draws a square on the controller screen starting from point (x,y) and will extend to the given
        width
        :param x: top left corner x coordinate
        :param y: top left corner y coordinate
        :param width: width of the square
        :param pixel: DisplayPixel type
        :param fill: False will be white, True will be black
        :param line_type: type of line drawn
        :return: nothing
        """
        self.sendDisplayDrawRect(x, y, width, width, pixel, fill, line_type)

    def controller_draw_point(self, x, y, pixel=DisplayPixel.Black):
        """
        draws a single pixel at the point (x,y)
        :param x: x coordinate
        :param y: y coordinate
        :param pixel: DisplayPixel type
        :return: nothing
        """
        self.sendDisplayDrawPoint(x, y, pixel)

    def controller_clear_screen(self, pixel=DisplayPixel.White):
        """
        clears all drawings from the controller screen
        :param pixel: make all pixels white or black. white is default.
        :return: nothing
        """
        self.sendDisplayClearAll(pixel)

    def controller_draw_polygon(self, point_list):
        """
        The polygon outline consists of straight lines between the
        given coordinates, plus a straight line between the last and the first coordinate.
        :param point_list: the list of coordinates
        :return: nothing
        """
        try:
            for i in range(len(point_list)):
                if i == len(point_list) - 1:
                    self.sendDisplayDrawLine(point_list[i][0], point_list[i][1], point_list[0][0], point_list[0][1])
                else:
                    self.sendDisplayDrawLine(point_list[i][0], point_list[i][1], point_list[i + 1][0],
                                             point_list[i + 1][1])
        except:
            print("Could not draw the list:", point_list)
            print("Use a list in the format: list [ (x1,y1), (x2, y2),..., (xn, yn) ]")

    def controller_draw_ellipse(self, ellipse_list, image, fill_in=None, pixel_width=1):
        """
        Draws an ellipse inside the given bounding box.
        :param ellipse_list: Two points to define the bounding box. Sequence of [(x0, y0), (x1, y1)]
                             where x1 >= x0 and y1 >= y0.
        :param image: image object created from create_image_canvas()
        :param fill_in: None by default. 0 will fill with black
        :param pixel_width: The line width, in pixels.
        :return: nothing
        """
        draw = PIL.ImageDraw.Draw(image)
        draw.ellipse(ellipse_list, fill=fill_in, width=pixel_width, outline=0)

    def controller_draw_arc(self, arc_list, start_angle, end_angle, image, pixel_width=1):
        """
        Draws an arc (a portion of a circle outline) between the start and end angles, inside the given bounding box.
        :param arc_list: Two points to define the bounding box. Sequence of [(x0, y0), (x1, y1)], where x1 >= x0
        and y1 >= y0.
        :param start_angle: Starting angle, in degrees. Angles are measured from 3 o’clock, increasing clockwise.
        :param end_angle: Ending angle, in degrees.
        :param image: image object created from create_image_canvas()
        :param pixel_width: The line width, in pixels.
        :return: nothing
        """
        draw = PIL.ImageDraw.Draw(image)
        draw.arc(arc_list, start_angle, end_angle, 0, width=pixel_width)

    def controller_draw_chord(self, chord_list, start_angle, end_angle, image, pixel_width=1):
        """
        Same as controller_draw_arc(), but connects the end points with a straight line.
        :param chord_list: Two points to define the bounding box. Sequence of [(x0, y0), (x1, y1)], where x1 >= x0
        and y1 >= y0.
        :param start_angle: Starting angle, in degrees. Angles are measured from 3 o’clock, increasing clockwise.
        :param end_angle: Ending angle, in degrees.
        :param image: image object created from create_image_canvas()
        :param pixel_width: The line width, in pixels.
        :return: nothing
        """
        draw = PIL.ImageDraw.Draw(image)
        draw.chord(chord_list, start_angle, end_angle, 0, width=pixel_width)

    def controller_draw_string(self, x, y, string, string_font=DisplayFont.LiberationMono5x8,
                               pixel_color=DisplayPixel.Black):
        """
        Draws a string starting from the given x, y position
        :param x: starting x position
        :param y: starting y position
        :param string: the string to write
        :param string_font: font for the string
        :param pixel_color: color of string
        :return: nothing
        """
        self.sendDisplayDrawString(x, y, string, string_font, pixel_color)

    def controller_draw_string_align(self, x_start, x_end, y, string, alignment=DisplayAlign.Center,
                                     string_font=DisplayFont.LiberationMono5x8, pixel_color=DisplayPixel.Black):
        """
        Draws a string from the given x_start, x_end and y positions. The string can be aligned along the x_start
        and x_end positions
        :param x_start: starting x position
        :param x_end: ending x position
        :param y: y position
        :param string: the string to write
        :param alignment: alignment between x_start and x_end. can align Left, Right, or Center.
        :param string_font: font for the string
        :param pixel_color: color of the string
        :return: nothing
        """
        self.sendDisplayDrawStringAlign(x_start, x_end, y, string, align=alignment, font=string_font, pixel=pixel_color)

    def controller_draw_image(self, pixel_list):
        self.controller_clear_screen()
        """
        draws image when given a pixel_list of image data
        :param pixel_list: the list of image data
        :return: nothing
        """
        if isinstance(pixel_list, list) is not True:
            print("Error: the pixel list passed into controller_draw_image() is not a list")
            return

        num_elem = len(pixel_list[0])
        for k in range(64):
            for i in range(128):

                if (127 * k) + i == 8001:
                    return  # end
                else:
                    current_index = pixel_list[(127 * k) + i]

                if num_elem == 4:  # png
                    if current_index[0] > 200 and current_index[1] > 200 and current_index[2] > 200 and \
                            current_index[3] > 200:
                        None
                    elif current_index[0] == 0 and current_index[1] == 0 and current_index[2] == 0 and \
                            current_index[3] == 0:
                        None
                    else:
                        self.sendDisplayDrawPoint(i, k, DisplayPixel.Black)
                        self.sendDisplayDrawPoint(i, k, DisplayPixel.Black)

                elif num_elem == 3:  # jpg
                    if current_index[0] > 200 and current_index[1] > 200 and current_index[2] > 200:
                        None
                    else:
                        self.sendDisplayDrawPoint(i, k, DisplayPixel.Black)
                        self.sendDisplayDrawPoint(i, k, DisplayPixel.Black)
                else:
                    print("Can't find image type. Please use a .jpg or .png file")
                    return

    # Display End

    # Buzzer Start

    def sendBuzzer(self, mode, value, duration):

        if ((not isinstance(mode, BuzzerMode)) or
                (not isinstance(value, int)) or
                (not isinstance(duration, int))):
            return None

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = mode
        data.value = value
        data.time = duration

        return self.transfer(header, data)

    def sendBuzzerMute(self, duration):

        if (not isinstance(duration, int)):
            return None

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = BuzzerMode.Mute
        data.value = BuzzerScale.Mute.value
        data.time = duration

        return self.transfer(header, data)

    def sendBuzzerMuteReserve(self, duration):

        if (not isinstance(duration, int)):
            return None

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = BuzzerMode.MuteReserve
        data.value = BuzzerScale.Mute.value
        data.time = duration

        return self.transfer(header, data)

    def sendBuzzerScale(self, scale, duration):

        if ((not isinstance(scale, BuzzerScale)) or
                (not isinstance(duration, int))):
            return None

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = BuzzerMode.Scale
        data.value = scale.value
        data.time = duration

        return self.transfer(header, data)

    def sendBuzzerScaleReserve(self, scale, duration):

        if ((not isinstance(scale, BuzzerScale)) or
                (not isinstance(duration, int))):
            return None

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = BuzzerMode.ScaleReserve
        data.value = scale.value
        data.time = duration

        return self.transfer(header, data)

    def sendBuzzerHz(self, hz, duration):

        if ((not isinstance(hz, int)) or
                (not isinstance(duration, int))):
            return None

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = BuzzerMode.Hz
        data.value = hz
        data.time = duration

        return self.transfer(header, data)

    def sendBuzzerHzReserve(self, hz, duration):

        if ((not isinstance(hz, int)) or
                (not isinstance(duration, int))):
            return None

        header = Header()

        header.dataType = DataType.Buzzer
        header.length = Buzzer.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Buzzer()

        data.mode = BuzzerMode.HzReserve
        data.value = hz
        data.time = duration

        return self.transfer(header, data)

    # Buzzer End

    # Vibrator Start

    def sendVibrator(self, on, off, total):

        if ((not isinstance(on, int)) or
                (not isinstance(off, int)) or
                (not isinstance(total, int))):
            return None

        header = Header()

        header.dataType = DataType.Vibrator
        header.length = Vibrator.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Vibrator()

        data.mode = VibratorMode.Instantally
        data.on = on
        data.off = off
        data.total = total

        return self.transfer(header, data)

    def sendVibratorReserve(self, on, off, total):

        if ((not isinstance(on, int)) or
                (not isinstance(off, int)) or
                (not isinstance(total, int))):
            return None

        header = Header()

        header.dataType = DataType.Vibrator
        header.length = Vibrator.getSize()
        header.from_ = DeviceType.Base
        header.to_ = DeviceType.Controller

        data = Vibrator()

        data.mode = VibratorMode.Continually
        data.on = on
        data.off = off
        data.total = total

        return self.transfer(header, data)

# Vibrator End


# Update Start


# Update End
