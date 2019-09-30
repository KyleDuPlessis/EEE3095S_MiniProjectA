#!/usr/bin/python3
"""
Mini-Project A
October 2019
Kyle du Plessis - DPLKYL002
Hendri Vermeulen - VRMHEN004
"""

"""
TO DO LIST:

[- map variables to pins to be used from GPIO pinout diagram - currently set to "0"]

1. Currently have -> "RTC Time", "Sys Timer", "Humidity", "Temp", "Light"
(there are 7 in total) - still need to implement the others 
+ interface with RTC to get the correct RTC Time value to be printed out under "RTC Time"

2. Blynk app once this ^ is completely done + working

"""

# import relevant libraries
import RPi.GPIO as GPIO
import Adafruit_MCP3008
import datetime
import threading
import os

# initialise global variables

# select pins to be used from GPIO pinout diagram
# button pins
resetSystemTimerButton = 0
changeReadingIntervalButton = 0
stopStartMonitoringButton = 0

# SPI pins
MOSI = 0
MISO = 0
CLK = 0
CS = 0

# ADC analog input pins (CH0-CH7)
potentiometer = 0
temperatureSensor = 0
lightSensor = 0

# reference values for light sensor according to datasheet
lightSensor_MAX = 950  # when shining phone torch at light sensor
lightSensor_MIN = 50  # when holding finger over light sensor

# reference values for temperature sensor according to datasheet
V0 = 0.5
Tc = 0.01

# other global variables
systemTimer = 0  # system timer starts at 00:00:00
monitoringEnabled = True  # start monitoring
readingInterval = 1  # set reading interval to 1 second initially
programClosed = False

# set mode to BOARD pin numbering system
GPIO.setmode(GPIO.BOARD)

# configure push buttons as input in pull up mode
GPIO.setup(resetSystemTimerButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(changeReadingIntervalButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(stopStartMonitoringButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# configure SPI pins as input / output respectively
GPIO.setup(MOSI, GPIO.OUT)
GPIO.setup(MISO, GPIO.IN)
GPIO.setup(CLK, GPIO.OUT)
GPIO.setup(CS, GPIO.OUT)

# create an ADC object
adc = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, mosi=MOSI, miso=MISO)


# button functionality
# this function resets the system timer on button press
def pressResetSystemTimerButton(arg):
    if (GPIO.input(arg) == GPIO.LOW):
        global systemTimer  # reset system timer
        systemTimer = 0
        os.system('clear')  # clean console window
        print("Reset button pressed - system timer has been reset.")  # print out confirmation message for test cases


# this function changes the reading interval on button press
def pressChangeReadingIntervalButton(arg):
    if (GPIO.input(arg) == GPIO.LOW):
        global readingInterval  # change reading interval
        if (readingInterval == 1):
            readingInterval = 2
        elif (readingInterval == 2):
            readingInterval = 5
        elif (readingInterval == 5):
            readingInterval = 1
        os.system('clear')  # clean console window
        print("Change reading interval button pressed - reading interval is now {}s.".format(
            readingInterval))  # print out confirmation message for test cases


# this function stops / starts monitoring on button press without affecting the system timer
def pressStopStartMonitoringButton(arg):
    if (GPIO.input(arg) == GPIO.LOW):
        global monitoringEnabled  # stop / start monitoring [The system timer is not affected by this functionality.]
        monitoringEnabled = not monitoringEnabled
        os.system('clear')  # clean console window
        if (monitoringEnabled):
            print(
                "Stop / start monitoring button pressed - monitoring enabled.")  # print out confirmation message for test cases
        else:
            print(
                "Stop / start monitoring button pressed - monitoring disabled.")  # print out confirmation message for test cases


# inputs - interrupts and edge detection
# falling edge detection on buttons, ignoring further edges for 200ms for switch bounce handling
GPIO.add_event_detect(resetSystemTimerButton, GPIO.FALLING, callback=pressResetSystemTimerButton,
                      bouncetime=200)  # set callback function to pressResetSystemTimerButton function
GPIO.add_event_detect(changeReadingIntervalButton, GPIO.FALLING, callback=pressChangeReadingIntervalButton,
                      bouncetime=200)  # set callback function to pressChangeReadingIntervalButton function
GPIO.add_event_detect(stopStartMonitoringButton, GPIO.FALLING, callback=pressStopStartMonitoringButton,
                      bouncetime=200)  # set callback function to pressStopStartMonitoringButton function


# system timer functionality
# this function displays the logging information
def displayLoggingInformation():
    if (not programClosed):  # only continue if parent thread is running
        global systemTimer
        if (monitoringEnabled):
            loggingInformationLine = getCurrentLoggingInformation()
            # print out current logging information line
            print("{:<15}{:<15}{:<15}{:<15}{:<15}".format(loggingInformationLine[0], loggingInformationLine[1],
                                                          loggingInformationLine[2], loggingInformationLine[3],
                                                          loggingInformationLine[4]))

        # create a thread for reading from the ADC
        threading.Timer(readingInterval, displayLoggingInformation).start()
        systemTimer += readingInterval


# ADC functionality
# this function gets the ADC value from the ADC analog input pins (CH0-CH7)
def getADCValue(ADCValue):
    return adc.read_adc(ADCValue)


# this function converts the ADC value to a fraction of 3.3V
def convertPotentiometer(ADCValue):
    voltageValue = ADCValue * (3.3 / 1023)
    return "{:.2f} V".format(voltageValue)


# this function converts the ADC value to degrees Celsius
def convertTemperatureSensor(ADCValue):
    voltageValue = ADCValue * (3.3 / 1023)
    degreesCelsiusValue = (voltageValue - V0) / Tc  # calculation according to datasheet
    return "{:.1f} C".format(degreesCelsiusValue)


# this function reports a value between 0 and 1023
def convertLightSensor(ADCValue):
    value = (ADCValue - lightSensor_MIN) / (
            lightSensor_MAX - lightSensor_MIN)  # [check if calculation is correct / reports correct value between 0 and 1023]
    return "{:.0f}%".format(value)


# this function gets the current logging information
def getCurrentLoggingInformation():
    min, sec = divmod(systemTimer, 60)
    hrs, min = divmod(min, 60)

    RTCTime = datetime.datetime.now().strftime("%H:%M:%S")  # NOTE - INTERFACE WITH RTC HERE
    systemTimerValue = "{:02.0f}:{:02.0f}:{:04.1f}".format(hrs, min, sec)

    potentiometerValue = convertPotentiometer(getADCValue(potentiometer))
    temperatureSensorValue = convertTemperatureSensor(getADCValue(temperatureSensor))
    lightSensorValue = convertLightSensor(getADCValue(lightSensor))

    return [RTCTime, systemTimerValue, potentiometerValue, temperatureSensorValue, lightSensorValue]


# main function
try:
    os.system('clear')
    print("Ready...")
    print("{:<15}{:<15}{:<15}{:<15}{:<15}".format("RTC Time", "Sys Timer", "Humidity", "Temp",
                                                  "Light"))  # 5 values to printed to screen (7 in total - add others later)
    displayLoggingInformation()

    # waiting for button presses - keep program running
    while (1):
        pass

except KeyboardInterrupt:
    print("Closing program...")
    # release all resources
    programClosed = True
    GPIO.cleanup()
