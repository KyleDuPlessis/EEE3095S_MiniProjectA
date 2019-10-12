#!/usr/bin/python3
"""
Mini-Project A
October 2019
Kyle du Plessis - DPLKYL002
Hendri Vermeulen - VRMHEN004
"""

"""
TO DO LIST:

1. Map variables to pins to be used from GPIO pinout diagram - all currently set to "0"

2. Currently have -> "RTC Time", "Sys Timer", "Humidity", "Temp", "Light"
(there are 7 in total) - still need to implement the others 
+ interface with RTC to get the correct RTC Time value to be printed out under "RTC Time"

3. Alarm - "You can flash an LED using a PWM signal, you can play audio through the audio jack, or you
can buy a buzzer from WhiteLab." 
There are no buzzers available - got a speaker and will connect to Pi using an aux cable 
and we can play a sound mp3 audio file buzz sound - but not through the DAC.

[http://soundbible.com/2197-Analog-Watch-Alarm.html]

4. Blynk app once all this ^ is completely done + working

"""

# import relevant libraries
import RPi.GPIO as GPIO
import Adafruit_MCP3008
import datetime
import threading
import time
import os
import Adafruit_GPIO.I2C as I2C
import spidev














#https://github.com/blynkkk/lib-python/blob/master/examples/02_read_virtual_pin.py

import blynklib
import random

BLYNK_AUTH = 'Ui8zBgkyzsq0vUVDJMwxix5jWKSFk8nS'

# initialize blynk
blynk = blynklib.Blynk(BLYNK_AUTH)
READ_PRINT_MSG = "[READ_VIRTUAL_PIN_EVENT] Pin: V{}"


# register handler for virtual pin V11 reading
@blynk.handle_event('read V11')
def read_virtual_pin_handler(pin):
    print(READ_PRINT_MSG.format(pin))
    blynk.virtual_write(pin, random.randint(0, 255))

























# initialise global variables

# select pins to be used from GPIO pinout diagram
# button pins
resetSystemTimerButton = 17
changeReadingIntervalButton = 27
stopStartMonitoringButton = 22
dismissAlarmButton = 4

# SPI0 ADC pins
MOSI = 10
MISO = 9
CLK = 11
CS = 8

# PWM Alarm
PWM0 = 12
# PWM DAC
PWM1 = 13

# Get I2C RTC Connection
RTCAddr = 0x6f
RTCSecReg = 0x00
RTCMinReg = 0x01
RTCHourReg = 0x02
TIMEZONE = 2
RTC = I2C.get_i2c_device(RTCAddr)


# Convert int to RTC BCD seconds
def decCompensation(units):
    unitsU = units % 10;

    if (units >= 50):
        units = 0x50 + unitsU;
    elif (units >= 40):
        units = 0x40 + unitsU;
    elif (units >= 30):
        units = 0x30 + unitsU;
    elif (units >= 20):
        units = 0x20 + unitsU;
    elif (units >= 10):
        units = 0x10 + unitsU;
    return units;


# init RTC
# get current time
currentDate = datetime.datetime.now()
# Set masks
RTCSecMask = 0b10000000
RTCMinMask = 0b0
RTCHourMask = 0b0
# Get current Time
startSec = int(currentDate.strftime("%S"))
startMin = int(currentDate.strftime("%M"))
startHour = int(currentDate.strftime("%H"))
# Update RTC
RTC.write8(RTCSecReg, RTCSecMask | decCompensation(startSec))
RTC.write8(RTCMinReg, RTCMinMask | decCompensation(startMin))
RTC.write8(RTCHourReg, RTCHourMask | decCompensation(startHour))

# ADC analog input pins (CH0-CH7)
potentiometer = 0
temperatureSensor = 1
lightSensor = 7

# reference values for light sensor according to datasheet
lightSensor_MIN = 50.0  # when shining phone torch at light sensor
lightSensor_MAX = 1023.0  # when holding finger over light sensor

# Alarm constants
dacVoltMin = 0.65
dacVoltMax = 2.65

# reference values for temperature sensor according to datasheet
V0 = 0.5
Tc = 0.01

# other global variables
systemTimer = 0  # system timer starts at 00:00:00
monitoringEnabled = True  # start monitoring
readingInterval = 1  # set reading interval to 1 second initially
programClosed = False

# set mode to BOARD pin numbering system
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# configure push buttons as input in pull up mode
GPIO.setup(resetSystemTimerButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(changeReadingIntervalButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(stopStartMonitoringButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(dismissAlarmButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# configure SPI0 ADC pins as input / output respectively
GPIO.setup(MOSI, GPIO.OUT)
GPIO.setup(MISO, GPIO.IN)
GPIO.setup(CLK, GPIO.OUT)
GPIO.setup(CS, GPIO.OUT)

# Setup PWM Alarm
GPIO.setup(PWM0, GPIO.OUT)
Alarm = GPIO.PWM(PWM0, 2)
Alarm.start(0);
Alarm.ChangeDutyCycle(0)

# Setup PWM DAC
GPIO.setup(PWM1, GPIO.OUT)
DAC = GPIO.PWM(PWM1, 1000)
DAC.start(0);
DAC.ChangeDutyCycle(0)

# DAC SPI
spi_max_speed = 4 * 1000000
spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = spi_max_speed

# create an ADC object
adc = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, mosi=MOSI, miso=MISO)

# Thread values
values = {
    "rtcTime": 0,
    "humidity": 0,
    "temp": 0,
    "light": 0,
    "dacOut": 0.0,
    "alarm": False
}
valuesUpdatorIsReady = False


def writeToDac(voltage):
    PWMVal = int((voltage / 3.3) * 100)
    DAC.ChangeDutyCycle(PWMVal)  # PWM DAC

    val = int((voltage / 3.3) * 1023)
    lowByte = val << 2 & 0b11111100
    highByte = ((val >> 6) & 0xff) | 0b0 << 7 | 0b0 << 6 | 0b1 << 5 | 0b1 << 4
    # print("Volt = ", voltage)
    # print("Highbyte = {0:8b}".format(highByte))
    # print("Lowbyte =  {0:8b}".format(lowByte))
    spi.xfer2([highByte, lowByte])


# button functionality
# this function resets the system timer on button press
def pressResetSystemTimerButton(arg):
    if (GPIO.input(arg) == GPIO.LOW):
        global systemTimer  # reset system timer
        global startSec
        global startMin
        global startHour
        systemTimer = 0;
        startMin, startSec = divmod(values["rtcTime"], 60)
        startHour, startMin = divmod(startMin, 60)
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


# this function stops / starts monitoring on button press without affecting the system timer
def dismissAlarm(arg):
    if (GPIO.input(arg) == GPIO.LOW):
        values["alarm"] = False
        Alarm.ChangeDutyCycle(0)


# inputs - interrupts and edge detection
# falling edge detection on buttons, ignoring further edges for 200ms for switch bounce handling
GPIO.add_event_detect(resetSystemTimerButton, GPIO.FALLING, callback=pressResetSystemTimerButton,
                      bouncetime=200)  # set callback function to pressResetSystemTimerButton function
GPIO.add_event_detect(changeReadingIntervalButton, GPIO.FALLING, callback=pressChangeReadingIntervalButton,
                      bouncetime=200)  # set callback function to pressChangeReadingIntervalButton function
GPIO.add_event_detect(stopStartMonitoringButton, GPIO.FALLING, callback=pressStopStartMonitoringButton,
                      bouncetime=200)  # set callback function to pressStopStartMonitoringButton function
GPIO.add_event_detect(dismissAlarmButton, GPIO.FALLING, callback=dismissAlarm,
                      bouncetime=200)  # set callback function to dismissAlarm function


def updateValues():
    global systemTimer
    global valuesUpdatorIsReady

    while (not programClosed):
        if (monitoringEnabled):
            valuesUpdatorIsReady = False

            sec = convertRTCBCDtoInt(RTC.readU8(RTCSecReg))
            min = convertRTCBCDtoInt(RTC.readU8(RTCMinReg))
            hrs = convertRTCBCDtoInt(RTC.readU8(RTCHourReg))

            values["rtcTime"] = (hrs * 3600 + min * 60 + sec)

            systemTimer = values["rtcTime"] - (startHour * 3600 + startMin * 60 + startSec)

            values["humidity"] = getADCValue(potentiometer) * (3.3 / 1023)
            values["temp"] = ((getADCValue(temperatureSensor) * (3.3 / 1023)) - V0) / Tc
            values["light"] = getADCValue(lightSensor)
            values["dacOut"] = (values["light"] / 1023.0) * values["humidity"]
            writeToDac(values["dacOut"]);

            valuesUpdatorIsReady = True
        time.sleep(float(readingInterval) / 10.0)


def updateAlarm():
    global systemTimer
    lastSound = systemTimer
    soundBefore = False
    alarmValHigh = False

    while (not programClosed):  # only continue if parent thread is running
        if (monitoringEnabled):
            if (systemTimer - lastSound > 179.99 or not soundBefore):
                if (values["dacOut"] > dacVoltMax or values["dacOut"] < dacVoltMin):
                    lastSound = systemTimer
                    values["alarm"] = True
                    soundBefore = True
                    Alarm.ChangeDutyCycle(50)
        time.sleep(float(readingInterval) / 10.0)


# system timer functionality
# this function displays the logging information
def displayLoggingInformation():
    global systemTimer
    global valuesUpdatorIsReady
    lastUpdated = systemTimer
    print("{:<15}{:<15}{:<15}{:<15}{:<15}{:<15}{:<15}".format(
        "RTC Time", "Sys Timer", "Humidity", "Temp", "Light", "DAC out", "Alarm"))

    loggingInformationLine = getCurrentLoggingInformation()
    print("{:<15}{:<15}{:<15}{:<15}{:<15}{:<15}{:<15}".format(
        loggingInformationLine[0],
        loggingInformationLine[1],
        loggingInformationLine[2],
        loggingInformationLine[3],
        loggingInformationLine[4],
        loggingInformationLine[5],
        loggingInformationLine[6]
    ))

    while (not programClosed):  # only continue if parent thread is running
        if (monitoringEnabled):
            if (systemTimer - lastUpdated > readingInterval - 0.1):
                lastUpdated = systemTimer
                loggingInformationLine = getCurrentLoggingInformation()
                # print out current logging information line
                print("{:<15}{:<15}{:<15}{:<15}{:<15}{:<15}{:<15}".format(
                    loggingInformationLine[0],
                    loggingInformationLine[1],
                    loggingInformationLine[2],
                    loggingInformationLine[3],
                    loggingInformationLine[4],
                    loggingInformationLine[5],
                    loggingInformationLine[6]
                ))
        time.sleep(float(readingInterval) / 5.0)


# ADC functionality
# this function gets the ADC value from the ADC analog input pins (CH0-CH7)
def getADCValue(ADCValue):
    return adc.read_adc(ADCValue)


# this function converts the ADC value to a fraction of 3.3V
def convertPotentiometer():
    return "{:.2f} V".format(values["humidity"])


# this function converts the ADC value to degrees Celsius
def convertTemperatureSensor():
    return "{:.1f} C".format(values["temp"])


# this function reports a value between 0 and 1023
def convertLightSensor():
    return "{:.0f}".format(values["light"])


# Convert from RTC BCD to int
def convertRTCBCDtoInt(bcd):
    firstDigit = bcd & 0b00001111
    secondDigit = (bcd & 0b01110000) >> 4;
    return secondDigit * 10 + firstDigit


# Gets the time from RTC
def formatTime(time):
    min, sec = divmod(time, 60)
    hrs, min = divmod(min, 60)
    return "{:02.0f}:{:02.0f}:{:04.1f}".format(hrs, min, sec)


# Gets the time from RTC
def getDACOutValue():
    return "{:02.2f}".format(values["dacOut"])


# Gets the time from RTC
def getAlarmValue():
    if (values["alarm"]):
        return "*"
    return ""


# this function gets the current logging information
def getCurrentLoggingInformation():
    RTCTime = formatTime(values["rtcTime"])
    systemTimerValue = formatTime(systemTimer)

    potentiometerValue = convertPotentiometer()
    temperatureSensorValue = convertTemperatureSensor()
    lightSensorValue = convertLightSensor()
    dacOutValue = getDACOutValue()
    alarmValue = getAlarmValue()

    return [RTCTime, systemTimerValue, potentiometerValue, temperatureSensorValue, lightSensorValue, dacOutValue,
            alarmValue]


# main function - program logic
def main():
    displayLoggingInformation()


# only run the functions if
if __name__ == "__main__":
    print("Creating threads...")
    valuesUpdator = threading.Thread(target=updateValues)
    alarm = threading.Thread(target=updateAlarm)

    # make sure the GPIO is stopped correctly
    try:

        # os.system('clear')
        print("Starting threads...")
        valuesUpdator.start()

        while (not valuesUpdatorIsReady):
            time.sleep(float(readingInterval) / 20.0)

        alarm.start()
        print("Ready...")

        while True:
            blynk.run()
            main()

    except KeyboardInterrupt:

        print("Exiting gracefully")
        # release all resources
        programClosed = True

        # wait for threads
        valuesUpdator.join()
        alarm.join()

        # turn off GPIOs
        GPIO.cleanup()

    except Exception as e:

        print("Some other error occurred")
        print(e.message)
        # release all resources
        programClosed = True

        # wait for threads
        valuesUpdator.join()
        logger.join()
        alarm.join()

        # turn off GPIOs
        GPIO.cleanup()
