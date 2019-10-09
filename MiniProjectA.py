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

# initialise global variables

# select pins to be used from GPIO pinout diagram
# button pins
resetSystemTimerButton = 17
changeReadingIntervalButton = 27
stopStartMonitoringButton = 22

# SPI0 ADC pins
MOSI = 10
MISO = 9
CLK = 11
CS = 8

#Get I2C RTC Connection
RTCAddr = 0x6f
RTCSecReg = 0x00
RTCMinReg = 0x01
RTCHourReg = 0x02
TIMEZONE = 2
RTC = I2C.get_i2c_device(RTCAddr)

#Convert int to RTC BCD seconds
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

#init RTC
#get current time
currentDate = datetime.datetime.now()
#Set masks
RTCSecMask = 0b10000000
RTCMinMask = 0b0
RTCHourMask = 0b0
#Get current Time
startSec = int(currentDate.strftime("%S"))
startMin = int(currentDate.strftime("%M"))
startHour = int(currentDate.strftime("%H"))
#Update RTC
RTC.write8(RTCSecReg, RTCSecMask | decCompensation(startSec))
RTC.write8(RTCMinReg, RTCMinMask | decCompensation(startMin))
RTC.write8(RTCHourReg, RTCHourMask | decCompensation(startHour))

# ADC analog input pins (CH0-CH7)
potentiometer = 0
temperatureSensor = 1
lightSensor = 7

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
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

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
        #threading.Timer(readingInterval, displayLoggingInformation).start()
        #systemTimer += readingInterval Moved to update with rtc


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

#Convert from RTC BCD to int
def convertRTCBCDtoInt(bcd):
	firstDigit = bcd & 0b00001111
	secondDigit = (bcd & 0b01110000) >> 4;
	return secondDigit*10 + firstDigit

#Gets the time from RTC
def getTimeFromRTCandUpdateSystemTimer():
	global systemTimer
	global startHours
	global startMin
	global startSec
	
	sec = convertRTCBCDtoInt(RTC.readU8(RTCSecReg))
	min = convertRTCBCDtoInt(RTC.readU8(RTCMinReg))
	hrs = convertRTCBCDtoInt(RTC.readU8(RTCHourReg))
	
	systemTimer = (hrs*3600 + min*60 + sec) - (startHour*3600 + startMin*60 + startSec) 
	
	return "{:02.0f}:{:02.0f}:{:04.1f}".format(hrs, min, sec)

# this function gets the current logging information
def getCurrentLoggingInformation():
    RTCTime = getTimeFromRTCandUpdateSystemTimer()
    
    min, sec = divmod(systemTimer, 60)
    hrs, min = divmod(min, 60)
    
    systemTimerValue = "{:02.0f}:{:02.0f}:{:04.1f}".format(hrs, min, sec)

    potentiometerValue = convertPotentiometer(getADCValue(potentiometer))
    temperatureSensorValue = convertTemperatureSensor(getADCValue(temperatureSensor))
    lightSensorValue = convertLightSensor(getADCValue(lightSensor))

    return [RTCTime, systemTimerValue, potentiometerValue, temperatureSensorValue, lightSensorValue]

# main function - program logic
def main():
	displayLoggingInformation()
	time.sleep(1)
    # pass # waiting for button presses - keep program running
    #x = 1
    #print("write your logic here")

# only run the functions if
if __name__ == "__main__":

    # make sure the GPIO is stopped correctly
    try:

        #os.system('clear')
        print("Ready...")
        print("{:<15}{:<15}{:<15}{:<15}{:<15}".format("RTC Time", "Sys Timer", "Humidity", "Temp",
                                                      "Light"))  # 5 values to printed to screen (7 in total - add others later)
        displayLoggingInformation()

        # waiting for button presses - keep program running
        while True:
            main()

    except KeyboardInterrupt:

        print("Exiting gracefully")
        # release all resources
        programClosed = True
        # turn off GPIOs
        GPIO.cleanup()

    except Exception as e:

        print("Some other error occurred")
        print(e.message)
        # release all resources
        programClosed = True
        # turn off GPIOs
        GPIO.cleanup()
