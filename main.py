#######################################################
# This example sends BME280 data to the wolkAbout cloud.
# PCA9536 outputs can also be controlled from within the
# wolkAbout dashboard.
#
# Upload the manifest to the wolkAbout platform. 
########################################################

# imports
import i2c
import streams
from wolkabout.iot import iot
from wireless import wifi
from bosch.bme280 import bme280
from espressif.esp32net import esp32wifi as wifi_driver

PCA9554A_I2C_ADDRESS        = 0x38
PCA9554A_REG_INPUT_PORT		= 0x00
PCA9554A_REG_OUTPUT_PORT	= 0x01
PCA9554A_REG_POL_INVERSION	= 0x02
PCA9554A_REG_CONFIG			= 0x03

PCA9554A_CONF_OUTPUT		= 0x00
PCA9554A_CONF_INPUT			= 0xFF

PCA9554A_ALL_OUTPUTS_OFF	= 0x00

class PCA9554A(i2c.I2C):
    
    def __init__(self, drvname, addr = PCA9554A_I2C_ADDRESS , clk = 100000):
        i2c.I2C.__init__(self, drvname, addr, clk)
        self._addr = addr
        #self.start()
        
    def init(self, state = PCA9554A_ALL_OUTPUTS_OFF):
        self.writePin(state)
        self.write_bytes(PCA9554A_REG_CONFIG, PCA9554A_CONF_OUTPUT)
        return True
    
    def writePin(self, state):
        if state == True:
            self.write_bytes(PCA9554A_REG_OUTPUT_PORT, 0x01)
        elif state == False:
            self.write_bytes(PCA9554A_REG_OUTPUT_PORT, 0x00)
    
    def getStatus(self):
        pin_state = self.write_read(PCA9554A_REG_OUTPUT_PORT, 1)[0]
        return pin_state

# wifi details
wifi_ssid = "username"
wifi_pass = "password"
wifi_secu = wifi.WIFI_WPA2

# rgb pins
RED = D25
GREEN = D26
BLUE = D27

# enable console
streams.serial()

# wolkabout project details
device_key = "device key"
device_password = "device password"
actuator_references = ["R"]

# rgb pins set as output
pinMode(RED, OUTPUT)
pinMode(GREEN, OUTPUT)
pinMode(BLUE, OUTPUT)

# xChip instances
SW01 = bme280.BME280(I2C0)
OC03 = PCA9554A(I2C0)

# initialize sensors
SW01.start()
OC03.init()

# init the wifi driver
wifi_driver.auto_init()

# method that establishes a wifi connection
def wifi_connect():
    for retry in range(10):
        try:
            print("Establishing Link...")
            wifi.link(wifi_ssid, wifi_secu, wifi_pass)
            print("Link Established")
            digitalWrite(GREEN, HIGH)
            break
        except Exception as e:
            print("ooops, something wrong while linking :(", e)
            digitalWrite(GREEN, LOW)
            digitalWrite(RED, HIGH)
            sleep(1000)
            digitalWrite(RED, LOW)
            sleep(1000)

# connect to wifi
wifi_connect()

# establish a connection between device and wolkabout iot platform
try:
    device = iot.Device(device_key, device_password, actuator_references)
except Exception as e:
    print("Something went wrong while creating the device: ", e)


# Provide implementation of a way to read actuator status if your device has actuators
class ActuatorStatusProviderImpl(iot.ActuatorStatusProvider):

    def get_actuator_status(reference):

        if reference == actuator_references[0]:
            value = OC03.getStatus()
            print(value)
            if value == 1:
                return iot.ACTUATOR_STATE_READY, "true"
            else:
                return iot.ACTUATOR_STATE_READY, "false"


class ActuationHandlerImpl(iot.ActuationHandler):

    def handle_actuation(reference, value):
        print("Setting actuator " + reference + " to value: " + value)

        if reference == actuator_references[0]:
            if value == "false":
                OC03.writePin(False)
            else:
                if value == "true":
                    OC03.writePin(True)


try:
    wolk = iot.Wolk(device, ActuationHandlerImpl, ActuatorStatusProviderImpl)
except Exception as e:
    print("Something went wrong while creating the Wolk instance: ", e)

# Establish a connection to the WolkAbout IoT Platform
try:
    print("Connecting to WolkAbout IoT Platform")
    wolk.connect()
    print("Done")
except Exception as e:
    print("Something went wrong while connecting: ", e)

publish_period = 5000

try:
    while True:
        if not wifi.is_linked():
            wifi_connect()

        sleep(publish_period)

        print("Publishing sensor readings and actuator statuses")
        temperature = SW01.get_temp()
        humidity = SW01.get_hum()
        pressure = SW01.get_press()
        print("T", temperature, "H", humidity, "P", pressure)
        wolk.add_sensor_reading("T", temperature)
        wolk.add_sensor_reading("H", humidity)
        wolk.add_sensor_reading("P", pressure)

        wolk.publish()
except Exception as e:
    print("Something went wrong: ", e)
