import time, board, microcontroller, asyncio, busio
from digitalio import DigitalInOut, Direction, Pull

from adafruit_esp32spi import adafruit_esp32spi
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
import adafruit_requests as requests

import adafruit_minimqtt.adafruit_minimqtt as MQTT


# Setup a feed named `feed` for publishing.

def connected(client, userdata, flags, rc):
    client.subscribe("kb2040_env/feeds/#")
    client.subscribe("cams/#")
    client.subscribe("pings/#")

def disconnected(client, userdata, rc):
    # This method is called when the client is disconnected
    print("Disconnected from MQTT Broker!")


def message(client, topic, message):
    """Method callled when a client's subscribed feed has a new
    value.
    :param str topic: The topic of the feed with a new value.
    :param str message: The new value
    """
    print(f"New message on topic {topic}: {message}")
    try:
        if topic == "pings/living_cam":
            if message == "off":
                PINS[0].value = True
            else:
                PINS[0].value = False

        if topic == "pings/env_hub":
            if message == "off":
                PINS[1].value = True
            else:
                PINS[1].value = False

        if topic == "pings/d_link_living_cam":
            if message == "off":
                PINS[2].value = True
            else:
                PINS[2].value = False

        if topic == "pings/d_link_kitchen":
            if message == "off":
                PINS[3].value = True
            else:
                PINS[3].value = False

        if topic == "cams/kitchen_occupancy":
            if message == "on":
                PINS[4].value = True
            else:
                PINS[4].value = False

        if topic == "cams/living_room_occupancy":
            if message == "on":
                PINS[5].value = True
            else:
                PINS[5].value = False

        if topic == "kb2040_env/feeds/co2":
            if int(message) > 1100:
                PINS[6].value = True
            else:
                PINS[6].value = False

        if topic == "kb2040_env/feeds/gas-index":
            if int(message) > 150:
                PINS[7].value = True
            else:
                PINS[7].value = False

        if topic == "kb2040_env/feeds/particles-25um":
            if int(message) > 25:
                PINS[8].value = True
            else:
                PINS[8].value = False


        if topic == "kb2040_env/feeds/env-concentration-25um":
            if int(message) > 25:
                PINS[9].value = True
            else:
                PINS[9].value = False

        if topic == "kb2040_env/feeds/particles-03um":
            if int(message) > 2000:
                PINS[9].value = True
            else:
                PINS[9].value = False
    except Exception as e:
        print(e)


try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

esp32_cs = DigitalInOut(board.D13)
esp32_ready = DigitalInOut(board.D11)
esp32_reset = DigitalInOut(board.D12)

spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

requests.set_socket(socket, esp)

if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
    print("ESP32 found and in idle mode")

print("Connecting to AP...")
while not esp.is_connected:
    try:
        esp.connect_AP(secrets["ssid"], secrets["password"])
    except Exception as e:
        print("could not connect to AP, resetting ", e)
        time.sleep(20)
        microcontroller.reset()

    print("Connecting to MQTT Broker")
    try:
        # Initialize MQTT interface with the esp interface
        MQTT.set_socket(socket, esp)

        # Set up a MiniMQTT Client
        mqtt_client = MQTT.MQTT(
            broker=secrets["mqtt_broker"], username=secrets["mqtt_username"], password=secrets["mqtt_password"]
        )

        # Setup the callback methods above
        mqtt_client.on_connect = connected
        mqtt_client.on_disconnect = disconnected
        mqtt_client.on_message = message
        # Connect the client to the MQTT broker.
        mqtt_client.connect()
        print("MQTT connected!")
    except Exception as e:
        print(e)
        raise

# LED setup.
PINS = (DigitalInOut(board.A0), DigitalInOut(board.A1), DigitalInOut(board.A2), DigitalInOut(board.A3),
    DigitalInOut(board.D24), DigitalInOut(board.D4), DigitalInOut(board.D25), DigitalInOut(board.SDA),
    DigitalInOut(board.SCL), DigitalInOut(board.D5), DigitalInOut(board.D6), DigitalInOut(board.D9))

for pin in PINS:
    pin.direction = Direction.OUTPUT

mqtt_client.connect()

while True:
#     for blink in PINS:
#         blink.value = True
#         time.sleep(1)
#         mqtt_client.loop()
#         print(esp.ping("192.168.1.142", ttl=30))
#         blink.value = False
    mqtt_client.loop()
#     mqtt_client.disconnect()
    time.sleep(.1)  # debounce delay
