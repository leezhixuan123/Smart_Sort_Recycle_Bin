import RPi.GPIO as GPIO
import time
import libcamera
import Adafruit_DHT
import json
import paho.mqtt.client as mqtt
from picamera2 import Picamera2
from PIL import Image
from classification import classify_image
import numpy as np

# Pin configuration
CAMERA_IR_SENSOR_PIN = 14  # GPIO pin connected to the camera-use IR sensor's OUT pin
BIN_PAPER_IR_SENSOR_PIN = 20  # GPIO pin connected to the paper bin IR sensor
BIN_PLASTIC_IR_SENSOR_PIN = 16  # GPIO pin connected to the plastic bin IR sensor
BIN_METAL_IR_SENSOR_PIN = 17  # GPIO pin connected to the metal bin IR sensor
BIN_TRASH_IR_SENSOR_PIN = 27  # GPIO pin connected to the trash bin IR sensor
DHT_SENSOR_PIN = 15  # GPIO pin connected to the DHT11 sensor
SERVO_PIN_PAPER = 12  # GPIO pin connected to the servo signal
SERVO_PIN_PLASTIC = 13  # GPIO pin connected to the servo signal
SERVO_PIN_METAL = 18  # GPIO pin connected to the servo signal
SERVO_PIN_TRASH = 21  # GPIO pin connected to the servo signal
RED_LED_PIN = 19  # GPIO pin for red LED (paper bin full)
YELLOW_LED_PIN = 22  # GPIO pin for yellow LED (plastic bin full)
GREEN_LED_PIN = 26  # GPIO pin for green LED (metal bin full)
RED_LED_TRASH_PIN = 23  # GPIO pin for second red LED (trash bin full)

# ThingsBoard configuration
THINGSBOARD_HOST = "demo.thingsboard.io"
ACCESS_TOKEN = "dR9THgVKZUAOpnOpgKxp"

# Initialize MQTT client
client = mqtt.Client()
client.username_pw_set(ACCESS_TOKEN)
client.connect(THINGSBOARD_HOST, 1883, 60)
client.loop_start()

# Initialize Picamera2
camera = Picamera2()
camera.start()

# Set up GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(CAMERA_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(BIN_PAPER_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(BIN_PLASTIC_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(BIN_METAL_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(BIN_TRASH_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN_PAPER, GPIO.OUT)
GPIO.setup(SERVO_PIN_PLASTIC, GPIO.OUT)
GPIO.setup(SERVO_PIN_METAL, GPIO.OUT)
GPIO.setup(SERVO_PIN_TRASH, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(YELLOW_LED_PIN, GPIO.OUT) 
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_TRASH_PIN, GPIO.OUT)

# DHT11 Sensor setup
DHT_SENSOR = Adafruit_DHT.DHT11

# Servo setup
servo_paper_pwm = GPIO.PWM(SERVO_PIN_PAPER, 50)  # Set servo frequency
servo_plastic_pwm = GPIO.PWM(SERVO_PIN_PLASTIC, 50)
servo_metal_pwm = GPIO.PWM(SERVO_PIN_METAL, 50)
servo_trash_pwm = GPIO.PWM(SERVO_PIN_TRASH, 50)

# Start all servos with 0% duty cycle
servo_paper_pwm.start(0)
servo_plastic_pwm.start(0)
servo_metal_pwm.start(0)
servo_trash_pwm.start(0)

# Variables for timing
last_capture_time = 0  # Tracks the last capture time for the object
detection_start_time = None  # Tracks when detection starts for the object
paper_bin_detection_start_time = None  # Tracks when the paper bin IR sensor detects full
plastic_bin_detection_start_time = None  # Tracks when the plastic bin IR sensor detects full
metal_bin_detection_start_time = None  # Tracks when the metal bin IR sensor detects full
trash_bin_detection_start_time = None  # Tracks when the trash bin IR sensor detects full
DETECTION_DURATION = 1  # Time (seconds) the object must be detected
COOLDOWN_PERIOD = 5  # Time (seconds) to wait before next capture
BIN_FULL_THRESHOLD = 5  # Time (seconds) the bin-use IR sensor must be active to consider the bin full

# Variables for bin full/not full status
paper_bin_full_sent = None  # Tracks the last sent paper bin full status
plastic_bin_full_sent = None  # Tracks the last sent plastic bin full status
metal_bin_full_sent = None  # Tracks the last sent metal bin full status
trash_bin_full_sent = None  # Tracks the last sent trash bin full status

# Averaging parameters for Temp/Humidity data processing 
num_readings = 10 
temperature_readings = []
humidity_readings = []

def get_average_reading(readings, new_reading):
    if len(readings) >= num_readings:
        readings.pop(0)  # Remove the oldest reading
    readings.append(new_reading)
    return np.mean(readings)  


def set_servo_angle(servo_pwm, pin, angle):
    duty_cycle = 2 + (angle / 18)  # Convert angle (0-180) to duty cycle (2-12)
    GPIO.output(pin, True)
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(pin, False)
    servo_pwm.ChangeDutyCycle(0)

def read_dht11():
    humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_SENSOR_PIN)
    if humidity is not None and temperature is not None:
        return humidity, temperature
    else:
        return None, None


def handle_bin_detection(current_time):
    global paper_bin_detection_start_time, plastic_bin_detection_start_time, metal_bin_detection_start_time, trash_bin_detection_start_time
    global paper_bin_full_sent, plastic_bin_full_sent, metal_bin_full_sent, trash_bin_full_sent
    
    # Read bin IR sensors
    paper_bin_sensor = GPIO.input(BIN_PAPER_IR_SENSOR_PIN)
    plastic_bin_sensor = GPIO.input(BIN_PLASTIC_IR_SENSOR_PIN)
    metal_bin_sensor = GPIO.input(BIN_METAL_IR_SENSOR_PIN)
    trash_bin_sensor = GPIO.input(BIN_TRASH_IR_SENSOR_PIN)
    
    # Handle paper bin detection
    if paper_bin_sensor == 0:  # Bin full detected
        if paper_bin_detection_start_time is None:
            paper_bin_detection_start_time = current_time
        elif current_time - paper_bin_detection_start_time >= BIN_FULL_THRESHOLD:
            if paper_bin_full_sent != 1:  # Send telemetry if not already sent
                client.publish("v1/devices/me/telemetry", json.dumps({"paperFull": 1}), qos=1)
                print("Paper bin is full! Sent telemetry: paperFull: 1")
                GPIO.output(RED_LED_PIN, GPIO.HIGH)
                paper_bin_full_sent = 1
    else:  # Bin not full
        if paper_bin_detection_start_time is not None:
            if current_time - paper_bin_detection_start_time >= BIN_FULL_THRESHOLD and paper_bin_full_sent != 0:
                client.publish("v1/devices/me/telemetry", json.dumps({"paperFull": 0}), qos=1)
                print("Paper bin is not full! Sent telemetry: paperFull: 0")
                GPIO.output(RED_LED_PIN, GPIO.LOW)
                paper_bin_full_sent = 0
        paper_bin_detection_start_time = None  # Reset detection start time
    
    # Handle plastic bin detection
    if plastic_bin_sensor == 0:  # Bin full detected
        if plastic_bin_detection_start_time is None:
            plastic_bin_detection_start_time = current_time
        elif current_time - plastic_bin_detection_start_time >= BIN_FULL_THRESHOLD:
            if plastic_bin_full_sent != 1:  # Send telemetry if not already sent
                client.publish("v1/devices/me/telemetry", json.dumps({"plasticFull": 1}), qos=1)
                print("Plastic bin is full! Sent telemetry: plasticFull: 1")
                GPIO.output(YELLOW_LED_PIN, GPIO.HIGH)
                plastic_bin_full_sent = 1
    else:  # Bin not full
        if plastic_bin_detection_start_time is not None:
            if current_time - plastic_bin_detection_start_time >= BIN_FULL_THRESHOLD and plastic_bin_full_sent != 0:
                client.publish("v1/devices/me/telemetry", json.dumps({"plasticFull": 0}), qos=1)
                print("Plastic bin is not full! Sent telemetry: plasticFull: 0")
                GPIO.output(YELLOW_LED_PIN, GPIO.LOW)
                plastic_bin_full_sent = 0
        plastic_bin_detection_start_time = None  # Reset detection start time
    
    # Handle metal bin detection
    if metal_bin_sensor == 0:  # Bin full detected
        if metal_bin_detection_start_time is None:
            metal_bin_detection_start_time = current_time
        elif current_time - metal_bin_detection_start_time >= BIN_FULL_THRESHOLD:
            if metal_bin_full_sent != 1:  # Send telemetry if not already sent
                client.publish("v1/devices/me/telemetry", json.dumps({"metalFull": 1}), qos=1)
                print("Metal bin is full! Sent telemetry: metalFull: 1")
                GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
                metal_bin_full_sent = 1
    else:  # Bin not full
        if metal_bin_detection_start_time is not None:
            if current_time - metal_bin_detection_start_time >= BIN_FULL_THRESHOLD and metal_bin_full_sent != 0:
                client.publish("v1/devices/me/telemetry", json.dumps({"metalFull": 0}), qos=1)
                print("Metal bin is not full! Sent telemetry: metalFull: 0")
                GPIO.output(GREEN_LED_PIN, GPIO.LOW)
                metal_bin_full_sent = 0
        metal_bin_detection_start_time = None  # Reset detection start time
    
    # Handle trash bin detection
    if trash_bin_sensor == 0:  # Bin full detected
        if trash_bin_detection_start_time is None:
            trash_bin_detection_start_time = current_time
        elif current_time - trash_bin_detection_start_time >= BIN_FULL_THRESHOLD:
            if trash_bin_full_sent != 1:  # Send telemetry if not already sent
                client.publish("v1/devices/me/telemetry", json.dumps({"trashFull": 1}), qos=1)
                print("Trash bin is full! Sent telemetry: trashFull: 1")
                GPIO.output(RED_LED_TRASH_PIN, GPIO.HIGH)
                trash_bin_full_sent = 1
    else:  # Bin not full
        if trash_bin_detection_start_time is not None:
            if current_time - trash_bin_detection_start_time >= BIN_FULL_THRESHOLD and trash_bin_full_sent != 0:
                client.publish("v1/devices/me/telemetry", json.dumps({"trashFull": 0}), qos=1)
                print("Trash bin is not full! Sent telemetry: trashFull: 0")
                GPIO.output(RED_LED_TRASH_PIN, GPIO.LOW)
                trash_bin_full_sent = 0
        trash_bin_detection_start_time = None  # Reset detection start time


def handle_object_detection(camera_sensor_value, current_time):
    """Handles object detection logic."""
    global detection_start_time, last_capture_time
    if camera_sensor_value == 0:  # Object detected
        if detection_start_time is None:
            detection_start_time = current_time  
        elif current_time - detection_start_time >= DETECTION_DURATION:
            if current_time - last_capture_time >= COOLDOWN_PERIOD:
                capture_and_classify_object(current_time)
                detection_start_time = None  # Reset detection timer
    else:
        detection_start_time = None  # Reset detection if no object is detected


def capture_and_classify_object(current_time):
    filename = f"object.jpg"
    print(f"Object detected! Capturing image: {filename}")
    camera.capture_file(filename)
    original_img = Image.open(filename)
    oriented_img = original_img.transpose(method=Image.FLIP_TOP_BOTTOM).transpose(method=Image.FLIP_LEFT_RIGHT) # orient the image correctly
    oriented_img.save(filename)

    classification_result = classify_image(filename)
    print(f"Classification result: {classification_result}")

    # Send telemetry data
    telemetry_data = {
        "paper": 1 if classification_result == "paper" else 0,
        "plastic": 1 if classification_result == "plastic" else 0,
        "metal": 1 if classification_result == "metal" else 0,
        "trash": 1 if classification_result == "trash" else 0
    }
    client.publish("v1/devices/me/telemetry", json.dumps(telemetry_data), qos=1)
    print(f"Classification data sent to ThingsBoard: {telemetry_data}")

    # Control servos based on classification result
    if classification_result == "paper":
        print("Moving servo to position for paper")
        set_servo_angle(servo_paper_pwm, SERVO_PIN_PAPER, 180)
        time.sleep(2)
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 0)
        time.sleep(2)
        set_servo_angle(servo_paper_pwm, SERVO_PIN_PAPER, 90)
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 90)
    elif classification_result == "plastic":
        print("Moving servo to position for plastic")
        set_servo_angle(servo_plastic_pwm, SERVO_PIN_PLASTIC, 180)
        time.sleep(2)
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 0)
        time.sleep(2)
        set_servo_angle(servo_plastic_pwm, SERVO_PIN_PLASTIC, 90)
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 90)
    elif classification_result == "metal":
        print("Moving servo to position for metal")
        set_servo_angle(servo_metal_pwm, SERVO_PIN_METAL, 180)
        time.sleep(2)
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 0)
        time.sleep(2)
        set_servo_angle(servo_metal_pwm, SERVO_PIN_METAL, 90)
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 90)
    else:  # Assume trash
        print("Moving servo to position for trash")
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 180)
        time.sleep(2)
        set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 90)

    last_capture_time = current_time

# Component state Initialisation
set_servo_angle(servo_paper_pwm, SERVO_PIN_PAPER, 90)
set_servo_angle(servo_plastic_pwm, SERVO_PIN_PLASTIC, 90)
set_servo_angle(servo_metal_pwm, SERVO_PIN_METAL, 90)
set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 90)
GPIO.output(RED_LED_PIN, GPIO.LOW)
GPIO.output(YELLOW_LED_PIN, GPIO.LOW)
GPIO.output(GREEN_LED_PIN, GPIO.LOW)
GPIO.output(RED_LED_TRASH_PIN, GPIO.LOW)

def main():
    print("Monitoring IR sensors and DHT11 sensor...")
    while True:
        camera_sensor_value = GPIO.input(CAMERA_IR_SENSOR_PIN)
        current_time = time.time()

        handle_bin_detection(current_time)
        handle_object_detection(camera_sensor_value, current_time)
        
        # Read DHT11 sensor
        humidity, temperature = read_dht11()
        if humidity is not None and temperature is not None:
            avg_humidity = get_average_reading(humidity_readings, humidity)
            avg_temperature = get_average_reading(temperature_readings, temperature)
            telemetry_data = {
                "temperature": avg_temperature,
                "humidity": avg_humidity,
            }
            # Send data to ThingsBoard
            client.publish("v1/devices/me/telemetry", json.dumps(telemetry_data), qos=1)
            print(f"Temp/Humidity data sent to ThingsBoard: {telemetry_data}")
        else:
            print("Failed to read DHT11 sensor")
        
        time.sleep(1)  # Adjust as needed

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting program...")

    finally:
        client.loop_stop()
        client.disconnect()
        servo_paper_pwm.stop()  # Stop PWM
        servo_plastic_pwm.stop()
        servo_metal_pwm.stop()
        servo_trash_pwm.stop()
        camera.stop_preview()  # Stop preview if used
        GPIO.output(RED_LED_PIN, GPIO.LOW) # Turn off red LED for paper
        GPIO.output(YELLOW_LED_PIN, GPIO.LOW) # Turn off yellow LED
        GPIO.output(GREEN_LED_PIN, GPIO.LOW) # Turn off green LED
        GPIO.output(RED_LED_TRASH_PIN, GPIO.LOW) # Turn off red LED for trash
        camera.close()  # Clean up the camera
        GPIO.cleanup()  # Clean up GPIO settings
