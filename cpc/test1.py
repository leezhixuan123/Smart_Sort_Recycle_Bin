import RPi.GPIO as GPIO
import time

# Pin configuration - same as in main.py
SERVO_PIN_PAPER = 12  # GPIO pin connected to the paper servo
SERVO_PIN_PLASTIC = 13  # GPIO pin connected to the plastic servo
SERVO_PIN_METAL = 18  # GPIO pin connected to the metal servo
SERVO_PIN_TRASH = 21  # GPIO pin connected to the trash servo

# Set up GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(SERVO_PIN_PAPER, GPIO.OUT)
GPIO.setup(SERVO_PIN_PLASTIC, GPIO.OUT)
GPIO.setup(SERVO_PIN_METAL, GPIO.OUT)
GPIO.setup(SERVO_PIN_TRASH, GPIO.OUT)

# Servo setup
servo_paper_pwm = GPIO.PWM(SERVO_PIN_PAPER, 50)  # Set servo frequency to 50Hz
servo_plastic_pwm = GPIO.PWM(SERVO_PIN_PLASTIC, 50)
servo_metal_pwm = GPIO.PWM(SERVO_PIN_METAL, 50)
servo_trash_pwm = GPIO.PWM(SERVO_PIN_TRASH, 50)

# Start all servos with 0% duty cycle
servo_paper_pwm.start(0)
servo_plastic_pwm.start(0)
servo_metal_pwm.start(0)
servo_trash_pwm.start(0)

def set_servo_angle(servo_pwm, pin, angle):
    """Set servo angle using PWM."""
    duty_cycle = 2 + (angle / 18)  # Convert angle (0-180) to duty cycle (2-12)
    GPIO.output(pin, True)
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(pin, False)
    servo_pwm.ChangeDutyCycle(0)

try:
    print("Setting all servos to 90 degrees...")
    
    # Set all servos to 90 degrees
    set_servo_angle(servo_paper_pwm, SERVO_PIN_PAPER, 90)
    print("Paper servo set to 90 degrees")
    
    set_servo_angle(servo_plastic_pwm, SERVO_PIN_PLASTIC, 90)
    print("Plastic servo set to 90 degrees")
    
    set_servo_angle(servo_metal_pwm, SERVO_PIN_METAL, 90)
    print("Metal servo set to 90 degrees")
    
    set_servo_angle(servo_trash_pwm, SERVO_PIN_TRASH, 90)
    print("Trash servo set to 90 degrees")
    
    print("All servos have been set to 90 degrees!")
    time.sleep(2)  # Keep the servos in position for 2 seconds

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Clean up
    servo_paper_pwm.stop()
    servo_plastic_pwm.stop()
    servo_metal_pwm.stop()
    servo_trash_pwm.stop()
    GPIO.cleanup()
    print("Cleanup complete") 