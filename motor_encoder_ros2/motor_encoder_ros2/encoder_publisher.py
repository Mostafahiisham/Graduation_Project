import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from collections import deque
from std_msgs.msg import Float32MultiArray

#Motor Monitoring Class
class MotorMonitor :
    def __init__(self,a_pin,b_pin):
        
        self.a_pin = a_pin
        self.b_pin = b_pin
        self.count = 0
        self.last_count = 0
        self.rpm_history = deque(maxlen=WINDOW_SIZE)
    
        #Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(a_pin,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(b_pin,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    
        #Callback on rising edge of A channel
        #self.cb = pi.callback(a_pin, pigpio.RISING_EDGE, self._pulse_callback)
		GPIO.add_event_detect(a_pin, GPIO.RISING, callback=self._pulse_callback)
    
    def _pulse_callback(self,gpio,level,tick):
        #Read B channel to determine direction
        #b_state = self.pi.read(self.b_pin)
		b_state = GPIO.input(self.b_pin)
        self.count += 1 if b_state else -1
    
    def update_rpm(self,PPR,SAMPLE_RATE):
        #Calculate RPM
        delta =self.count - self.last_count
        rpm = (delta /PPR) * (60 * SAMPLE_RATE)
        self.last_count = self.count
        self.rpm_history.append(rpm)
        return sum(self.rpm_history)/len(self.rpm_history)
    
    
    def cleanup(self):
        #self.cb.cancel()
		GPIO.remove_event_detect(self.a_pin)
        
class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_rpms', 10)

        # Parameters
        self.declare_parameter('PPR', 500)
        self.declare_parameter('SAMPLE_RATE', 50.0)
        
        self.PPR = self.get_parameter('PPR').value
        self.SAMPLE_RATE = self.get_parameter('SAMPLE_RATE').value
        
        #Motor GPIO assignments
        self.MOTORS = {
        1:{'A':17,'B':27 },
        2:{'A':5,'B':6},
        3:{'A':23, 'B':24},
        4:{'A':25, 'B':8},
        }
        # 17       #Channel A-Motor1
        # 27       #Channel B-Motor1
        # 5        #Channel A-Motor2
        # 6        #Channel B-motor2
        # 23       #Channel A-Motor3
        # 24       #Channel B-Motor3
        # 25       #Channel A-Motor4
        # 8        #Channel B-Motor4
        
        #self.pi = pi()
        #if not self.pi.connected:
        #    self.get_logger().error("Failed to connect to pigpio daemon")
        #    raise RuntimeError("Pigpio connection failed")
        
        
        #self.motors = {id: MotorMonitor(self.pi, pins['A'], pins['B']) 
        #              for id, pins in self.MOTORS.items()}
        try:
            self.motors = {id: MotorMonitor(pins['A'], pins['B']) 
                          for id, pins in self.MOTORS.items()}
        except Exception as e:
            self.get_logger().error(f"GPIO initialization failed: {str(e)}")
            raise RuntimeError("GPIO initialization failed")
        
        timer_period = 1.0 / self.SAMPLE_RATE
        self.timer = self.create_timer(timer_period, self.timer_callback)          
		
		
        timer_period = 1.0 / self.SAMPLE_RATE
        self.timer = self.create_timer(timer_period, self.timer_callback)      
    
    
    def timer_callback(self):
        rpm_readings = [motor.update_rpm(self.PPR, self.SAMPLE_RATE) 
                       for motor in self.motors.values()]
 
        msg = Float32MultiArray()
        msg.data = rpm_readings
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing: {msg.data}')       

    def destroy_node(self):
        for motor in self.motors.values():
            motor.cleanup()
        GPIO.cleanup()
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    encoder_publisher = EncoderPublisher()
    
    try:
        rclpy.spin(encoder_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        encoder_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()