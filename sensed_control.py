from machine import Pin, PWM, ADC
import utime

STOP = "00"
CCW = "01" #counter-clockwise
CW = "10" #clockwise
SHORT_BRAKE = "11" #the h-bridge is shorted internally, this makes the motor brake

class Motor:
    def __init__(self, pwm_pin):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.speed = 0.5 # speed from 0-1
        self.mode = STOP # STOP, CCW, CW, or SHORT_BRAKE
        self.update()
        
    def update(self):
        self.pwm.duty_u16(int(self.speed*65535))
    
    def change_mode(self, mode):
        self.mode = mode
    
    def change_speed(self, speed):
        self.speed = speed
        if self.speed > 1:
            self.speed = 1
        if self.speed < 0:
            self.speed = 0
        self.update()
    def change_duty(self, duty):
        self.pwm.duty_u16(duty)

def send_data(clock_pin, data_pin, data):
    clock_pin.value(0)
    data_pin.value(data)
    clock_pin.value(1)
    
def shift_out(bits, data, clock, latch):
  send_data(clock, latch, 0)
  for bit in reversed(bits):
    send_data(clock, data, int(bit))
  send_data(clock, latch, 1)
  
def check_bit(data, n):
    bitmask = 1 << n
    return data & bitmask
    
shld = Pin(13, Pin.OUT)
clk = Pin(14, Pin.OUT)
clk_inh = Pin(20, Pin.OUT)
qh = Pin(22, Pin.IN)
delay = 5e-6

def load_parallel():
    clk_inh.value(1)
    shld.value(0) # load A-H pins to registers
    shld.value(1) # latch register values
    
def shift_in(n):
    clk_inh.value(0)
    data = qh.value()
    
    for i in range(n-1):
        clk.value(1)
        utime.sleep(delay)
        clk.value(0)
        data = data << 1
        data = data + qh.value()
        
    return data

data_pin = 28
latch_pin = 27
clock_pin = 26
motor_PWM_pins = [21,19,18,17,0,1,2,3,4,5,6,7,8,9,10,11]

data_pin=Pin(data_pin, Pin.OUT)
latch_pin=Pin(latch_pin, Pin.OUT)
clock_pin=Pin(clock_pin, Pin.OUT)

#adc = ADC(Pin())
led = Pin(25, Pin.OUT)

motors = list()
for pin in motor_PWM_pins:
    motors.append(Motor(pin));

clk.value(0)

for motor in motors:
    motor.change_speed(0.3)
    
#motors[0].change_speed(0.25)
#motors[1].change_speed(0.25)
#motors[2].change_speed(0.23)
#motors[3].change_speed(0.27)

#directions = [STOP,CW,CCW,STOP]
directions = [CW]*len(motor_PWM_pins)

bits = "0"*len(motor_PWM_pins)
shift_out(bits,data_pin,clock_pin,latch_pin)

counts = [0]*len(motor_PWM_pins)
detected = [True]*len(motor_PWM_pins)
max_count = 4

while True:
    load_parallel()
    data = shift_in(16)
    bits = ""
    for i in range(len(motors)):
        if check_bit(data,i):
            bits += directions[i]
            detected[i] = False
        else:
            if not detected[i]:
                detected[i] = True
                counts[i] += 1
            if counts[i] >= max_count:
                bits += SHORT_BRAKE
            else:
                bits += directions[i]
            
    #duty = adc.read_u16()
    #for motor in motors:
    #    motor.change_duty(duty)
    shift_out(bits,data_pin,clock_pin,latch_pin)

bits = "0"*len(motor_PWM_pins)
shift_out(bits,data_pin,clock_pin,latch_pin)
led.value(0)