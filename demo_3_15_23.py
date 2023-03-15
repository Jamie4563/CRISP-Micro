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
    
shld = Pin(15, Pin.OUT)
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
motor_PWM_pins = [21,3,1,2]

data_pin=Pin(data_pin, Pin.OUT)
latch_pin=Pin(latch_pin, Pin.OUT)
clock_pin=Pin(clock_pin, Pin.OUT)

#adc = ADC(Pin())
led = Pin(25, Pin.OUT)

motors = list()
for pin in motor_PWM_pins:
    motors.append(Motor(pin));
    
n = len(motor_PWM_pins)
num_piso = 1

clk.value(0)

for motor in motors:
    motor.change_speed(0.25)
    
#motors[0].change_speed(0.25)
#motors[1].change_speed(0.25)
#motors[2].change_speed(0.23)
#motors[3].change_speed(0.27)

#directions = [STOP,CW,CCW,STOP]
directions = [STOP]*n

bits = "0"*n
shift_out(bits,data_pin,clock_pin,latch_pin)

counts = [0]*n
detected = [True]*n

procedure = 1 #SET THIS NUMBER EQUAL TO EITHER 1 OR 2

if procedure == 1:
    # ROTATE object in place counterclockwise (spin cubes 720 degrees--i.e., 2 full revs--then back)
    #procedure = ["10101010","01010101"]
    procedure = ["01010101","10101010"]
    max_count = 8
elif procedure == 2:
    # MOVE object counterclockwise around grid perimeter
    #procedure = ["10000100","00100001","01001000","00010010"]
    procedure = ["01001000","00010010","10000100","00100001"]
    max_count = 2
else:
    procedure = ["00000000"]
    max_count = 1

for instruction in procedure:
    #print(instruction)
    counts = [0]*n
    for i in range(0,len(instruction),2):
        if instruction[i:i+2] != STOP: counts[int(i/2)] = max_count
    detected = [True]*n
    while counts != [0]*n:
        #print(counts)
        load_parallel()
        data = shift_in(8*num_piso)
        #print(bin(data))
        for i in range(len(motors)):
            if check_bit(data,i):
                detected[i] = False
            else:
                if not detected[i]:
                    detected[i] = True
                    if counts[i] > 0: counts[i] -= 1
        bits = ""
        for i in range(n):
            if counts[i]:
                bits += instruction[2*i:2*i+2]
            else:
                bits += STOP
        shift_out(bits,data_pin,clock_pin,latch_pin)

bits = "0"*n
shift_out(bits,data_pin,clock_pin,latch_pin)
led.value(0)