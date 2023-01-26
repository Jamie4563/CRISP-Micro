from machine import Pin, PWM
import utime
import _thread

STOP = "00"
CCW = "01"
CW = "10"
SHORT_BRAKE = "11"

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

def send_data(clock_pin, data_pin, data):
    clock_pin.value(0)
    data_pin.value(data)
    clock_pin.value(1)
    
def shift_update(bits, data, clock, latch):
  send_data(clock, latch, 0)
  for bit in reversed(bits):
    send_data(clock, data, int(bit))
  send_data(clock, latch, 1)
  
def check_bit(data, n):
    bitmask = 1 << n
    return data & bitmask
    
shld = Pin(15, Pin.OUT)
clk = Pin(14, Pin.OUT)
qh = Pin(22, Pin.IN)
delay = 5e-6

def load_parallel():
    clk_inh.value(1)
    shld.value(0) # load A-H pins to registers
    shld.value(1) # latch register values
    
def shift_out(n):
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

data_pin=Pin(data_pin, Pin.OUT)
latch_pin=Pin(latch_pin, Pin.OUT)
clock_pin=Pin(clock_pin, Pin.OUT)
led = Pin(25, Pin.OUT)

motors = [Motor(21)]

clk.value(0)

motor.change_speed(0.5)
direction = STOP

while True:
    load_parallel()
    data = shift_in(8)
    bits = ""
    for i in range(len(motors)):
        if check_bit(data, i):
            bits += SHORT_BRAKE
        else:
            bits += direction
    shift_update(bits,data_pin,clock_pin,latch_pin)