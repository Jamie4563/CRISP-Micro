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

data_pin = 28
latch_pin = 27
clock_pin = 26

data_pin=Pin(data_pin, Pin.OUT)
latch_pin=Pin(latch_pin, Pin.OUT)
clock_pin=Pin(clock_pin, Pin.OUT)
shld = Pin(15, Pin.OUT)
clk = Pin(14, Pin.OUT)
encoder = Pin(22, Pin.IN)
led = Pin(25, Pin.OUT)
delay = 5e-6

motor = Motor(21)

clk.value(0)
motor.change_speed(0.5)

steps = 100
t = 0
while True:
    #print(t)
    outstring = ""
    bits = [0]*32
    bits[1] = 1
    shld.value(0)
    utime.sleep(delay)
    shld.value(1)
    utime.sleep(delay)
    clk.value(1)
    for i in range(8):
        clk.value(0)
        utime.sleep(delay)
        clk.value(1)
        outstring += str(encoder.value())
        utime.sleep(0.001)
    clk.value(0)
    if outstring[5] == '0':
        if t>=steps:
            bits[0] = 1
        led.value(1)
        #print('Magnet detected')
    else:
        bits[0] = 0
        led.value(0)
    shift_update(bits,data_pin,clock_pin,latch_pin)
    #print(outstring)
    if t<steps:
        t += 1
    #utime.sleep(0.001)
