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

data_pin=Pin(data_pin, Pin.OUT) # data input pin for SIPO shift register
latch_pin=Pin(latch_pin, Pin.OUT) # latch pin for SIPO shift register
clock_pin=Pin(clock_pin, Pin.OUT) # clock pin for SIPO shift register
shld = Pin(15, Pin.OUT) # shift load pin for PISO shift register
clk = Pin(14, Pin.OUT) # clock pin for PISO shift register
encoder = Pin(22, Pin.IN) # data output pin for PISO shift register which manages the encoders 
led = Pin(25, Pin.OUT) # status LED
delay = 5e-6 # 5 microsecond delay between changing signals to the PISO shift register (timing according to datasheet)

motor = Motor(21) # motor constructed on PWM pin 21

clk.value(0)

motor.change_speed(0.5)
direction = SHORT_BRAKE # direction set to global constant for the "short brake" motor state, see top of program

steps = 100 # steps of the loop before encoder checking starts,
# this is to override the encoder so that the voxel can rotate no matter what when the program begins
stoptimer = 0 # the stop timer is set when the encoder trips and is decremented on each loop, 
# thus preventing a momentary change in encoder signal from restarting the voxel
t = 0
while True:
    #print(t)
    outstring = "" # bit string received from PISO shift register
    bits = [0]*32 # bit list to send to SIPO shift register chain, 32 long to allow for up to 16 motors
    led.value(0)
    shld.value(0)
    utime.sleep(delay)
    shld.value(1)
    utime.sleep(delay)
    clk.value(1)
    for i in range(8): # shifting in 8 bits from the PISO
        clk.value(0)
        utime.sleep(delay)
        clk.value(1)
        outstring += str(encoder.value())
        utime.sleep(0.001)
    clk.value(0)
    if outstring[5] == '0': # checking the specific bit that corresponds to the currently wired sensor
        if t>=steps:
            stoptimer = 10
        led.value(1)
        #print('Magnet detected')
    if stoptimer > 0: # brakes the motor if stop timer is set
        stoptimer -= 1
        bits[0] = int(SHORT_BRAKE[0])
        bits[1] = int(SHORT_BRAKE[1])
    else: # starts the motor if stop timer is not set
        bits[0] = int(direction[0])
        bits[1] = int(direction[1])
    #print(stoptimer)
    shift_update(bits,data_pin,clock_pin,latch_pin)
    #print(outstring)
    if t<steps:
        t += 1
    #utime.sleep(0.001)
