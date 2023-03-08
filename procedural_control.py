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
n = len(motor_PWM_pins)

data_pin=Pin(data_pin, Pin.OUT)
latch_pin=Pin(latch_pin, Pin.OUT)
clock_pin=Pin(clock_pin, Pin.OUT)

led = Pin(25, Pin.OUT)

motors = list()
for pin in motor_PWM_pins:
    motors.append(Motor(pin));

clk.value(0)

for motor in motors:
    motor.change_speed(0.3)
    

bits = "0"*n
shift_out(bits,data_pin,clock_pin,latch_pin)

max_count = 4
id_map = {
    "A1": 0, "A2": 1, "A3": 2, "A4": 3,
    "B1": 4, "B2": 5, "B3": 6, "B4": 7,
    "C1": 8, "C2": 9, "C3": 10, "C4": 11,
    "D1": 12, "D2": 13, "D3": 14, "D4": 15
}
procedure = list()

if input("Load procedure? y/n") == "y":
    procedure = input("Paste procedure here")
    procedure = procedure.strip()
    procedure = procedure.split(',')
else:
    while True:
        ccw_cells = input("""Enter the cell ID (e.g. B3) for each cell you would like to rotate counter-clockwise in a list separated by whitespace""")
        cw_cells = input("""Enter the cell ID (e.g. B3) for each cell you would like to rotate clockwise in a list separated by whitespace""")
        bit_array = [STOP]*n
        ccw_cells = ccw_cells.split()
        cw_cells = cw_cells.split()
        for id in ccw_cells:
            bit_array[id_map[id]] = CCW
        for id in cw_cells:
            bit_array[id_map[id]] = CW
        procedure.append("".join(bit_array))
        if input("Continue adding instructions? y/n") == "n": break
        
for instruction in procedure:
    counts = [0]*n
    for i in range(0,len(instruction),2):
        if instruction[i:i+2] != STOP: counts[i/2] = max_count
    detected = [True]*n
    while counts != [0]*n:
        load_parallel()
        data = shift_in(n)
        for i in range(len(motors)):
            if check_bit(data,i):
                detected[i] = False
            else:
                if not detected[i]:
                    detected[i] = True
                    if counts > 0: counts[i] -= 1
        bits = ""
        for i in range(n):
            if counts[i]:
                bits += instruction[2*i:2*i+2]
            else:
                bits += STOP
        shift_out(bits,data_pin,clock_pin,latch_pin)

bits = "0"*len(motor_PWM_pins)
shift_out(bits,data_pin,clock_pin,latch_pin)
led.value(0)
if input("Save procedure? y/n") == "y":
    print("Copy this to clipboard")
    savestring = ""
    for instruction in procedure:
        savestring+=instruction
        savestring+=','
    print(savestring[0:-1])
