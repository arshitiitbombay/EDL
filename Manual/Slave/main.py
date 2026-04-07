import rp2
import machine
import time
from i2c_responder import I2CResponder 

# ==========================================
# PIO ENCODER CLASS
# ==========================================
@rp2.asm_pio()
def quadrature_encoder():
    wrap_target()
    wait(1, pin, 0)      
    in_(pins, 2)         
    push(noblock)        
    wait(0, pin, 0)      
    wrap()

class PIOEncoder:
    def __init__(self, sm_id, pin_a, pin_b):
        self.sm = rp2.StateMachine(sm_id, quadrature_encoder, in_base=pin_a)
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks = 0
        self.sm.active(1)

    def read(self):
        while self.sm.rx_fifo():
            state = self.sm.get() & 0x03 
            if state == 1:       
                self.ticks += 1  
            elif state == 3:     
                self.ticks -= 1  
        return abs(self.ticks)
        
    def reset(self):
        # FLUSH THE HARDWARE: Drain any ghost ticks left in the FIFO!
        while self.sm.rx_fifo():
            self.sm.get()
        self.ticks = 0

# ==========================================
# SYSTEM SETUP
# ==========================================
NUM_MOTORS = 8

print("Initializing PIO Encoders...")
encoders = [
    PIOEncoder(0, machine.Pin(0), machine.Pin(1)),
    PIOEncoder(1, machine.Pin(2), machine.Pin(3)),
    PIOEncoder(2, machine.Pin(8), machine.Pin(9)),
    PIOEncoder(3, machine.Pin(10), machine.Pin(11)),
    PIOEncoder(4, machine.Pin(12), machine.Pin(13)),
    PIOEncoder(5, machine.Pin(14), machine.Pin(15)),
    PIOEncoder(6, machine.Pin(20), machine.Pin(21)),
    PIOEncoder(7, machine.Pin(18), machine.Pin(19))
]

i2c_target = I2CResponder(i2c_device_id=0, sda_gpio=4, scl_gpio=5, responder_address=0x50)

print("SLAVE READY (PIO Native Mode).")

incoming_buffer = []

while True:
    # --- 1. DRAIN PIO FIFOS CONSTANTLY ---
    # Running this every cycle without sleeping prevents dropped ticks!
    for m in range(NUM_MOTORS):
        encoders[m].read()

    # --- 2. HANDLE MASTER COMMANDS ---
    if i2c_target.write_data_is_available():
        incoming_buffer.extend(i2c_target.get_write_data(max_size=2))
        
    if len(incoming_buffer) >= 2:
        cmd = incoming_buffer[0]
        m = incoming_buffer[1]
        
        if cmd == 0x01 and 0 <= m < NUM_MOTORS:
            # Trigger the deep flush and reset!
            encoders[m].reset() 
            
        incoming_buffer = incoming_buffer[2:] # (not wipe)

    # --- 3. SEND TELEMETRY PACKET ---
    if i2c_target.read_is_pending():
        data_packet = bytearray(16) #changed 17 to 16
        #data_packet[0] = 0x00 
        
        for m in range(NUM_MOTORS):
            # Grab the current absolute value
            current_ticks = abs(encoders[m].ticks)
            data_packet[(m*2)] = current_ticks & 0xFF #was 1 + (m*2)
            data_packet[1 + (m*2)] = (current_ticks >> 8) & 0xFF  #was 2 + (m*2)
            
        for byte in data_packet:
            i2c_target.put_read_data(byte)
