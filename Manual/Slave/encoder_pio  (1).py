import rp2
import machine

@rp2.asm_pio()
def quadrature_encoder():
    """State machine for reading A/B quadrature signals"""
    wrap_target()
    wait(1, pin, 0)      # Wait for Pin A (the base pin) to go HIGH
    in_(pins, 2)         # Instantly read the state of BOTH Pin A and Pin B
    push(noblock)        # Push that 2-bit state to the Python script
    wait(0, pin, 0)      # Wait for Pin A to go LOW before looping
    wrap()

class PIOEncoder:
    def __init__(self, sm_id, pin_a, pin_b):
        # in_base=pin_a tells the PIO that Pin A is bit 0, and Pin B is bit 1
        self.sm = rp2.StateMachine(sm_id, quadrature_encoder, in_base=pin_a)
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks = 0
        self.sm.active(1)

    def read(self):
        # Drain the hardware FIFO and calculate true distance
        while self.sm.rx_fifo():
            # Get the 2-bit state pushed by the PIO
            state = self.sm.get() & 0x03 
            
            # Bit 0 is Pin A (always 1 here). Bit 1 is Pin B.
            # By checking Pin B, we know the direction!
            if state == 1:       # 0b01 (Pin A is 1, Pin B is 0)
                self.ticks += 1  # Spooling Forward
            elif state == 3:     # 0b11 (Pin A is 1, Pin B is 1)
                self.ticks -= 1  # Being dragged Backward
                
        # Return the absolute value! 
        # This ensures the logic in main.py (current_ticks >= target_ticks) 
        # works flawlessly whether the Master commanded a Forward OR Backward move.
        return abs(self.ticks)