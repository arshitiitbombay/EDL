import machine

class MCP23S17:
    def __init__(self, spi, cs, address=1):
        self.spi = spi
        self.cs = cs
        
        # Shift the 3-bit address into the correct position (Bits 1, 2, 3)
        self.opcode = 0x40 | (address << 1)  

        self.cs.value(1) # Ensure chip is deselected to start
        
        # -------- INITIALIZATION --------
        # IOCON register = 0x0A (Enable Hardware Addressing)
        self.write_reg(0x0A, 0x08)  
        
        # CONFIGURE BOTH PORTS AS INPUTS (0xFF = 11111111)
        self.write_reg(0x00, 0xFF)  # IODIRA 
        self.write_reg(0x01, 0xFF)  # IODIRB 

        # ENABLE INTERNAL PULL-UPS ON BOTH PORTS
        self.write_reg(0x0C, 0xFF)  # GPPUA
        self.write_reg(0x0D, 0xFF)  # GPPUB

    # ---------- LOW LEVEL ----------
    def write_reg(self, reg, val):
        self.cs.value(0)
        # Send 3 bytes: Write Opcode, Register Address, Data Value
        self.spi.write(bytearray([self.opcode, reg, val]))
        self.cs.value(1)

    def read_reg(self, reg):
        self.cs.value(0)
        # Send 2 bytes: Read Opcode (opcode | 1), Register Address
        self.spi.write(bytearray([self.opcode | 1, reg]))
        # Read the 1 byte response
        result = self.spi.read(1)
        self.cs.value(1)
        return result[0]

    # ---------- READ FUNCTIONS ----------
    def read_gpioa(self):
        return self.read_reg(0x12)

    def read_gpiob(self):
        return self.read_reg(0x13)

    def read_all(self):
        """Returns a single 16-bit integer representing all 16 pins"""
        a = self.read_gpioa()
        b = self.read_gpiob()
        # Shift Port B left by 8 bits, and combine with Port A
        return (b << 8) | a