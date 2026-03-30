from pca9685 import PCA9685

class FeatherDriver:
    # Adafruit hardwires specific PCA9685 pins to the TB6612 H-Bridges
    # Format: (PWM_Pin, IN1_Pin, IN2_Pin)
    MOTOR_PINS = [
        (8, 10, 9),   # Motor 1 (M1 on board)
        (13, 11, 12), # Motor 2 (M2 on board)
        (2, 4, 3),    # Motor 3 (M3 on board)
        (7, 5, 6)     # Motor 4 (M4 on board)
    ]

    def __init__(self, i2c_bus, addresses=[0x60, 0x61, 0x62]):  #OR 0x60, 0x61, 0x62
        self.boards = []
        for addr in addresses:
            board = PCA9685(i2c_bus, address=addr)
            board.freq(1600) # Standard 50Hz for motors
            self.boards.append(board)
            
        # Your specific power levels
        self.PWM_MAX = 4095
        self.PWM_MOVE = 500  #3000    # ~75% power
        self.PWM_TENSION = 800  # ~20% power
        self.PWM_HOLD = 0     # ~10% power stall torque

    def set_motor(self, global_motor_index, speed, direction_forward=True):
        """
        global_motor_index: 0 to 8 (spanning your 3 boards)
        speed: 0 to 4095
        """
        board_idx = global_motor_index // 4
        local_motor_idx = global_motor_index % 4
        
        if board_idx >= len(self.boards):
            return # Motor doesn't exist
            
        board = self.boards[board_idx]
        pwm_pin, in1_pin, in2_pin = self.MOTOR_PINS[local_motor_idx]
        
        # 1. Set the Speed
        board.duty(pwm_pin, speed)
        
        # 2. Set the Direction (The part I missed earlier!)
        if speed == 0:
            # Coast / Disconnect
            board.duty(in1_pin, 0)
            board.duty(in2_pin, 0)
        elif direction_forward:
            board.duty(in1_pin, 4095) # HIGH
            board.duty(in2_pin, 0)    # LOW
        else:
            board.duty(in1_pin, 0)    # LOW
            board.duty(in2_pin, 4095) # HIGH

    # --- Your Robot State Functions ---
    def state_move(self, motor_index, forward=True):
        self.set_motor(motor_index, self.PWM_MOVE, forward)

    def state_tension(self, motor_index, forward=True):
        self.set_motor(motor_index, self.PWM_TENSION, forward)

    def state_hold(self, motor_index, forward=True):
        # Applies stall torque in the current direction of tension
        self.set_motor(motor_index, self.PWM_HOLD, forward)

    def set_grasp(self, is_grasping):
        # Motor 9 (Index 8) is your 12V grabber
        self.set_motor(8, self.PWM_MAX if is_grasping else 0, True)