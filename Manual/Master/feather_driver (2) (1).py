from pca9685 import PCA9685

class FeatherDriver:
    MOTOR_PINS = [
        (8, 10, 9),   # Motor 1 (M1 on board)
        (13, 11, 12), # Motor 2 (M2 on board)
        (2, 4, 3),    # Motor 3 (M3 on board)
        (7, 5, 6)     # Motor 4 (M4 on board)
    ]

    def __init__(self, i2c_bus, addresses=[0x60, 0x61, 0x62]):
        self.boards = []
        for addr in addresses:
            board = PCA9685(i2c_bus, address=addr)
            board.freq(1600) 
            self.boards.append(board)
            
        self.PWM_MAX = 4095
        self.PWM_TENSION = 800  
        self.PWM_HOLD = 0       

    def set_motor(self, global_motor_index, speed, direction_forward=True):
        board_idx = global_motor_index // 4
        local_motor_idx = global_motor_index % 4
        
        if board_idx >= len(self.boards):
            return 
            
        board = self.boards[board_idx]
        pwm_pin, in1_pin, in2_pin = self.MOTOR_PINS[local_motor_idx]
        
        board.duty(pwm_pin, speed)
        
        if speed == 0:
            board.duty(in1_pin, 0)
            board.duty(in2_pin, 0)
        elif direction_forward:
            board.duty(in1_pin, 4095) 
            board.duty(in2_pin, 0)    
        else:
            board.duty(in1_pin, 0)    
            board.duty(in2_pin, 4095) 

    def state_move(self, motor_index, forward=True, speed=100):
        clamped_speed = max(0, min(100, speed))
        pwm_value = int((clamped_speed / 100.0) * 4095)
        self.set_motor(motor_index, pwm_value, forward)

    def state_tension(self, motor_index, forward=True):
        self.set_motor(motor_index, self.PWM_TENSION, forward)

    def state_hold(self, motor_index, forward=True):
        self.set_motor(motor_index, self.PWM_HOLD, forward)

    def set_grasp(self, is_grasping):
        self.set_motor(8, self.PWM_MAX if is_grasping else 0, True)
