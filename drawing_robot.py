from __future__ import annotations
from dynamixel_sdk import *
import time
from typing import Dict, Tuple
import math

###############################################
# Protocol 2.0 version for MX-series (P-mode)
###############################################

# Joint → Motor IDs
JOINT_MAP = {
    1: 1,   # prismatic
    2: 2,   # revolute 1
    3: 3,   # revolute 2
    5: 5,   # color switcher
}

# sign & offset for each joint
JOINT_SIGNS  = {1:+1, 2:+1, 3:+1, 5:+1}
JOINT_OFFSET = {1:0,  2:0,  3:0,  5:0}

# Limits in degrees
# Regulate joints 2,3,5 to 0-360. no regulation for prismatic joint
LIMITS = {
    1: (None, None),
    2: (0, 360),
    3: (0, 360),
    5: (None, None),
}

###############################################
# Port & Dynamixel Settings
###############################################

DEVICENAME = "COM13"
BAUDRATE   = 57600

# Control Table Protocol 2.0
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116   # 4-byte
ADDR_PRESENT_POSITION = 132   # 4-byte
ADDR_PROFILE_ACCEL    = 108   # 4-byte
ADDR_PROFILE_VELOCITY = 112   # 4-byte
ADDR_TORQUE_LIMIT     = 102   # 2-byte

MODE_POSITION = 3
MODE_EXTENDED_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
TICKS_MAX = 4095

PROFILE_ACCEL = 40
PROFILE_VELOC = 60
TORQUE_LIMIT_VAL = 1000

EPOS_REV_MAX = 200
MIN_TICK_5 = -EPOS_REV_MAX * TICKS_MAX
MAX_TICK_5 =  EPOS_REV_MAX * TICKS_MAX
###############################################
# Utility
###############################################

def wrap_deg(d):
    d = d % 360.0
    return d if d >= 0 else d + 360.0

def deg_to_ticks(d):
    return int(round(wrap_deg(d) / 360 * TICKS_MAX))

def ticks_to_deg(t):
    return (t % TICKS_MAX) / TICKS_MAX * 360.0


###############################################
# Arm Class (Protocol 2.0)
###############################################

class Arm5DOF:
    def __init__(self, port_name=DEVICENAME, baud=BAUDRATE):

        # Open port
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            raise RuntimeError("Cannot open port")
        if not self.port.setBaudRate(baud):
            raise RuntimeError("Cannot set baudrate")

        # Protocol 2.0
        self.pkt = PacketHandler(2.0)

        # Configure motors
        for j, dxid in JOINT_MAP.items():
            self.pkt.write1ByteTxRx(self.port, dxid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

            if j == 5:
                self.pkt.write1ByteTxRx(self.port, dxid, ADDR_OPERATING_MODE, MODE_EXTENDED_POSITION)
            else:
                self.pkt.write1ByteTxRx(self.port, dxid, ADDR_OPERATING_MODE, MODE_POSITION)
            self.pkt.write4ByteTxRx(self.port, dxid, ADDR_PROFILE_ACCEL,  PROFILE_ACCEL)
            self.pkt.write4ByteTxRx(self.port, dxid, ADDR_PROFILE_VELOCITY, PROFILE_VELOC)

            self.pkt.write2ByteTxRx(self.port, dxid, ADDR_TORQUE_LIMIT, TORQUE_LIMIT_VAL)
            self.pkt.write1ByteTxRx(self.port, dxid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    ###############################################
    # Movement API
    ###############################################

    def move(self, joint: int, deg: float, wait=False):
        if joint not in JOINT_MAP:
            raise ValueError("Invalid joint")

        # direction + offset
        deg = JOINT_SIGNS[joint] * deg + JOINT_OFFSET[joint]

        lo, hi = LIMITS[joint]

        # Apply lower limit if defined
        if lo is not None:
            deg = max(deg, lo)

        # Apply upper limit if defined
        if hi is not None:
            deg = min(deg, hi)

        # convert to ticks
        pos = deg_to_ticks(deg)
        dxid = JOINT_MAP[joint]

        # WRITE 4-BYTE GOAL POSITION
        self.pkt.write4ByteTxRx(self.port, dxid, ADDR_GOAL_POSITION, pos)

        if wait:
            self._wait_joint(dxid, deg)

    def move_all(self, commands: Dict[int, float], wait=False):
        """ dict: {joint → degree} """
        for j, d in commands.items():
            self.move(j, d, wait=False)

        if wait:
            for j, d in commands.items():
                dxid = JOINT_MAP[j]
                target = wrap_deg(JOINT_SIGNS[j] * d + JOINT_OFFSET[j])
                self._wait_joint(dxid, target)

    ###############################################
    # Sensor API
    ###############################################

    def read(self, joint):
        dxid = JOINT_MAP[joint]
        ticks,_,_ = self.pkt.read4ByteTxRx(self.port, dxid, ADDR_PRESENT_POSITION)
        return ticks_to_deg(ticks)

    ###############################################
    # Wait function for smooth motion
    ###############################################

    def _wait_joint(self, dxid, target_deg, tol=1.5):
        t0 = time.time()
        while time.time() - t0 < 3:
            ticks,_,_ = self.pkt.read4ByteTxRx(self.port, dxid, ADDR_PRESENT_POSITION)
            now = ticks_to_deg(ticks)
            if abs((now - target_deg) % 360) < tol:
                return
            time.sleep(0.01)

    ###############################################
    # Special joint functions
    ###############################################

    def prismatic(self, mm):
        deg = mm * 2  # TODO replace with real conversion
        self.move(1, deg, wait=True)

    def color_next(self):
        self.move(5, 180, wait=True)

    ###############################################
    # Close the port
    ###############################################

    def close(self):
        for dxid in JOINT_MAP.values():
            self.pkt.write1ByteTxRx(self.port, dxid, ADDR_TORQUE_ENABLE, 0)
        self.port.closePort()

    def angle_converter(self, joint: int, angle):
        if (joint == 2):
            angle -= 17
            return angle
        if (joint == 3):
            angle -=35
            return angle
        
    def turn_relative(self, joint: int, delta_deg: float, wait: bool = True):
        dxid = JOINT_MAP[joint]

        if joint in (1, 5):
            # 1 & 5：在 tick 空间做真正的相对多圈运动
            cur_ticks, comm_r, err_r = self.pkt.read4ByteTxRx(
                self.port, dxid, ADDR_PRESENT_POSITION
            )
            # print(f"[turn_relative] joint {joint} read cur_ticks={cur_ticks}, comm={comm_r}, err={err_r}")

            delta_ticks = int(delta_deg / 360.0 * TICKS_MAX)
            target_ticks = cur_ticks + delta_ticks

        
            if joint == 5:
                if target_ticks < MIN_TICK_5:
                    target_ticks = MIN_TICK_5
                if target_ticks > MAX_TICK_5:
                    target_ticks = MAX_TICK_5

            comm_w, err_w = self.pkt.write4ByteTxRx(
                self.port, dxid, ADDR_GOAL_POSITION, int(target_ticks)
            )
            # print(f"[turn_relative] joint {joint} write comm={comm_w}, err={err_w}, delta_ticks={delta_ticks}")

            if not wait:
                return

            t0 = time.time()
            while time.time() - t0 < 5:
                now_ticks, _, _ = self.pkt.read4ByteTxRx(self.port, dxid, ADDR_PRESENT_POSITION)
                if abs(now_ticks - target_ticks) < 10:
                    return
                time.sleep(0.05)

        else:
            # 2、3 继续用角度相对控制（单圈）
            current_deg = self.read(joint)
            target_deg = current_deg + delta_deg
            self.move(joint, target_deg, wait=wait)



    def hl(self, mode):
        if mode == 1: # to the highest point
            self.move(1, 111, wait=True)
        if mode == 0: # to the lowest point
            self.move(1, 160, wait=True)
    
    def home(self):
        self.hl(1)
        self.move(2, self.angle_converter(2, 180), wait=True)
        self.move(3, self.angle_converter(3, 180), wait=True)

    def switch_pen(self):
        self.turn_relative(5, 135, wait=True) 


    # Geometry Setting'
    L2 = 5.5
    L3 = 5.5
    SERVO2_MIN = 100
    SERVO2_MAX = 220
    SERVO3_MIN = 90
    SERVO3_MAX = 220

    def ik_xy(self, x: float, y: float) -> tuple[float, float]:
        L2, L3 = self.L2, self.L3

        r2 = x * x + y * y
        cos_t3 = (r2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
        #cos_t3 = max(-1.0, min(1.0, cos_t3))  # 数值保护

        candidates = []

        for sign in (+1.0, -1.0):  # elbow-down / elbow-up
            t3 = sign * math.acos(cos_t3)
            t2 = math.atan2(y, x) - math.atan2(
                L3 * math.sin(t3),
                L2 + L3 * math.cos(t3)
            )

            geom2 = math.degrees(t2)
            geom3 = math.degrees(t3)
            print(geom2)
            print(geom3)
            ok2 = (self.GEOM2_MIN <= geom2 <= self.GEOM2_MAX)
            ok3 = (self.GEOM3_MIN <= geom3 <= self.GEOM3_MAX)

            if ok2 and ok3:
                # 合法解：转换为舵机角，存起来
                servo2 = self.geom_to_servo(2, geom2)
                servo3 = self.geom_to_servo(3, geom3)
                candidates.append((servo2, servo3))

        if not candidates:
            raise ValueError(f"IK: point (x={x:.2f}, y={y:.2f}) unreachable in joint limits")

        mid2 = 0.5 * (self.SERVO2_MIN + self.SERVO2_MAX)
        mid3 = 0.5 * (self.SERVO3_MIN + self.SERVO3_MAX)

        def cost(sol):
            s2, s3 = sol
            return (s2 - mid2) ** 2 + (s3 - mid3) ** 2

        best = min(candidates, key=cost)
        return best  # (servo2, servo3)

    
    def move_xy(self, x: float, y: float, wait: bool = True):
        """
        用 IK 把末端移动到纸上 (x,y) [inch]。
        这里得到的是舵机角度，直接丢给 self.move(2/3, angle)。
        """
        servo2_deg, servo3_deg = self.ik_xy(x, y)
        self.move(2, servo2_deg, wait=False)
        self.move(3, servo3_deg, wait=wait)

    def draw_line_xy(self, x1: float, y1: float,
                           x2: float, y2: float,
                           steps: int = 40):

        self.move_xy(x1, y1, wait=True)
        for k in range(1, steps + 1):
            u = k / steps
            x = x1 * (1 - u) + x2 * u
            y = y1 * (1 - u) + y2 * u
            last = (k == steps)
            self.move_xy(x, y, wait=last)

    def draw_rectangle_4colors_2d(self,
                                  x0: float, y0: float,
                                  w: float, h: float,
                                  steps_per_edge: int = 40):

        p1 = (x0,       y0      )  
        p2 = (x0 + w,   y0      )  
        p3 = (x0 + w,   y0 + h  )  
        p4 = (x0,       y0 + h  )  

        edges = [
            (p1, p2),  
            (p2, p3),  
            (p3, p4), # 
            (p4, p1), 
        ]

        self.move_xy(*p1, wait=True)

        for i, (start, end) in enumerate(edges):
            if i > 0:
               
                self.switch_pen()

            x1, y1 = start
            x2, y2 = end
            self.draw_line_xy(x1, y1, x2, y2, steps=steps_per_edge)


    def servo_to_geom(self, joint: int, servo_deg: float) -> float:
   
        if joint == 2:
            return servo_deg + 17.0 
        if joint == 3:
            return servo_deg + 35.0  
        return servo_deg

    def geom_to_servo(self, joint: int, geom_deg: float) -> float:

        if joint == 2:
            return geom_deg - 17.0
        if joint == 3:
            return geom_deg - 35.0
        return geom_deg

    @property
    def GEOM2_MIN(self):
        return self.servo_to_geom(2, self.SERVO2_MIN)

    @property
    def GEOM2_MAX(self):
        return self.servo_to_geom(2, self.SERVO2_MAX)

    @property
    def GEOM3_MIN(self):
        return self.servo_to_geom(3, self.SERVO3_MIN)

    @property
    def GEOM3_MAX(self):
        return self.servo_to_geom(3, self.SERVO3_MAX)

  
    def fk_xy_from_servo(self, j2_servo: float, j3_servo: float) -> tuple[float, float]:
        
        th2_geom = math.radians(self.servo_to_geom(2, j2_servo))
        th3_geom = math.radians(self.servo_to_geom(3, j3_servo))

        x = self.L2 * math.cos(th2_geom) + self.L3 * math.cos(th2_geom + th3_geom)
        y = self.L2 * math.sin(th2_geom) + self.L3 * math.sin(th2_geom + th3_geom)
        return x, y
    
    def scribble(self, joint: int, joint1, times, st, end):
        for i in range(times):
            self.move(joint, self.angle_converter(joint, st), wait=True)
            self.move(joint1,self.angle_converter(joint1, st), wait=True)
            self.move(joint, self.angle_converter(joint, end), wait=True)
            self.move(joint1,self.angle_converter(joint, end), wait=False)
###############################################
# RUN TEST
###############################################

if __name__ == "__main__":
    arm = Arm5DOF()
    #arm.move(5, 145, wait=True)
    #arm.move(2,163, wait=True)
    # arm.home()

    # arm.draw_rectangle_4colors_2d(
    # x0 = 8.0,   
    # y0 = 2.0,   
    # w  = 4.0,   
    # h  = 3.0,  
    # steps_per_edge = 50
    # )
    #arm.move(1,60,wait=True)
    #arm.switch_pen()
    #arm.hl(1)
    #arm.hl(1)
    #arm.home(
    
    arm.home()
    arm.hl(0)
    arm.scribble(2,3,3, 130, 220)
    arm.home()
    arm.switch_pen()
    arm.hl(0)
    arm.scribble(2,3,3,150, 210)
    arm.home()
    #arm.close()
