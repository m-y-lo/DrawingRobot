from __future__ import annotations
from dynamixel_sdk import *  # pip install dynamixel-sdk
import time
from typing import Iterable, Dict, List, Tuple, Optional


## motor 1: prismatic
## motor 2: first revolute
## motor 3: second revolute
## motor 5 (4): color switcher

## GOAL: draw 5 strokes, switch colors
## TODO: will need to make sure all motor directions are correct once arm is assembled! (can change direction with a +/-1)


# Ports and Joint IDs
DEVICENAME = "COM5"                 # Win: "COMX" | macOS: "/dev/tty.usbserial-XXXX"
                                    # check com port in device manager
BAUDRATE   = 57600
JOINT_IDS  = [1, 2, 3, 5] #6]        

# Direction
JOINT_OFFSETS_DEG = [0, 0, 0, 0]
JOINT_SIGNS       = [+1, +1, +1, +1]

# Limitaions
SOFT_LIMITS_DEG: List[Optional[Tuple[float, float]]] = [
    (0, 300), (0, 300), (0, 300), (0, 300)
]

# Smooth
PROFILE_ACCEL = 50
PROFILE_VELOC = 150

# Control Table
ADDR_TORQUE_ENABLE     = 64
ADDR_OPERATING_MODE    = 11
ADDR_GOAL_POSITION     = 116
ADDR_PRESENT_POSITION  = 132
ADDR_PROFILE_VELOCITY  = 112
ADDR_PROFILE_ACCEL     = 108

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0
MODE_POSITION  = 3
TICKS_MAX      = 4095  

# Constants
PI = 3.1415926
GEAR_RATIO = 30/76      # from little gear to big gear on color switching end effector

# Degree setup
def _wrap_deg(d: float) -> float:
    d = d % 360.0
    return d if d >= 0 else d + 360.0

def deg_to_ticks(deg: float) -> int:
    return int(round(_wrap_deg(deg) / 360.0 * TICKS_MAX)) & TICKS_MAX

def ticks_to_deg(ticks: int) -> float:
    return ((int(ticks) & TICKS_MAX) / TICKS_MAX) * 360.0

class Arm5DOF:
    def __init__(self, port_name: str = DEVICENAME, baud: int = BAUDRATE, joint_ids: List[int] = JOINT_IDS):
        self.joint_ids = joint_ids
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            raise RuntimeError(f"Cannot open port: {port_name}")
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"Cannot set baudrate: {baud}")
        self.pkt = PacketHandler(2.0)

        for jid in self.joint_ids:
            model, comm, err = self.pkt.ping(self.port, jid)
            if comm != COMM_SUCCESS or err != 0:
                raise RuntimeError(f"Ping ID={jid} failed")
            self.pkt.write1ByteTxRx(self.port, jid, ADDR_OPERATING_MODE, MODE_POSITION)
            if PROFILE_ACCEL is not None:
                self.pkt.write4ByteTxRx(self.port, jid, ADDR_PROFILE_ACCEL, int(PROFILE_ACCEL))
            if PROFILE_VELOC is not None:
                self.pkt.write4ByteTxRx(self.port, jid, ADDR_PROFILE_VELOCITY, int(PROFILE_VELOC))
            self.pkt.write1ByteTxRx(self.port, jid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def close(self):
        for jid in self.joint_ids:
            self.pkt.write1ByteTxRx(self.port, jid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.port.closePort()

    def _map_joint(self, jn: int) -> int:
        if not 1 <= jn <= len(self.joint_ids):
            raise ValueError(f"joint number must be 1..{len(self.joint_ids)}")
        return self.joint_ids[jn - 1]

    def _apply_model(self, jn: int, user_deg: float) -> float:
        i = jn - 1
        return _wrap_deg(JOINT_SIGNS[i] * user_deg + JOINT_OFFSETS_DEG[i])

    def _inverse_model(self, jn: int, servo_deg: float) -> float:
        i = jn - 1
        s = JOINT_SIGNS[i]
        off = JOINT_OFFSETS_DEG[i]
        val = (servo_deg - off) / (s if s != 0 else 1)
        return _wrap_deg(val)

    def _apply_limits(self, jn: int, user_deg: float) -> float:
        lim = SOFT_LIMITS_DEG[jn - 1]
        if lim is None:
            return user_deg
        lo, hi = lim
        d = _wrap_deg(user_deg)
        if lo is not None and d < lo: d = lo
        if hi is not None and d > hi: d = hi
        return d

    def turn(self, jn: int, degree: float, wait: bool=False, tol_deg: float=1.0, timeout: float=3.0):
        jid = self._map_joint(jn)
        limited = self._apply_limits(jn, degree)
        servo_deg = self._apply_model(jn, limited)
        goal = deg_to_ticks(servo_deg)
        self.pkt.write4ByteTxRx(self.port, jid, ADDR_GOAL_POSITION, goal)
        if not wait: return
        t0 = time.time()
        while time.time() - t0 < timeout:
            cur_ticks, _, _ = self.pkt.read4ByteTxRx(self.port, jid, ADDR_PRESENT_POSITION)
            cur_deg = ticks_to_deg(cur_ticks)
            err = min((cur_deg - servo_deg) % 360, (servo_deg - cur_deg) % 360)
            if err <= tol_deg: return
            time.sleep(0.01)

    def turns(self, targets: Iterable[float] | Dict[int, float], wait: bool=False, tol_deg: float=1.0, timeout: float=3.0):
        user_cmd = [None]*len(self.joint_ids)  # type: ignore
        if isinstance(targets, dict):
            for k,v in targets.items():
                if not 1 <= k <= len(self.joint_ids): raise ValueError("joint index out of range")
                user_cmd[k-1] = v
        else:
            seq = list(targets)
            if len(seq) != len(self.joint_ids): raise ValueError(f"list must have {len(self.joint_ids)} angles")
            user_cmd = seq

        gsw = GroupSyncWrite(self.port, self.pkt, ADDR_GOAL_POSITION, 4)
        set_idx = []
        for jn, maybe_deg in enumerate(user_cmd, start=1):
            if maybe_deg is None: continue
            set_idx.append(jn)
            jid = self._map_joint(jn)
            servo_deg = self._apply_model(jn, self._apply_limits(jn, maybe_deg))
            pos = deg_to_ticks(servo_deg)
            param = [pos & 0xFF, (pos>>8)&0xFF, (pos>>16)&0xFF, (pos>>24)&0xFF]
            if not gsw.addParam(jid, bytes(param)):
                raise RuntimeError(f"GSW addParam failed for joint {jn} (ID={jid})")
        if gsw.txPacket() != COMM_SUCCESS: raise RuntimeError("GroupSyncWrite txPacket failed")
        gsw.clearParam()

        if not wait or not set_idx: return
        ids_wait  = [self._map_joint(i) for i in set_idx]
        targets_d = [self._apply_model(i, self._apply_limits(i, user_cmd[i-1])) for i in set_idx]  # type: ignore
        t0 = time.time()
        while time.time() - t0 < timeout:
            ok = True
            for jid, tg in zip(ids_wait, targets_d):
                now_ticks, _, _ = self.pkt.read4ByteTxRx(self.port, jid, ADDR_PRESENT_POSITION)
                now_deg = ticks_to_deg(now_ticks)
                err = min((now_deg - tg) % 360, (tg - now_deg) % 360)
                if err > tol_deg: ok = False; break
            if ok: return
            time.sleep(0.01)

    def get(self, jn: int) -> float:
        jid = self._map_joint(jn)
        ticks, _, _ = self.pkt.read4ByteTxRx(self.port, jid, ADDR_PRESENT_POSITION)
        return ticks_to_deg(ticks)

    def seq_same_delta_hold_then_return(self, delta_deg: float, dwell: float=0.5, settle: bool=True):
        user0 = []
        for jn in range(1, len(self.joint_ids)+1):
            servo_deg = self.get(jn)
            user0.append(self._inverse_model(jn, servo_deg))

        for jn in range(1, len(self.joint_ids)+1):
            self.turn(jn, user0[jn-1] + delta_deg, wait=settle)
            time.sleep(dwell)

        self.turns(user0, wait=True)

    def go_prismatic(self, vertical_amt, direction):
        """
        vertical_amt: the length (in mm) you want to move the prismatic joint
        direction: +1 = up
                   -1 = down
        """
        diameter = 30 #mm
        vert_deg = direction*360*(vertical_amt/(PI*diameter))
        self.turn(jn = 1, degree = vert_deg)


    def go_to(self):
        """
        TODO: implement this function
        this function should move motors 2/3 to move end effector to target position
        (put inverse kinematics here)
        """
        temp = 1

    def scribble():
        """
        TODO: implement this function
        this function should move the arm back and forth in a scribbling motion 5 times
        """

    def color_rotate(self, num_switches):
        """ Move motor 4 to rotate end effector. (One color at a time)"""
        color_switch = 90 
        self.turn(jn = 4, degree = color_switch/GEAR_RATIO)

    def draw_gradient(self):
        self.go_to()
        self.go_prismatic(5, -1)
        self.scribble()
        self.go_prismatic(5, 1)
        self.color_rotate(self, 1)
        self.go_prismatic(5, -1)
        self.scribble()
        self.go_prismatic(5, 1)
        self.color_rotate(self, 1)
        self.go_prismatic(5, -1)
        self.scribble()
        self.go_prismatic(5, 1)
        self.color_rotate(self, 1)
        self.go_prismatic(5, -1)
        self.scribble()
        self.go_prismatic(5, 1)
        self.go_to()
        

    
if __name__ == "__main__":
    arm = Arm5DOF()
    try:
        #arm.seq_same_delta_hold_then_return(delta_deg=30, dwell=0.6, settle=True)
        arm.draw_gradient()

    finally:
        arm.close()


   
