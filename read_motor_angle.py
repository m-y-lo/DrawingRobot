from dynamixel_sdk import *  # pip install dynamixel-sdk
import time

DEVICENAME = "COM5"                 # Win: "COMX" | macOS: "/dev/tty.usbserial-XXXX"
                                    # check com port in device manager
BAUDRATE   = 57600

port = PortHandler(DEVICENAME)
packet = PacketHandler(2.0)
port.openPort()
port.setBaudRate(BAUDRATE)
# print(port.openPort())
MOTOR_ID = 5
ticks, comm, err = packet.read4ByteTxRx(port, MOTOR_ID, 132)
ticks_raw = ticks % 4096
angle = ticks_raw / 4095 * 360
print(angle)

packet.write1ByteTxRx(port, 3, 11, 3)

port.closePort()