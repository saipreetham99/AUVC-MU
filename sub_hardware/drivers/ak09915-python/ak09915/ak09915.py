import smbus2
import time

REG_WIA1 = 0x00
REG_WIA2 = 0x01
REG_ST1 = 0x10
REG_HXL = 0x11
REG_ST2 = 0x18
REG_CNTL1 = 0x30
REG_CNTL2 = 0x31
REG_CNTL3 = 0x32

REG_ST1_DRDY = 1

REG_CNTL2_MODE = 1 # single measurement mode
REG_CNTL2_SDR = 1<<6 # low noise drive

REG_CNTL3_SRST = 1 # software reset
_address = 0x0C

# 8.5 ms typical when SDR = 1 (low noise drive)
MIN_DELAY_MEASURE = 0.0085

class CompassData:
    def __init__(self, rawdata):
        self.x_raw = int.from_bytes(rawdata[0:2], 'little', signed=True)
        self.y_raw = int.from_bytes(rawdata[2:4], 'little', signed=True)
        self.z_raw = int.from_bytes(rawdata[4:6], 'little', signed=True)

        # field strength in Gauss
        self.x = self.x_raw*0.0015
        self.y = self.y_raw*0.0015
        self.z = self.z_raw*0.0015


class AK09915:
    def __init__(self, bus=1):
        self._bus = smbus2.SMBus(bus)

        self.reset()
        self._wia1 = self.read(REG_WIA1)[0]
        self._wia2 = self.read(REG_WIA2)[0]


    def reset(self):
        self.write(REG_CNTL3, [REG_CNTL3_SRST])
        time.sleep(0.010)

    def measure(self):
        self.write(REG_CNTL2, [REG_CNTL2_MODE | REG_CNTL2_SDR])
        time.sleep(MIN_DELAY_MEASURE)
        status = self.read(REG_ST1)[0]
        while not ((status & REG_ST1_DRDY) == REG_ST1_DRDY):
            status = self.read(REG_ST1)[0]
            continue

        rawdata = self.read(REG_HXL, 6)
        return CompassData(rawdata)

    def read(self, register, length=1):
        data = self._bus.read_i2c_block_data(_address, register, length)
        return data

    def write(self, register_address, data):
        self._bus.write_i2c_block_data(_address, register_address, data)
