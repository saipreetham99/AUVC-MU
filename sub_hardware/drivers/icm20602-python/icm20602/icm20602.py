import spidev
import time

class Vector3D:
    def __init__(self, data):
        self.x = int.from_bytes(data[0:2], "big", signed=True)
        self.y = int.from_bytes(data[2:4], "big", signed=True)
        self.z = int.from_bytes(data[4:6], "big", signed=True)
    
    def __mul__(self, other):
        result = Vector3D([0]*6)
        result.x = self.x*other
        result.y = self.y*other
        result.z = self.z*other
        return result

    def __rmul__(self, other):
        return self.__mul__(other)

class ICMData:
    def __init__(self, rawdata):
        self.a_raw = Vector3D(rawdata[0:6])
        self.t_raw = int.from_bytes(rawdata[6:8], "big", signed=True)
        self.g_raw = Vector3D(rawdata[8:14])

        self.a = self.a_raw*(2.0/0x8000)
        # see TEMP_OUT register documentation
        self.t = 25 + self.t_raw/326.8
        self.g = self.g_raw*(250.0/0x8000)

class ICM20602(object):
    REG_XG_OFFS_TC_H = 0x04
    REG_XG_OFFS_TC_L = 0x05
    REG_YG_OFFS_TC_H = 0x07
    REG_YG_OFFS_TC_L = 0x08
    REG_ZG_OFFS_TC_H = 0x0A
    REG_ZG_OFFS_TC_L = 0x0B
    REG_SELF_TEST_X_ACCEL = 0x0D
    REG_SELF_TEST_Y_ACCEL = 0x0E
    REG_SELF_TEST_Z_ACCEL = 0x0F
    REG_XG_OFFS_USRH = 0x13
    REG_XG_OFFS_USRL = 0x14
    REG_YG_OFFS_USRH = 0x15
    REG_YG_OFFS_USRL = 0x16
    REG_ZG_OFFS_USRH = 0x17
    REG_ZG_OFFS_USRL = 0x18
    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_ACCEL_CONFIG_2 = 0x1D
    REG_LP_MODE_CFG = 0x1E
    REG_ACCEL_WOM_X_THR = 0x20
    REG_ACCEL_WOM_Y_THR = 0x21
    REG_ACCEL_WOM_Z_THR = 0x22
    REG_FIFO_EN = 0x23
    REG_FSYNC_INT = 0x36
    REG_INT_PIN_CFG = 0x37
    REG_INT_ENABLE = 0x38
    REG_FIFO_WM_INT_STATUS = 0x39
    REG_INT_STATUS = 0x3A
    REG_ACCEL_XOUT_H = 0x3B
    REG_ACCEL_XOUT_L = 0x3C
    REG_ACCEL_YOUT_H = 0x3D
    REG_ACCEL_YOUT_L = 0x3E
    REG_ACCEL_ZOUT_H = 0x3F
    REG_ACCEL_ZOUT_L = 0x40
    REG_TEMP_OUT_H = 0x41
    REG_TEMP_OUT_L = 0x42
    REG_GYRO_XOUT_H = 0x43
    REG_GYRO_XOUT_L = 0x44
    REG_GYRO_YOUT_H = 0x45
    REG_GYRO_YOUT_L = 0x46
    REG_GYRO_ZOUT_H = 0x47
    REG_GYRO_ZOUT_L = 0x48
    REG_SELF_TEST_X_GYRO = 0x50
    REG_SELF_TEST_Y_GYRO = 0x51
    REG_SELF_TEST_Z_GYRO = 0x52
    REG_FIFO_WM_TH1 = 0x60
    REG_FIFO_WM_TH2 = 0x61
    REG_SIGNAL_PATH_RESET = 0x68
    REG_ACCEL_INTEL_CTRL = 0x69
    REG_USER_CTRL = 0x6A
    REG_PWR_MGMT_1 = 0x6B
    REG_PWR_MGMT_2 = 0x6C
    REG_I2C_IF = 0x70
    REG_FIFO_COUNTH = 0x72
    REG_FIFO_COUNTL = 0x73
    REG_FIFO_R_W = 0x74
    REG_WHO_AM_I = 0x75
    REG_XA_OFFSET_H = 0x77
    REG_XA_OFFSET_L = 0x78
    REG_YA_OFFSET_H = 0x7A
    REG_YA_OFFSET_L = 0x7B
    REG_ZA_OFFSET_H = 0x7D
    REG_ZA_OFFSET_L = 0x7E

    def __init__(self, bus=1, cs=2):

        # gyro digital low pass filter configuration parameters
        # see register 26 documentation
        # Note bit 7 should be set to 0 by the user
        self._dlpf_cfg = 0x1 # 1kHz sample rate

        # gyro configuration
        # see register 27 documentation
        self._gyro_fs_sel = 0x0 << 3 # full scale resolution
        self._fchoice_b = 0x0 # dlpf bandwidth bypass

        # accelerometer configuration
        # see register 0x28 documentation
        self._accel_fs_sel = 0x0 << 3

        # accelerometer configuration 2
        # see register 0x29 documentation
        self._accel_fchoice_b = 0 << 3 # no bypass, 1khz output
        self._accel_dlpf_cfg = 0

        # see register 0x70, enable/disable i2c interface
        self.i2c = False

        self._bus = spidev.SpiDev()
        self._bus.open(bus, cs)
        self._bus.max_speed_hz = 10000000

        print("reset", self.reset())
        self._id = self.read_id()
        print("detected device id %d on bus %s" % (self._id, bus))
        self.initialize()


    def initialize(self):
        # disable i2c communication
        self.write(self.REG_I2C_IF, [0x40])

        # gyro dlpf configuration
        self.write(self.REG_CONFIG, [self._dlpf_cfg])

        # gyro full scale range and dlpf bypass selection
        self.write(self.REG_GYRO_CONFIG, [self._fchoice_b | self._gyro_fs_sel])

        # accelerometer full scale range configuration
        self.write(self.REG_ACCEL_CONFIG, [self._accel_fs_sel])

        # accelerometer dlpf and dlpf bypass configuration
        self.write(self.REG_ACCEL_CONFIG_2, [self._accel_fchoice_b | self._accel_dlpf_cfg])

        # OUTPUT_LIMIT bit:
        # "To avoid limiting sensor output to less than 0x7FFF,
        # set this bit to 1. This should be done every time the ICM-20602 is powered up"
        self.write(self.REG_ACCEL_INTEL_CTRL, [0x2])

        # exit sleep mode
        self.write(self.REG_PWR_MGMT_1, [0x01])

        # delay to allow sensors to start up and stabilize
        time.sleep(0.1)

    def reset(self):
        self.write(self.REG_PWR_MGMT_1, [0x80])
        time.sleep(0.01)
        # device reset bit is cleared when device reset is complete
        return self.readbyte(self.REG_PWR_MGMT_1) == 0x41

    def read_id(self):
        return self.readbyte(self.REG_WHO_AM_I)
    
    def read(self, reg, nbytes=1):
        xferdata = [0] * (nbytes+1)
        xferdata[0] = reg | 0x80 # read transaction
        return self._bus.xfer(xferdata)[1:]

    def readbyte(self, reg):
        return self.read(reg)[0]

    def read_all(self):
        data = self.read(self.REG_ACCEL_XOUT_H, 14)

        return ICMData(data)

    # todo!
    def self_test(self):
        pass

    # todo write and verify
    def write(self, reg, data):
        data.insert(0, reg)
        return self._bus.xfer(data)
