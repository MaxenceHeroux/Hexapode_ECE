"""Pilote minimal du ST LSM6DSV16X en I2C (carte Neuro IMU, PCB/Neuro_IMU).

Lecture accelerometre + gyroscope + temperature, sans ROS (teste seul). Registres
d'apres la datasheet ST LSM6DSV16X :
  WHO_AM_I 0x0F = 0x70 ; CTRL1 0x10 (ODR accel) ; CTRL2 0x11 (ODR gyro) ;
  CTRL3 0x12 (BDU, IF_INC, SW_RESET) ; CTRL6 0x15 (pleine echelle gyro) ;
  CTRL8 0x17 (pleine echelle accel) ; STATUS 0x1E ; OUT_TEMP 0x20 ;
  OUTX_L_G 0x22..0x27 (gyro X, Y, Z) puis OUTX_L_A 0x28..0x2D (accel X, Y, Z).
"""
import math
import struct
import time

WHO_AM_I, CTRL1, CTRL2, CTRL3, CTRL6, CTRL8, STATUS, OUT_TEMP, OUTX_L_G = \
    0x0F, 0x10, 0x11, 0x12, 0x15, 0x17, 0x1E, 0x20, 0x22
WHO_AM_I_VALUE = 0x70
G = 9.80665

# code ODR (CTRL1 / CTRL2, bits 3..0) -> Hz
ODR_CODES = {7.5: 0x2, 15: 0x3, 30: 0x4, 60: 0x5, 120: 0x6, 240: 0x7, 480: 0x8, 960: 0x9}
# pleine echelle accel (g) -> (code CTRL8, sensibilite mg/LSB)
ACC_FS = {2: (0x0, 0.061), 4: (0x1, 0.122), 8: (0x2, 0.244), 16: (0x3, 0.488)}
# pleine echelle gyro (dps) -> (code CTRL6, sensibilite mdps/LSB)
GYR_FS = {125: (0x0, 4.375), 250: (0x1, 8.75), 500: (0x2, 17.5), 1000: (0x3, 35.0),
          2000: (0x4, 70.0), 4000: (0xC, 140.0)}


class LSM6DSV16X:
    def __init__(self, bus=1, address=0x6A, odr=240, acc_fs=4, gyr_fs=1000):
        import smbus
        self.bus = smbus.SMBus(bus)
        self.addr = address
        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        if who != WHO_AM_I_VALUE:
            raise RuntimeError(f"WHO_AM_I = 0x{who:02x} (attendu 0x70) a l'adresse 0x{address:02x}")
        self.bus.write_byte_data(self.addr, CTRL3, 0x01)          # SW_RESET
        time.sleep(0.05)
        self.bus.write_byte_data(self.addr, CTRL3, 0x44)          # BDU + IF_INC
        self.acc_scale = ACC_FS[acc_fs][1] * 1e-3 * G             # LSB -> m/s2
        self.gyr_scale = math.radians(GYR_FS[gyr_fs][1] * 1e-3)   # LSB -> rad/s
        self.bus.write_byte_data(self.addr, CTRL8, ACC_FS[acc_fs][0])
        self.bus.write_byte_data(self.addr, CTRL6, GYR_FS[gyr_fs][0])
        self.bus.write_byte_data(self.addr, CTRL1, ODR_CODES[odr])  # mode haute performance
        self.bus.write_byte_data(self.addr, CTRL2, ODR_CODES[odr])
        time.sleep(0.1)

    def read(self):
        """(ax, ay, az) m/s2, (gx, gy, gz) rad/s, repere du composant."""
        d = bytes(self.bus.read_i2c_block_data(self.addr, OUTX_L_G, 12))
        gx, gy, gz, ax, ay, az = struct.unpack("<6h", d)
        return ((ax * self.acc_scale, ay * self.acc_scale, az * self.acc_scale),
                (gx * self.gyr_scale, gy * self.gyr_scale, gz * self.gyr_scale))

    def temperature(self):
        d = bytes(self.bus.read_i2c_block_data(self.addr, OUT_TEMP, 2))
        return 25.0 + struct.unpack("<h", d)[0] / 256.0

    def close(self):
        self.bus.close()
