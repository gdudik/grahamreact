import icm42688_SPI as imu

def read_pwr(): 
    pwr = imu.read_pwr_mgmt()
    print(hex(pwr))

def accel_on():
    imu.set_pwr_mgmt('LN', 'off', 0, 1)