import adafruit_mpl3115a2
import adafruit_bno055

class SensorSet:

    ALTI_BUS = 0x60
    IMU_BUS = 0x28
    ALTI_INIT_AVG_COUNT = 50
    ALTI_INIT_FAULT_LIM = 3

    def __init__(self, i2c, logger, ground_pressure=102250) -> None:
        self.i2c = i2c
        self.ground_pressure = ground_pressure
        self.logger = logger
        
        sensorset_init_message = "SS-INIT\n"
        sensorset_init_message += self.__altimeter_reset(i2c, ground_pressure)
        sensorset_init_message += self.__imu_reset(i2c)
        sensorset_init_message += self.__average_on_pad_alti()
        logger.log(sensorset_init_message)

    def get_on_pad_alti(self):
        return self.alti_on_pad

    def read_data(self):
        logger = self.logger
        # diagnose and reset sensors if necessary
        if not self.healthy_alti:
            self.__altimeter_reset(self.i2c, self.ground_pressure)
        if not self.healthy_imu:
            self.__imu_reset(self.i2c)
        # attempt to read data from sensors
        altitude = self.read_altitude()
        za, lza = self.read_acceleration()
        logger.log(f"SS-READ:ALTI-{altitude}|ZACC-{za}|LZAC-{lza}|HEATI-{self.healthy_alti}|HEIMU-{self.healthy_imu}")
        return altitude, za, lza, self.healthy_alti, self.healthy_imu

    def read_acceleration(self):
        imu = self.imu
        logger = self.logger
        try:
            _, _, za = imu.acceleration
            _, _, lza = imu.linear_acceleration
            self.za = za
            self.lza = lza
        except Exception as err:
            za, lza = None, None
            logger.log(f"IMU-RDER:{err}")
            self.healthy_imu = False

        return za, lza

    def read_altitude(self):
        altimeter = self.altimeter
        logger = self.logger
        try:
            altitude = altimeter.altitude
            self.altitude = altitude
        except Exception as err:
            altitude = None
            logger.log(f"ALTI-RDER:{err}")
            self.healthy_alti = False

        return altitude
        
    def __average_on_pad_alti(self):
        alti_read_count, average_alti, fail_count = 0, 0, 0
        while alti_read_count < SensorSet.ALTI_INIT_AVG_COUNT:
            altitude = self.read_altitude()
            # allowance of sensor fault and self reset the altimeter
            if not self.healthy_alti and fail_count < SensorSet.ALTI_INIT_FAULT_LIM:
                fail_count += 1
                self.__altimeter_reset(self.i2c, self.ground_pressure)
                continue
            elif fail_count == SensorSet.ALTI_INIT_FAULT_LIM:
                break
            # average filter for a reference on pad altitude
            alti_read_count += 1
            average_alti = (average_alti*(alti_read_count-1)+altitude)/alti_read_count
        self.on_pad_setup = fail_count != SensorSet.ALTI_INIT_FAULT_LIM
        self.alti_on_pad = average_alti

        return f"ONPAD-ALTISET: {average_alti}-{self.on_pad_setup}-{fail_count}\n"
        
    def __altimeter_reset(self, i2c, ground_pressure):
        try:
            altimeter = adafruit_mpl3115a2.MPL3115A2(i2c, address=SensorSet.ALTI_BUS)
            altimeter.sealevel_pressure = ground_pressure
            alti_reset_message = f"ALTI-SUCCESS-GLP:{ground_pressure}\n"
            healthy_alti = True
        except Exception as err:
            altimeter = None
            alti_reset_message = f"ALTI-FAIL-ERR:{err}\n"
            healthy_alti = False
        self.altimeter = altimeter
        self.healthy_alti = healthy_alti

        return alti_reset_message
    
    def __imu_reset(self, i2c):
        try:
            imu = adafruit_bno055.BNO055_I2C(i2c, address=SensorSet.IMU_BUS)
            imu_reset_message = "IMU-SUCCESS\n"
            healthy_imu = True
        except Exception as err:
            imu = None
            imu_reset_message = f"IMU-FAIL-ERR:{err}\n"
            healthy_imu = False
        self.imu = imu
        self.healthy_imu = healthy_imu

        return imu_reset_message
