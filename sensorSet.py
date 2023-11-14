import adafruit_mpl3115a2
import adafruit_bno055

class SensorSet:

    ALTI_BUS = 0x60
    IMU_BUS = 0x28
    ALTI_INIT_AVG_COUNT = 50
    ALTI_INIT_FAULT_LIM = 3

    def __init__(self, i2c, logger, ground_pressure=102250) -> None:
        """Create a sensor set composed by a BNO055 IMU and a MPL3115A2 Altimeter
        Apply an average filter for 50 samples to obtain a stablizing on pad altitude

        Args:
            i2c (I2C): i2c bus object from the Beaglebone Black
            logger (Logger): data logger
            ground_pressure (int, optional): ground air pressure, 
            used to replace sealevel pressure. Defaults to 102250.
        """
        # assign fields for the class instance
        self.i2c = i2c
        self.ground_pressure = ground_pressure
        self.logger = logger
        
        # proceed to initialize Altimeter, IMU, and determine altitude on pad
        sensorset_init_message = "SS-INIT\n"
        sensorset_init_message += self.__altimeter_reset(i2c, ground_pressure)
        sensorset_init_message += self.__imu_reset(i2c)
        sensorset_init_message += self.__average_on_pad_alti()
        logger.log(sensorset_init_message)

    def get_on_pad_alti(self):
        """getter function for on pad altitude obtained during set up

        Returns:
            float: on pad altitude
        """
        return self.alti_on_pad

    def read_data(self):
        """acquire packaged z-axis accelerations and altitude

        Returns:
            float: altitude measurement
            float: z-axis acceleration
            float: z-axis linear acceleration
            bool: altimeter data availability
            bool: IMU data availability
        """
        logger = self.logger
        # diagnose sensor health and reset sensors if necessary
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
        """read measurement from IMU accelerometer

        Returns:
            float: z-axis acceleration
            float: z-axis linear acceleration
        """
        imu = self.imu
        logger = self.logger
        # obtain z-axis acceleration from IMU
        try:
            _, _, za = imu.acceleration
            _, _, lza = imu.linear_acceleration
            self.za = za
            self.lza = lza
        # log exceptions reported
        except Exception as err:
            za, lza = None, None
            logger.log(f"IMU-RDER:{err}")
            self.healthy_imu = False

        return za, lza

    def read_altitude(self):
        """read measurement from altimeter

        Returns:
            float: altitude measurement
        """
        altimeter = self.altimeter
        logger = self.logger
        # obtain altitude from altimeter
        try:
            altitude = altimeter.altitude
            self.altitude = altitude
        # log exceptions reported
        except Exception as err:
            altitude = None
            logger.log(f"ALTI-RDER:{err}")
            self.healthy_alti = False

        return altitude
        
    def __average_on_pad_alti(self):
        """initialize the altimeter reading and record an on pad altitude

        Returns:
            str: on pad altitude initialization message
        """
        alti_read_count, average_alti, fail_count = 0, 0, 0
        # average filter loop
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

        # at the event of on pad altitude fail, sensor set will initialize on pad altitude to be zero
        self.on_pad_setup = fail_count != SensorSet.ALTI_INIT_FAULT_LIM
        self.alti_on_pad = average_alti

        return f"ONPAD-ALTISET: {average_alti}-{self.on_pad_setup}-{fail_count}\n"
        
    def __altimeter_reset(self, i2c, ground_pressure):
        """initialize the altimeter by re-assigning an altimeter instance

        Args:
            i2c (I2C): i2c bus object from the Beaglebone Black
            ground_pressure (float): ground air pressure, used to replace sealevel pressure.

        Returns:
            str: altimeter initialization message
        """
        # reset altimeter by reconstruct an altimeter object
        try:
            altimeter = adafruit_mpl3115a2.MPL3115A2(i2c, address=SensorSet.ALTI_BUS)
            altimeter.sealevel_pressure = ground_pressure
            alti_reset_message = f"ALTI-SUCCESS-GLP:{ground_pressure}\n"
            healthy_alti = True
        # log exceptions reported
        except Exception as err:
            altimeter = None
            alti_reset_message = f"ALTI-FAIL-ERR:{err}\n"
            healthy_alti = False

        self.altimeter = altimeter
        self.healthy_alti = healthy_alti

        return alti_reset_message
    
    def __imu_reset(self, i2c):
        """initialize the IMU by re-assigning an IMU instance

        Args:
            i2c (I2C): i2c bus object from the Beaglebone Black

        Returns:
            str: IMU initialization message
        """
        # reset IMU by reconstruct an IMU object
        try:
            imu = adafruit_bno055.BNO055_I2C(i2c, address=SensorSet.IMU_BUS)
            imu_reset_message = "IMU-SUCCESS\n"
            healthy_imu = True
        # log exceptions reported
        except Exception as err:
            imu = None
            imu_reset_message = f"IMU-FAIL-ERR:{err}\n"
            healthy_imu = False
        self.imu = imu
        self.healthy_imu = healthy_imu

        return imu_reset_message
