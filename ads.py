import os
import board
import logger
import sensorSet
from datetime import date
import numpy as np
import Adafruit_BBIO.PWM as PWM

from define_types import State, StateUpdate, CriticalTimes, LookUp

class ADS:

    g0 = 9.80665

    def __init__(self, system_param, test_mode=False) -> None:
        """initialize an Active Drag System and incorporate test mode for ground test

        Args:
            system_param (dict): dictionary that contains user-defined and system parameters
            test_mode (bool, optional): switch for test mode. Defaults to False.
        """
        # initializing logger with date specified file name
        launch_day = date.today().strftime("%m%d%y")
        execlog_dir = system_param.get("execlog_dir",".")
        execlog = logger.Logger(os.path.join(execlog_dir,"launch"+launch_day+".txt"))

        # initializing sensors
        i2c = board.I2C()
        ground_pressure = system_param.get("ground_pressure",102250)
        sensors = sensorSet.SensorSet(i2c=i2c,ground_pressure=ground_pressure,logger=execlog)
        
        # setting up PWM signal for the servo
        duty_max, duty_min = system_param["duty_cyle"]
        max_angl, min_angl = system_param.get("servo_lim",[100,120])
        servo_pin = system_param["servo_pin"]
        duty_span = duty_max-duty_min
        # lambda function to translate deployment percentage to PWM duty cycle
        # max_angl and min_angl are given by physical system actuation range
        # duty_max and duty_min are suggested from TODO percentage 
        deply_duty = lambda deply: 100-((((max_angl-min_angl)*deply/100+min_angl)/180)*duty_span+duty_min)
        PWM.start(servo_pin, deply_duty(0), 60.0, 1)

        # loading system parameters
        lookup_coeff = system_param["look_up_coeff"]
        kalman_filter_weight = system_param["kalman_filter"]
        state_update_criteria = system_param["state_update_criteria"]

        # initialize class instance fields
        self.execlog = execlog
        self.sensors = sensors
        self.servo_pin = servo_pin
        self.state = State.ON_PAD
        self.lookup_coeff: LookUp = lookup_coeff
        self.kalman_filter_weight = kalman_filter_weight
        self.state_update_criteria: StateUpdate = state_update_criteria
        self.on_pad_alti = sensors.get_on_pad_alti()
        self.apogee_alti = sensors.get_on_pad_alti()
        self.critical_times: CriticalTimes = {
            "lift_off": 0,
            "burn_out": 0,
            "complete": 0
        }
        self.test_mode = test_mode
        self.sensor_health = True
        self.deply_duty = deply_duty
        self.fail_time = execlog.get_log_time()
        # kalman filter initialized state and covariance
        self.x = np.array([[0],[0]])
        self.P = np.zeros(shape=[2,2])
        self.za = 0

    def get_mission_time(self):
        """getter for elapsed mission time

        Returns:
            float: elapsed mission time
        """
        return self.execlog.get_log_time()
    
    def get_state(self):
        """getter for state machine state

        Returns:
            State: current state of the ADS instance
        """
        return self.state
    
    def get_sensor_health(self):
        """getter for sensor set health or data availability

        Returns:
            bool: if all sensors' data is available
        """
        return self.sensor_health

    def update(self, dt):
        """update the dynamics state of the ADS/LV
        update the state machine state of the ADS
        update servo deployment duty cycle

        Args:
            dt (float): time gap since the last state update

        Returns:
            float: ADS deployment percentage
            State: ADS machine state
            bool: if abort is triggered
        """
        execlog = self.execlog
        sensors = self.sensors
        fail_time = self.fail_time
        suc = self.state_update_criteria
        x = self.x
        p = self.P

        # retrieve Kalman Filter design variables
        obser_model = np.array(self.kalman_filter_weight["H"])
        pro_noi_cov = np.array(self.kalman_filter_weight["Q"])
        obs_noi_cov = np.array(self.kalman_filter_weight["R"])
        # update dynamics measurements from sensor set
        altitude, za, lza, healthy_alti, healthy_imu = sensors.read_data()

        if healthy_alti and healthy_imu:
            # state estimator - standard Kalman filter
            state_matrix = np.array([[1.0, dt], [0.0, 1.0]])
            input_matrix = np.array([[0.5*dt**2], [dt]])
            # prediction steps
            x = state_matrix @ x + input_matrix * lza
            p = state_matrix @ p @ state_matrix.T + pro_noi_cov
            # update steps
            y = altitude - float(obser_model @ x)
            resid_cov = obser_model @ p @ obser_model.T + obs_noi_cov
            kalm_gain = (p @ obser_model.T) / resid_cov
            x += kalm_gain * y
            p = (np.eye(2) - kalm_gain @ obser_model) @ p

            # re-assign the dynamics states
            self.x = x
            self.P = p
            self.za = za

            # record the maximum altitude reached
            if altitude > self.apogee_alti:
                self.apogee_alti = altitude
            # update state machine state with updated dynamical states
            self.update_machine_state()
            # refresh sensor set fail time
            self.fail_time = execlog.get_log_time()

        self.sensor_health = healthy_alti and healthy_imu
        # signal abort command when sensor set fails and reconnect overtime
        abort = self.sensor_health and execlog.get_log_time()-fail_time > suc["abort_time_limit"]
        # instruct deployment percentage based on updated machine state and dynamical states
        deployment = self.deployment_plan()
        PWM.set_duty_cycle(self.servo_pin,self.deply_duty(deployment))
        execlog.log(f"ADS:{self.state}|KFALT-{float(x[0])}|KFVEL-{float(x[1])}|DEPLY-{deployment}")

        return deployment, self.state, abort

    def update_machine_state(self):
        """state machine logic implemented in if-else conditional statements
        """
        execlog = self.execlog
        suc = self.state_update_criteria
        on_pad_alti = self.on_pad_alti
        apogee_alti = self.apogee_alti
        alti = float(self.x[0])
        acce = self.za
        # ascent event timeline
        lift_off_time = self.critical_times["lift_off"]
        burn_out_time = self.critical_times["burn_out"]
        complete_time = self.critical_times["complete"]

        # state machine update - switch case structure
        state = self.state
        if state is State.ON_PAD:
            # dual criterias must meet to trigger the launch stage change
            if acce >= suc["boost_acce_thres"]*ADS.g0 and alti >= suc["boost_alti_thres"] + on_pad_alti:
                lift_off_time = execlog.get_log_time()
                self.critical_times.update({"lift_off": lift_off_time})
                execlog.log(f"LIFTOFF@{lift_off_time}\n")
                state = State.BOOST

            # embedded test mode to skip the launch impulse detection and drive state machine by overtimes
            if self.test_mode and execlog.get_log_time() >= 15:
                lift_off_time = execlog.get_log_time()
                self.critical_times.update({"lift_off": lift_off_time})
                execlog.log(f"[TEST]LIFTOFF@{lift_off_time}\n")
                state = State.BOOST

        elif state is State.BOOST:
            # coast phase determination by either change of impulse at burnout or a motor burn overtime
            if acce <= suc["coast_acce_thres"]*ADS.g0 or execlog.get_log_time()-lift_off_time >= suc["boost_time_limit"]:
                burn_out_time = execlog.get_log_time()
                self.critical_times.update({"burn_out": burn_out_time})
                execlog.log(("[TEST]" if self.test_mode else "") + f"BURNOUT@{burn_out_time}\n")
                state = State.COAST

        elif state is State.COAST:
            # apogee state determination by either decrements of altitude or coast momentum overtime
            if alti < apogee_alti-suc["apoge_alti_detec"] or execlog.get_log_time()-burn_out_time >= suc["coast_time_limit"]:
                complete_time = execlog.get_log_time()
                self.critical_times.update({"complete": complete_time})
                execlog.log(("[TEST]" if self.test_mode else "") + f"APOGEE@{complete_time}\n")
                state = State.APOGEE

        elif state is State.APOGEE:
            # remind operation until a determining altitude (mainly for recording sensor data during descent)
            if alti <= suc["desct_alti_thres"]+on_pad_alti:
                state = State.DESCENT

        self.state = state

    def deployment_look_up(self):
        """polyfit 45 implemented deployment look up table based on dynamics state

        Returns:
            float: percentage of deployment (could be >100 or <0)
        """
        lookup_coeff = self.lookup_coeff
        h, v = self.x

        # look up table for deployment percentage by polyfit45
        return lookup_coeff["P00"] + lookup_coeff["P10"]*v + lookup_coeff["P01"]*h + \
            lookup_coeff["P20"]*v**2 + lookup_coeff["P11"]*v*h + lookup_coeff["P02"]*h**2 + \
            lookup_coeff["P30"]*v**3 + lookup_coeff["P21"]*v**2*h + lookup_coeff["P12"]*v*h**2 + \
            lookup_coeff["P03"]*h**3 + lookup_coeff["P40"]*v**4 + lookup_coeff["P31"]*v**3*h + \
            lookup_coeff["P22"]*v**2*h**2 + lookup_coeff["P13"]*v*h**3
    
    def deployment_plan(self):
        """state machine state based deployment instruction

        Returns:
            float: percentage of deployment (strictly [0, 100])
        """
        state = self.state
        return min(max(0, self.deployment_look_up()), 100) if state is State.COAST and self.sensor_health else 0

    def clean(self):
        """clean up logger and PWM signals
        """
        # clean up data logger
        self.execlog.clean()
        # stop PWM signals
        PWM.stop(self.servo_pin)
        PWM.cleanup()
