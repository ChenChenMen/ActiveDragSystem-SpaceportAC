from typing import TypedDict
from enum import Enum

class State(Enum):
	ON_PAD = 0
	BOOST = 1
	COAST = 2
	APOGEE = 3
	DESCENT = 4

class StateUpdate(TypedDict):
    boost_acce_thres: float
    boost_alti_thres: float
    coast_acce_thres: float
    apoge_alti_detec: float
    desct_alti_thres: float
    boost_time_limit: float
    coast_time_limit: float
    abort_time_limit: float

class CriticalTimes(TypedDict):
    lift_off: float
    burn_out: float
    complete: float

class LookUp(TypedDict):
    P00: float
    P10: float
    P01: float
    P20: float
    P11: float
    P02: float
    P30: float
    P21: float
    P12: float
    P03: float
    P40: float
    P31: float
    P22: float
    P13: float
    