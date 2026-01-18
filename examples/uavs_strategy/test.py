import asyncio
import argparse
from calendar import c
import getpass
import os, os.path as osp
import sys
import redis
import json
import numpy as np
import spade
import agentspeak
import time
import collections
import random
import math


from typing import Dict, List, Optional, Iterable, Tuple, Any
from matplotlib.animation import FuncAnimation
# from time import time
from datetime import datetime
from sympy import N


from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour
from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO
from examples.uavs_strategy.planning_modules.uav_planning_actions import PlanningLib
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.planning_modules.formation_generator import FormationGenerator3D, Formation_Elements
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import FormationAPFStep, Single_APFStep, FetchWorldState ,DT
from examples.uavs_strategy.key_path_analyzer import KeyPathAnalyzer

if __name__ == "__main__":
    _lnglat2utm_convertor = bfunc.LngLat2UTM()
    coord_utm = [425833.8106528529315, 4150365.2198808141984]
    coord_ll = _lnglat2utm_convertor.utm_to_lng_lat(coord_utm[0], coord_utm[1])
    print(f"UTM:{coord_utm} -> LngLat:{coord_ll}")