import os, os.path as osp
import sys

import argparse
import asyncio
import getpass
from datetime import datetime, timedelta

SPADE_ROOT = osp.abspath(osp.join(osp.abspath(__file__), "../../../.."))
SPADE_DIR = osp.join(SPADE_ROOT, "spade-master")
SPADE_BDI_DIR = osp.join(SPADE_ROOT, "spade_bdi-master")

if not SPADE_DIR in sys.path:
    sys.path.insert(0, SPADE_DIR)

if not SPADE_BDI_DIR in sys.path:
    sys.path.insert(0, SPADE_BDI_DIR)

import spade
from spade.behaviour import PeriodicBehaviour, TimeoutBehaviour
from spade.template import Template

from spade_bdi.bdi import BDIAgent

class UAVAgent(BDIAgent):
    def add_custom_actions(self, actions):
        @actions.add_function(".move", (int, ))
        def _my_move(x):
            print(f"action 'move': {x} ...")
        
        @actions.add_function(".detour", (int, int))
        def _my_move(x, y):
            print(f"action 'move': {x}, {y} ...")
    
async def main(server, password):
    _blue01 = UAVAgent(f"blue01@{server}", password=password, "blue_uav01.asl")
    _blue02 = UAVAgent(f"blue02@{server}", password=password, "blue_uav02.asl")

    await _blue01.start()
    await _blue02.start()

    await asyncio.sleep(10)

    await _blue01.stop()
    await _blue02.stop()

if __name__ == "__main__":
    spade.run(main("pc-20240630akhw", "weiguanghost"))

    