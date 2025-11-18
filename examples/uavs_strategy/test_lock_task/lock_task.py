import asyncio
import argparse
import getpass
import os, os.path as osp
import sys
import redis
import json
import numpy as np

from spade_bdi.bdi import BDIAgent

async def main(server: str, password: str):
    asl_path = os.path.join(os.path.dirname(__file__), "uav_lock_task.asl")  # blue01的ASL文件路径
    a = BDIAgent("task_lock@{}".format(server), password, asl_path)

    await a.start()
    print("Agent start")
    await asyncio.sleep(5)
    a.bdi.set_belief("can_task_start", True)
    print("Belief set")


    await asyncio.sleep(5)
    await a.stop()

if __name__ == "__main__":

    # 启动
    import spade

    spade.run(main("127.0.0.1", "202127"))
