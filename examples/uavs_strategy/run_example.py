import sys
import os, os.path as osp

import asyncio
from datetime import datetime, timedelta

SPADE_PKGS_ROOT = osp.abspath(osp.join(osp.abspath(__file__), "../../../.."))
# import pdb; pdb.set_trace()
SPADE_DIR = osp.join(SPADE_PKGS_ROOT, "spade-master")
SPADE_BDI_DIR = osp.join(SPADE_PKGS_ROOT, "spade_bdi-master")

# import pdb; pdb.set_trace()
if not SPADE_DIR in sys.path:
    sys.path.insert(0, SPADE_DIR)
if not SPADE_BDI_DIR in sys.path:
    sys.path.insert(0, SPADE_BDI_DIR)

import agentspeak
import spade
from spade.behaviour import PeriodicBehaviour, TimeoutBehaviour
from spade.template import Template

from spade_bdi.bdi import BDIAgent

fleet1 = [
    122.09686551225596,
    37.56536338371065
]

fleet2 = [
    122.10258217246229,
    37.56342057758475
]


class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, position=None):
        super().__init__(jid, password, asl_file)
        self.position = position if position else None

    async def setup(self):
        _template = Template(metadata={"performative": "RedUavAlert"})
        self.add_behaviour(self.RedUavAlert(period=5, start_at=datetime.now()), _template)

    class RedUavAlert(PeriodicBehaviour):
        # 这里进行红方uav是否在威胁范围内的周期性检查
        async def run(self):
            print(f"{self.agent.name} checking alter from red uavs ...")

    def add_custom_actions(self, actions):
        @actions.add(".act_breakthrough", 1)
        def _action_breakthrough(agent, term, intention):
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            print(f"{agent.name} is breaking through {_arg} ...")
            yield

        @actions.add(".act_escape", 1)
        def _action_escape(agent, term, intention):
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            print(f"{agent.name} is escaping {_arg} ...")
            yield

        @actions.add(".act_attack", 1)
        def _action_attack(agent, term, intention):
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            print(f"{agent.name} is attacking {_arg} ...")
            yield

        @actions.add(".act_detour", 1)
        def _action_detour(agent, term, intention):
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            print(f"{agent.name} is detouring {_arg} ...")
            yield

        @actions.add(".act_get_position", 1)
        def _action_get_position(agent, term, intention):
            # 获取 UAV 当前的位置
            position = self.position
            print(f"{agent.name} current position: {position}")

            # 将位置绑定到 X 变量
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            _arg.set_value(position)  # 将当前位置赋给 X 变量

            yield


async def main(server, password):
    uav_blue01 = BlueUAVAgent(f"blue01@{server}", password, "uav_blue_01.asl",position=fleet1)
    uav_blue01.bdi.set_belief("my_friend", f"blue02@{server}")

    uav_blue02 = BlueUAVAgent(f"blue02@{server}", password, "uav_blue_02.asl",position=fleet2)
    uav_blue02.bdi.set_belief("my_friend", f"blue01@{server}")

    await uav_blue01.start()
    await uav_blue02.start()

    await asyncio.sleep(4)

    await uav_blue01.stop()
    await uav_blue02.stop()


if __name__ == "__main__":
    spade.run(main("127.0.0.1", "202127"))
