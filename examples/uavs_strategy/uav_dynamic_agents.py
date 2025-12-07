# 动态创建 / 删除 BlueUAVAgent
import asyncio
import argparse
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour


class DemoAgent(Agent):
    """
    一个最简单的智能体：
    周期性打印自己的 jid，方便观察谁还活着。
    """
    class HeartbeatBehaviour(CyclicBehaviour):
        async def run(self):
            print(f"[HB] {self.agent.jid} is alive.")
            await asyncio.sleep(1)

    async def setup(self):
        print(f"[SETUP] Agent {self.jid} started.")
        hb = self.HeartbeatBehaviour()
        self.add_behaviour(hb)


async def spawn_agent(name: str, server: str, password: str):
    """
    运行过程中动态创建并启动一个智能体。
    """
    jid = f"{name}@{server}"
    ag = DemoAgent(jid, password)
    # 如需自动在 XMPP 服务器上注册账号，可以加 auto_register=True（前提是服务器允许）
    await ag.start()  # await ag.start(auto_register=True)
    print(f"[SPAWN] Agent {jid} created and started.")
    return ag


async def main(server: str, password: str):
    agents = []

    # 1. 启动时先创建一个初始 agent
    a1 = await spawn_agent("agent01", server, password)
    agents.append(a1)

    # 2. 用一个简单循环模拟“程序运行过程”
    #    每隔几秒钟：创建新 agent / 停掉旧 agent
    for step in range(6):
        print(f"\n========== STEP {step} ==========")

        # 在 step == 2 时，动态再创建一个智能体
        if step == 2:
            a2 = await spawn_agent("agent02", server, password)
            agents.append(a2)

        # 在 step == 4 时，销毁最早的那个智能体
        if step == 4 and agents:
            old = agents.pop(0)
            print(f"[STOP] Stopping oldest agent: {old.jid}")
            await old.stop()

        await asyncio.sleep(2)

    # 3. 结束前把剩下的智能体都停掉
    print("\n[SHUTDOWN] Stopping remaining agents...")
    for ag in agents:
        print(f"[STOP] {ag.jid}")
        await ag.stop()

    # SPADE 内部有些清理是异步的，留一点时间完成关闭
    await asyncio.sleep(1)
    print("[DONE] All agents stopped. Bye.")


if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--server", type=str, default="127.0.0.1",
    #                     help="XMPP server domain, e.g., 127.0.0.1 or localhost")
    # parser.add_argument("--password", type=str, required=True,
    #                     help="Password for all test agents")
    # args = parser.parse_args()

    # 启动 asyncio 主循环
    asyncio.run(main("127.0.0.1", "202127"))