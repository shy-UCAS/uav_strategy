import argparse
import asyncio
import getpass

import spade
import os
from spade_bdi.bdi import BDIAgent


async def main(server, password):
    current_dir = os.path.dirname(__file__)
    asl_file = os.path.join(current_dir, "basic.asl")
    a = BDIAgent(f"bdiagent@{server}", password, asl_file)
    await a.start()

    await asyncio.sleep(1)

    a.bdi.set_belief("car", "azul", "big")
    print("BEFORE SETTING BELIEF")
    a.bdi.print_beliefs()
    print(f"\n")
    print("GETTING FIRST CAR BELIEF")
    print(a.bdi.get_belief("car"))
    print(f"\n")
    print("GETTING ALL BELIEFS")
    a.bdi.print_beliefs()
    a.bdi.remove_belief("car", 'azul', "big")
    a.bdi.print_beliefs()
    print(a.bdi.get_beliefs())
    a.bdi.set_belief("car", 'amarillo')
    await asyncio.sleep(0.1)
    a.bdi.print_beliefs()

    await a.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", help="XMPP Server")
    parser.add_argument("--password", help="Password")
    args = parser.parse_args()

    if args.server is None:
        server = input("XMPP Server> ")
    else:
        server = args.server

    if args.password is None:
        passwd = getpass.getpass()
    else:
        passwd = args.password
    spade.run(main(server, passwd))
