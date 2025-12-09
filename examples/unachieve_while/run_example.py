import argparse
import asyncio
import getpass
import spade
import os
from spade_bdi.bdi import BDIAgent


async def main(server, password):
    current_dir = os.path.dirname(__file__)
    asl_file = os.path.join(current_dir, "receiver.asl")
    b = BDIAgent("receiver@{}".format(server), password, asl_file)
    b.bdi.set_belief("sender", "sender@{}".format(server))
    await b.start()

    asl_file = os.path.join(current_dir, "sender.asl")
    a = BDIAgent("sender@{}".format(server), password, asl_file)
    a.bdi.set_belief("receiver", "receiver@{}".format(server))
    await a.start()

    await asyncio.sleep(5)
    await a.stop()
    await b.stop()


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
