"""TLS tunnel for SSH through the Tailscale Funnel: ssh runs it as its ProxyCommand.

The Funnel only accepts TLS on port 10000, and carries the SSH connection to the robot inside it.
This program opens the TLS connection, then copies the bytes both ways: standard input -> robot,
robot -> standard output. It does what "openssl s_client" would do, without needing openssl, which
is not on every system (Windows).

Only asyncio uses the TLS connection, in a single thread: OpenSSL does not support two threads
reading and writing the same connection at once. A second thread waits for the bytes of ssh on the
standard input, which asyncio cannot wait for on every system.

    python tunnel.py holobot1.tail5610aa.ts.net 10000
"""
import asyncio
import os
import socket
import ssl
import sys
import threading


async def relay(host, port):
    reader, writer = await connect(host, port)
    threading.Thread(target=read_input, args=(asyncio.get_running_loop(), writer), daemon=True).start()
    while data := await reader.read(65536):  # robot -> ssh, until the robot closes the connection
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()


async def connect(host, port):
    """The TLS connection to the robot, through the first relay of the Funnel that answers.

    The public name of a robot has one address per relay. After the robot restarts, some relays can
    stay mute for a few minutes: they accept the connection, then cut it. We try them in turn.
    """
    context = ssl.create_default_context()  # checks the certificate of the Funnel, like a browser
    found = await asyncio.get_running_loop().getaddrinfo(host, port, type=socket.SOCK_STREAM)
    for address in dict.fromkeys(info[4][0] for info in found):  # in order, without duplicates
        try:
            return await asyncio.wait_for(
                asyncio.open_connection(address, port, ssl=context, server_hostname=host), timeout=3)
        except (OSError, asyncio.TimeoutError) as error:  # a mute relay: the next one
            failure = error
    raise failure


def read_input(loop, writer):
    """ssh -> robot, until ssh closes our standard input: the bytes are read here, and sent by asyncio."""
    while data := os.read(sys.stdin.fileno(), 65536):  # os.read: no lock left taken when the program ends
        asyncio.run_coroutine_threadsafe(send(writer, data), loop).result()


async def send(writer, data):
    writer.write(data)
    await writer.drain()  # waits while the network is slower than ssh


if __name__ == "__main__":
    asyncio.run(relay(sys.argv[1], int(sys.argv[2])))
