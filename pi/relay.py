"""Runs on the Pi. Forwards TCP connections from the laptop to the robot, so the laptop can reach the robot through
the Pi's Ethernet port and does not have to join the robot's WiFi itself.

    laptop --Ethernet--> Pi :8181 --robot WiFi--> 192.168.4.1:81   (robot WebSocket)
    laptop --Ethernet--> Pi :8180 --robot WiFi--> 192.168.4.1:80   (robot HTTP settings)

Start it with sh pi/start-relay.sh. A WebSocket is an ordinary TCP stream, so copying bytes both ways is enough.
"""
import asyncio

ROBOT = "192.168.4.1"
ROUTES = {8181: 81, 8180: 80}


async def pipe(reader, writer):
    try:
        while data := await reader.read(65536):
            writer.write(data)
            await writer.drain()
    except (ConnectionError, OSError):
        pass
    finally:
        writer.close()


def route(robot_port):
    async def handle(reader, writer):
        try:
            robot_reader, robot_writer = await asyncio.wait_for(asyncio.open_connection(ROBOT, robot_port), 5)
        except (OSError, asyncio.TimeoutError):
            writer.close()
            return
        await asyncio.gather(pipe(reader, robot_writer), pipe(robot_reader, writer))
    return handle


async def main():
    servers = [await asyncio.start_server(route(robot_port), "0.0.0.0", port) for port, robot_port in ROUTES.items()]
    print("relay: " + ", ".join(f"{port} -> {ROBOT}:{robot_port}" for port, robot_port in ROUTES.items()), flush=True)
    await asyncio.gather(*(server.serve_forever() for server in servers))


asyncio.run(main())
