import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError


async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Waiting for global position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Position OK")
            break

    # Arm
    print("Arming...")
    await drone.action.arm()

    # Initial setpoint before starting offboard
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -5.0, 0.0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        return

    print("Taking off to 5m...")
    await asyncio.sleep(5)

    side = 5.0  # square side length in meters

    corners = [
        (side, 0.0),       # Corner 1
        (side, side),      # Corner 2
        (0.0, side),       # Corner 3
        (0.0, 0.0)         # Corner 4 (back home)
    ]

    for i, (north, east) in enumerate(corners):

        print(f"Flying to corner {i+1}")

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north,
                east,
                -5.0,     # altitude (NED -> negative is up)
                0.0
            )
        )

        # Give enough time to reach corner
        await asyncio.sleep(6)

        print("Holding for 2 seconds...")
        await asyncio.sleep(2)

    print("Square completed")

    await drone.offboard.stop()

    print("Landing...")
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())