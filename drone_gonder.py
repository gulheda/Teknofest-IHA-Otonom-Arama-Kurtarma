import asyncio
from mavsdk import System

async def drone_gonder(target_lat, target_lon, target_alt=15):
    """
    Hesaplanan GPS koordinatına drone'u gönderir.
    Hedefe ulaşınca yük bırakır ve eve döner.

    Parametreler:
        target_lat: Hedef enlemi
        target_lon: Hedef boylamı
        target_alt: Uçuş irtifası (varsayılan 15m)
    """
    print(f"\n[DRONE] Hedefe gonderiliyor: {target_lat:.6f}, {target_lon:.6f}")

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14560")

    print("[DRONE] Baglaniliyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[DRONE] Baglandi!")
            break

    print("[DRONE] GPS bekleniyor...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("[DRONE] GPS OK!")
            break

    # Arm
    try:
        await drone.action.arm()
        print("[DRONE] Arm edildi!")
    except Exception as e:
        print(f"[DRONE] Zaten arm edilmis: {e}")

    # Kalkış
    await drone.action.set_takeoff_altitude(target_alt)
    try:
        await drone.action.takeoff()
        print("[DRONE] Kalkiyor...")
        await asyncio.sleep(10)
    except Exception as e:
        print(f"[DRONE] Zaten havada: {e}")
        await asyncio.sleep(3)

    # Hedefe git
    print(f"[DRONE] Hedefe gidiliyor: {target_lat:.6f}, {target_lon:.6f}")
    await drone.action.goto_location(target_lat, target_lon, target_alt, 0)
    await asyncio.sleep(20)

    # Hover
    print("[DRONE] Hedefe ulasti! Hover yapiliyor...")
    await asyncio.sleep(5)

    # Yük bırak
    print("[DRONE] YUK BIRAKILIYOR!")
    # Gerçekte servo komutu:
    # await drone.action.set_actuator(1, 1.0)
    await asyncio.sleep(3)

    # Eve dön
    print("[DRONE] Eve doniyor (RTL)...")
    await drone.action.return_to_launch()
    print("[DRONE] GOREV TAMAMLANDI!")

if __name__ == "__main__":
    # Test için direkt çalıştırılabilir
    target_lat = -35.362485
    target_lon = 149.165440
    asyncio.run(drone_gonder(target_lat, target_lon))
