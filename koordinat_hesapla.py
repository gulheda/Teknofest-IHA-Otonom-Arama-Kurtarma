import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import asyncio
import math
from mavsdk import System
from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan

# Kamera parametreleri (Raspberry Pi HQ + 8mm lens)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FOV_H = math.radians(60)
FOV_V = math.radians(45)

# LiDAR verisi
lidar_altitude = [None]

def lidar_callback(msg):
    if msg.ranges and msg.ranges[0] != float('inf'):
        lidar_altitude[0] = msg.ranges[0]

def pixel_to_gps(iha_lat, iha_lon, altitude, heading_deg, pixel_x, pixel_y):
    """
    Piksel koordinatından GPS koordinatı hesaplar.
    
    Parametreler:
        iha_lat    : İHA enlemi
        iha_lon    : İHA boylamı
        altitude   : Yerden yükseklik (LiDAR'dan)
        heading_deg: İHA'nın baktığı yön (derece)
        pixel_x    : Tespit edilen nesnenin X piksel koordinatı
        pixel_y    : Tespit edilen nesnenin Y piksel koordinatı
    
    Döndürür:
        target_lat, target_lon: Hedefin GPS koordinatları
    """
    heading = math.radians(heading_deg)

    # Piksel → normalize koordinat (-1 ile 1 arası)
    norm_x = (pixel_x - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2)
    norm_y = (pixel_y - IMAGE_HEIGHT / 2) / (IMAGE_HEIGHT / 2)

    # Normalize → metre offset
    dx = altitude * math.tan(FOV_H / 2) * norm_x
    dy = altitude * math.tan(FOV_V / 2) * norm_y

    # İHA yönüne göre döndür
    dx_r = dx * math.cos(heading) - dy * math.sin(heading)
    dy_r = dx * math.sin(heading) + dy * math.cos(heading)

    # Metre → GPS derece
    R = 6371000
    target_lat = iha_lat + math.degrees(dy_r / R)
    target_lon = iha_lon + math.degrees(dx_r / (R * math.cos(math.radians(iha_lat))))

    return target_lat, target_lon

async def run():
    # LiDAR dinle
    node = Node()
    node.subscribe(LaserScan, '/lidar/range', lidar_callback)

    # İHA'ya bağlan
    iha = System()
    await iha.connect(system_address="udpin://0.0.0.0:14550")

    print("[IHA] Baglaniliyor...")
    async for state in iha.core.connection_state():
        if state.is_connected:
            print("[IHA] Baglandi!")
            break

    # GPS al
    async for position in iha.telemetry.position():
        iha_lat = position.latitude_deg
        iha_lon = position.longitude_deg
        break

    # Heading al
    async for heading in iha.telemetry.heading():
        iha_heading = heading.heading_deg
        break

    # LiDAR irtifa bekle
    import time
    time.sleep(1)

    if lidar_altitude[0] and lidar_altitude[0] != float('inf'):
        altitude = lidar_altitude[0]
        print(f"[LiDAR] Irtifa: {altitude:.2f}m")
    else:
        async for position in iha.telemetry.position():
            altitude = position.relative_altitude_m
            break
        print(f"[GPS] Irtifa: {altitude:.2f}m")

    print(f"[IHA] Konum: {iha_lat:.6f}, {iha_lon:.6f}")
    print(f"[IHA] Heading: {iha_heading:.1f}")

    # Test piksel koordinatı (normalde YOLO'dan gelir)
    pixel_x, pixel_y = 599, 70

    target_lat, target_lon = pixel_to_gps(
        iha_lat, iha_lon, altitude, iha_heading, pixel_x, pixel_y
    )

    print(f"\n[SONUC] Kazazede Koordinati:")
    print(f"  Lat: {target_lat:.6f}")
    print(f"  Lon: {target_lon:.6f}")

    return target_lat, target_lon

if __name__ == "__main__":
    asyncio.run(run())
