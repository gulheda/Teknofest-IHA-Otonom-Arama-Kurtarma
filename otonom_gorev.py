import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import asyncio
import math
import cv2
import numpy as np
import threading
from mavsdk import System
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from ultralytics import YOLO

# ============================================================
# KAMERA PARAMETRELERİ (Raspberry Pi HQ + 8mm lens)
# ============================================================
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FOV_H = math.radians(60)   # Yatay FOV
FOV_V = math.radians(45)   # Dikey FOV

# ============================================================
# PAYLAŞILAN VERİLER
# ============================================================
latest_frame = [None]
lidar_altitude = [None]
tespit_piksel = [None]
gorev_tamamlandi = [False]

# ============================================================
# YOLO MODELİ
# ============================================================
model = YOLO("yolov8x.pt")

# ============================================================
# GAZEBO NODE
# ============================================================
node = Node()

def image_callback(msg):
    """Gazebo kamerasından görüntü alır"""
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((msg.height, msg.width, 3))
    latest_frame[0] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def lidar_callback(msg):
    """TF03-180 LiDAR mesafe verisi alır"""
    if msg.ranges and msg.ranges[0] != float('inf'):
        lidar_altitude[0] = msg.ranges[0]

node.subscribe(Image, '/camera/image', image_callback)
node.subscribe(LaserScan, '/lidar/range', lidar_callback)

# ============================================================
# KOORDİNAT HESAPLAMA
# ============================================================
def pixel_to_gps(iha_lat, iha_lon, altitude, heading_deg, pixel_x, pixel_y):
    """
    Kamera görüntüsündeki piksel koordinatını GPS koordinatına çevirir.
    
    Parametreler:
        iha_lat: İHA enlemi
        iha_lon: İHA boylamı
        altitude: Yerden yükseklik (LiDAR'dan)
        heading_deg: İHA'nın baktığı yön (derece)
        pixel_x: Tespit edilen nesnenin görüntüdeki X koordinatı
        pixel_y: Tespit edilen nesnenin görüntüdeki Y koordinatı
    
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

    # Metre → GPS derece (Vincenty yaklaşımı)
    R = 6371000
    target_lat = iha_lat + math.degrees(dy_r / R)
    target_lon = iha_lon + math.degrees(dx_r / (R * math.cos(math.radians(iha_lat))))

    return target_lat, target_lon

# ============================================================
# YOLO KİŞİ TESPİT THREAD'İ
# ============================================================
def yolo_thread():
    """
    Arka planda sürekli çalışır.
    Gazebo kamerasından görüntü alır, YOLO ile kişi tespit eder.
    """
    print("[YOLO] Basliyor...")
    while not gorev_tamamlandi[0]:
        if latest_frame[0] is not None:
            frame = latest_frame[0].copy()
            results = model(frame, conf=0.1, verbose=False)

            for r in results:
                for box in r.boxes:
                    if int(box.cls[0]) == 0:  # COCO class 0 = person
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        conf = float(box.conf[0])

                        # Bounding box çiz
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, f"KAZAZEDE {conf:.2f}",
                                    (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                        tespit_piksel[0] = (cx, cy)
                        print(f"[YOLO] KAZAZEDE TESPIT! Piksel: ({cx}, {cy}) | Güven: {conf:.2f}")

            cv2.imshow("YOLO - IHA Kamera", frame)
            cv2.waitKey(1)

    cv2.destroyAllWindows()

# ============================================================
# DRONE GÖREV FONKSİYONU
# ============================================================
async def drone_gonder(target_lat, target_lon):
    """
    Hesaplanan GPS koordinatına drone'u gönderir.
    Hedefe ulaşınca yük bırakır ve eve döner.
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
    await drone.action.set_takeoff_altitude(15)
    try:
        await drone.action.takeoff()
        print("[DRONE] Kalkiyor...")
        await asyncio.sleep(10)
    except Exception as e:
        print(f"[DRONE] Zaten havada: {e}")
        await asyncio.sleep(3)

    # Hedefe git
    print(f"[DRONE] Hedefe gidiliyor: {target_lat:.6f}, {target_lon:.6f}")
    await drone.action.goto_location(target_lat, target_lon, 15, 0)
    await asyncio.sleep(20)

    # Hover + Yük bırak
    print("[DRONE] Hedefe ulasti! Hover yapiliyor...")
    await asyncio.sleep(5)
    print("[DRONE] YUK BIRAKILIYOR!")
    # Gerçekte: servo komutu ile yük bırakma mekanizması çalışır
    # await drone.action.set_actuator(1, 1.0)
    await asyncio.sleep(3)

    # Eve dön
    print("[DRONE] Eve doniyor (RTL)...")
    await drone.action.return_to_launch()
    print("[DRONE] GOREV TAMAMLANDI!")

# ============================================================
# VTOL İHA GÖREV FONKSİYONU
# ============================================================
async def iha_gorev():
    """
    VTOL İHA'ya bağlanır.
    YOLO tespiti gelene kadar bekler.
    Tespit gelince koordinat hesaplar ve drone'a gönderir.
    """
    iha = System()
    await iha.connect(system_address="udpin://0.0.0.0:14550")

    print("[IHA] Baglaniliyor...")
    async for state in iha.core.connection_state():
        if state.is_connected:
            print("[IHA] Baglandi!")
            break

    print("[IHA] YOLO ve LiDAR hazir olana kadar bekleniyor...")
    await asyncio.sleep(3)

    print("[IHA] Kazazede aranıyor...")

    while True:
        # YOLO tespiti var mı?
        if tespit_piksel[0] is not None:
            pixel_x, pixel_y = tespit_piksel[0]

            # İHA GPS konumu al
            async for position in iha.telemetry.position():
                iha_lat = position.latitude_deg
                iha_lon = position.longitude_deg
                break

            # İHA heading al
            async for heading in iha.telemetry.heading():
                iha_heading = heading.heading_deg
                break

            # LiDAR irtifa al (yoksa GPS irtifası kullan)
            if lidar_altitude[0] and lidar_altitude[0] != float('inf'):
                altitude = lidar_altitude[0]
                print(f"[IHA] LiDAR irtifa: {altitude:.2f}m")
            else:
                async for position in iha.telemetry.position():
                    altitude = position.relative_altitude_m
                    break
                print(f"[IHA] GPS irtifa: {altitude:.2f}m")

            print(f"[IHA] Konum: {iha_lat:.6f}, {iha_lon:.6f} | Heading: {iha_heading:.1f}°")

            # Koordinat hesapla
            target_lat, target_lon = pixel_to_gps(
                iha_lat, iha_lon, altitude, iha_heading, pixel_x, pixel_y
            )

            print(f"[IHA] Kazazede koordinati: {target_lat:.6f}, {target_lon:.6f}")

            # Drone'a gönder
            await drone_gonder(target_lat, target_lon)

            gorev_tamamlandi[0] = True
            print("\n[SİSTEM] TÜM GÖREV TAMAMLANDI!")
            break

        await asyncio.sleep(0.5)

# ============================================================
# ANA FONKSİYON
# ============================================================
async def main():
    # YOLO thread'i başlat
    t = threading.Thread(target=yolo_thread, daemon=True)
    t.start()

    # İHA görevini başlat
    await iha_gorev()

if __name__ == "__main__":
    asyncio.run(main())
