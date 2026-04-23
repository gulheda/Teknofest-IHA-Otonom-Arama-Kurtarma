cat > ~/otonom_gorev.py << 'EOF'
import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import asyncio
import math
import cv2
import numpy as np
import threading
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from ultralytics import YOLO

# ============================================================
# KAMERA PARAMETRELERİ (Raspberry Pi HQ + 8mm lens)
# ============================================================
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FOV_H = math.radians(60)
FOV_V = math.radians(45)

# ============================================================
# GRID TARAMA PARAMETRELERİ
# ============================================================
TARAMA_IRTIFA = 50
SERIT_ARALIGI = 20
ALAN_GENISLIK = 300
ALAN_UZUNLUK = 100
HOME_LAT = -35.363258
HOME_LON = 149.165207

# ============================================================
# PAYLAŞILAN VERİLER
# ============================================================
latest_frame = [None]
lidar_altitude = [None]
tespit_piksel = [None]
islenen_koordinatlar = []  # Daha önce gönderilen koordinatlar
gorev_devam = [True]

# ============================================================
# YOLO MODELİ
# ============================================================
model = YOLO("yolov8x.pt")

# ============================================================
# GAZEBO NODE
# ============================================================
node = Node()

def image_callback(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((msg.height, msg.width, 3))
    latest_frame[0] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def lidar_callback(msg):
    if msg.ranges and msg.ranges[0] != float('inf'):
        lidar_altitude[0] = msg.ranges[0]

node.subscribe(Image, '/camera/image', image_callback)
node.subscribe(LaserScan, '/lidar/range', lidar_callback)

# ============================================================
# YARDIMCI FONKSİYONLAR
# ============================================================
def metre_to_gps(lat, lon, dx, dy):
    R = 6371000
    target_lat = lat + math.degrees(dy / R)
    target_lon = lon + math.degrees(dx / (R * math.cos(math.radians(lat))))
    return target_lat, target_lon

def pixel_to_gps(iha_lat, iha_lon, altitude, heading_deg, pixel_x, pixel_y):
    heading = math.radians(heading_deg)
    norm_x = (pixel_x - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2)
    norm_y = (pixel_y - IMAGE_HEIGHT / 2) / (IMAGE_HEIGHT / 2)
    dx = altitude * math.tan(FOV_H / 2) * norm_x
    dy = altitude * math.tan(FOV_V / 2) * norm_y
    dx_r = dx * math.cos(heading) - dy * math.sin(heading)
    dy_r = dx * math.sin(heading) + dy * math.cos(heading)
    R = 6371000
    target_lat = iha_lat + math.degrees(dy_r / R)
    target_lon = iha_lon + math.degrees(dx_r / (R * math.cos(math.radians(iha_lat))))
    return target_lat, target_lon

def koordinat_islendi_mi(lat, lon, esik=10):
    """Daha önce bu koordinata gidildi mi kontrol eder (10m eşik)"""
    for prev_lat, prev_lon in islenen_koordinatlar:
        dlat = abs(lat - prev_lat) * 111000
        dlon = abs(lon - prev_lon) * 111000 * math.cos(math.radians(lat))
        mesafe = math.sqrt(dlat**2 + dlon**2)
        if mesafe < esik:
            return True
    return False

def grid_waypoints_olustur():
    waypoints = []
    y = 0
    direction = 1
    while y <= ALAN_UZUNLUK:
        x_start = 0 if direction == 1 else ALAN_GENISLIK
        x_end = ALAN_GENISLIK if direction == 1 else 0
        lat, lon = metre_to_gps(HOME_LAT, HOME_LON, x_start, y)
        waypoints.append((lat, lon, TARAMA_IRTIFA))
        lat, lon = metre_to_gps(HOME_LAT, HOME_LON, x_end, y)
        waypoints.append((lat, lon, TARAMA_IRTIFA))
        y += SERIT_ARALIGI
        direction *= -1
    return waypoints

# ============================================================
# YOLO THREAD
# ============================================================
def yolo_thread():
    print("[YOLO] Basliyor...")
    while gorev_devam[0]:
        if latest_frame[0] is not None:
            frame = latest_frame[0].copy()
            results = model(frame, conf=0.1, verbose=False)
            for r in results:
                for box in r.boxes:
                    if int(box.cls[0]) == 0:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        conf = float(box.conf[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, f"KAZAZEDE {conf:.2f}",
                                    (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0, 255, 0), 2)
                        tespit_piksel[0] = (cx, cy)
                        print(f"[YOLO] KAZAZEDE TESPIT! Piksel: ({cx}, {cy})")
            cv2.imshow("YOLO - IHA Kamera", frame)
            cv2.waitKey(1)
    cv2.destroyAllWindows()

# ============================================================
# DRONE GÖREV FONKSİYONU
# ============================================================
async def drone_gonder(target_lat, target_lon):
    print(f"\n[DRONE] Hedefe gonderiliyor: {target_lat:.6f}, {target_lon:.6f}")

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14560")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[DRONE] Baglandi!")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("[DRONE] GPS OK!")
            break

    try:
        await drone.action.arm()
        print("[DRONE] Arm edildi!")
    except:
        print("[DRONE] Zaten arm edilmis")

    await drone.action.set_takeoff_altitude(15)
    try:
        await drone.action.takeoff()
        print("[DRONE] Kalkiyor...")
        await asyncio.sleep(10)
    except:
        print("[DRONE] Zaten havada")
        await asyncio.sleep(3)

    print(f"[DRONE] Hedefe gidiliyor...")
    await drone.action.goto_location(target_lat, target_lon, 15, 0)
    await asyncio.sleep(20)

    print("[DRONE] Hedefe ulasti! Hover...")
    await asyncio.sleep(5)

    print("[DRONE] YUK BIRAKILIYOR!")
    # Gerçekte: await drone.action.set_actuator(1, 1.0)
    await asyncio.sleep(3)

    print("[DRONE] Eve doniyor (RTL)...")
    await drone.action.return_to_launch()
    await asyncio.sleep(15)
    print("[DRONE] Eve dondu!")

# ============================================================
# VTOL İHA GRID TARAMA + KAZAZEDE TESPİT
# ============================================================
async def iha_gorev():
    iha = System()
    await iha.connect(system_address="udpin://0.0.0.0:14550")

    print("[IHA] Baglaniliyor...")
    async for state in iha.core.connection_state():
        if state.is_connected:
            print("[IHA] Baglandi!")
            break

    print("[IHA] GPS bekleniyor...")
    async for health in iha.telemetry.health():
        if health.is_global_position_ok:
            print("[IHA] GPS OK!")
            break

    # Grid waypoint'leri oluştur ve yükle
    waypoints = grid_waypoints_olustur()
    print(f"[IHA] {len(waypoints)} waypoint olusturuldu")

    mission_items = []
    for lat, lon, alt in waypoints:
        mission_items.append(
            MissionItem(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_altitude_m=alt,
                speed_m_s=15,
                is_fly_through=True,
                gimbal_pitch_deg=float('nan'),
                gimbal_yaw_deg=float('nan'),
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=float('nan'),
                camera_photo_interval_s=float('nan'),
                acceptance_radius_m=10,
                yaw_deg=float('nan'),
                camera_photo_distance_m=float('nan')
            )
        )

    mission_plan = MissionPlan(mission_items)

    print("[IHA] Gorev yukleniyor...")
    await iha.mission.set_return_to_launch_after_mission(True)
    await iha.mission.upload_mission(mission_plan)
    print("[IHA] Gorev yuklendi!")

    # Arm ve kalkış
    await iha.action.arm()
    await iha.action.takeoff()
    print("[IHA] Kalkiyor...")
    await asyncio.sleep(15)

    # Grid taramayı başlat
    print("[IHA] GRID TARAMA BASLADI!")
    await iha.mission.start_mission()

    # Tarama sırasında kazazede ara
    while True:
        # Görev tamamlandı mı?
        mission_finished = False
        async for progress in iha.mission.mission_progress():
            print(f"[IHA] Waypoint: {progress.current}/{progress.total}")
            if progress.current == progress.total:
                mission_finished = True
            break

        if mission_finished:
            print("[IHA] GRID TARAMA TAMAMLANDI! RTL...")
            gorev_devam[0] = False
            break

        # Kazazede tespit edildi mi?
        if tespit_piksel[0] is not None:
            pixel_x, pixel_y = tespit_piksel[0]
            tespit_piksel[0] = None  # Sıfırla

            # İHA konumu al
            async for position in iha.telemetry.position():
                iha_lat = position.latitude_deg
                iha_lon = position.longitude_deg
                break

            async for heading in iha.telemetry.heading():
                iha_heading = heading.heading_deg
                break

            # LiDAR irtifa
            if lidar_altitude[0]:
                altitude = lidar_altitude[0]
            else:
                async for position in iha.telemetry.position():
                    altitude = position.relative_altitude_m
                    break

            # Koordinat hesapla
            target_lat, target_lon = pixel_to_gps(
                iha_lat, iha_lon, altitude, iha_heading, pixel_x, pixel_y
            )

            # Daha önce gidildi mi?
            if koordinat_islendi_mi(target_lat, target_lon):
                print("[IHA] Bu kazazede zaten islendi, devam...")
                continue

            print(f"[IHA] Kazazede koordinati: {target_lat:.6f}, {target_lon:.6f}")

            # Koordinatı kaydet
            islenen_koordinatlar.append((target_lat, target_lon))

            # İHA görevi durdur
            await iha.mission.pause_mission()
            print("[IHA] Gorev duraklatildi, drone gonderiliyor...")

            # Drone'u gönder
            await drone_gonder(target_lat, target_lon)

            # İHA göreve devam et
            print("[IHA] Gorev devam ediyor...")
            await iha.mission.start_mission()

        await asyncio.sleep(1)

# ============================================================
# ANA FONKSİYON
# ============================================================
async def main():
    print("=" * 50)
    print("  TEKNOFEST OTONOM ARAMA KURTARMA SİSTEMİ")
    print("=" * 50)

    # YOLO thread başlat
    t = threading.Thread(target=yolo_thread, daemon=True)
    t.start()

    # İHA görevini başlat
    await iha_gorev()

if __name__ == "__main__":
    asyncio.run(main())
EOF
echo "Hazir! python3 ~/otonom_gorev.py ile calistirin"
