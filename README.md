# TEKNOFEST İHA Otonom Arama-Kurtarma Çalışması

Bu depo, ekibimizin TEKNOFEST kapsamında geliştirdiği önceki İHA çalışmalarının teknik çıktılarından bir bölümünü içerir. Çalışmanın amacı, geniş bir alanı tarayan bir hava aracı ile hedef bölgeye otonom olarak ilerleyen multikopteri aynı arama-kurtarma senaryosu içerisinde değerlendirmekti.

Bu çalışma, güncel bitirme projemiz olan GPS/GNSS erişiminin bulunmadığı ortamlarda tam otonom keşif ve navigasyon çalışmasından ayrıdır. Repo, daha önce İHA sistemleri, otonom uçuş, görüntü işleme, görev yazılımı ve yer kontrol arayüzü üzerinde edindiğimiz uygulamalı deneyimi göstermek amacıyla korunmaktadır.

## Gerçekleştirilen çalışmalar

Ana İHA'nın mekanik üretimi ve elektronik sistem entegrasyonu gerçekleştirildi. Ancak ana İHA'nın motorunda meydana gelen teknik arıza nedeniyle fiziksel uçuş testi tamamlanamadı.

Otonom uçuş testleri projenin multikopter platformu üzerinde gerçekleştirildi. Multikopter üzerinde otonom kalkış, hedef noktaya ilerleme, yön/konum kontrolü ve görev senaryoları fiziksel olarak test edildi.

Yazılım ve simülasyon tarafında ise aşağıdaki konular üzerinde çalışıldı:

- ArduPilot ve SITL tabanlı uçuş simülasyonu
- Gazebo Harmonic ile iki araçlı görev senaryosu
- MAVSDK/Python ile otonom görev kontrolü
- YOLOv8 tabanlı insan tespiti
- LiDAR irtifa verisinin işlenmesi
- Kamera piksel koordinatından GPS hedef koordinatı hesaplama
- Hedefe yönelme ve RTL görev akışları
- C# / Windows Forms tabanlı yer kontrol arayüzü
- MAVLink telemetrisi üzerinden konum, irtifa, hız, tutum ve batarya verilerinin görselleştirilmesi

## Sistem yaklaşımı

Simülasyon senaryosunda VTOL platformun alan taraması yapması, kamera görüntüsünden insan tespiti gerçekleştirmesi ve hedef konumunu hesaplaması; multikopterin ise belirlenen hedefe otonom olarak ilerlemesi üzerine çalışılmıştır.

```text
VTOL / arama platformu
  -> alan tarama
  -> görüntü alma
  -> YOLOv8 ile kişi tespiti
  -> LiDAR + konum + kamera geometrisi ile hedef konumu

Multikopter
  -> otonom kalkış
  -> hedef noktaya ilerleme
  -> görev bölgesinde konumlanma
  -> RTL / görev sonlandırma
```

## Proje durumu

| Bileşen | Durum |
| --- | --- |
| Ana İHA mekanik üretimi | Tamamlandı |
| Ana İHA elektronik entegrasyonu | Tamamlandı |
| Ana İHA fiziksel uçuşu | Motor arızası nedeniyle tamamlanamadı |
| Multikopter otonom uçuş testleri | Fiziksel olarak gerçekleştirildi |
| ArduPilot / Gazebo simülasyonu | Gerçekleştirildi |
| YOLO tabanlı hedef tespiti | Yazılım ve simülasyon çalışmaları gerçekleştirildi |
| Piksel -> GPS hedef hesabı | Prototip geliştirildi |
| Yer kontrol arayüzü | Geliştirildi |

## Otonom uçuş videosu

Multikopter üzerinde gerçekleştirilen otonom uçuş çalışmalarından kısa bir örnek:

https://youtube.com/shorts/KACo_1uFrfQ

## Repo yapısı

```text
.
├── otonom_gorev.py
│   └── İki araçlı görev senaryosu, grid tarama ve hedef tespiti
├── koordinat_hesapla.py
│   └── Kamera pikseli + irtifa + heading verisinden hedef GPS hesabı
├── drone_gonder.py
│   └── Multikopter hedef uçuşu ve RTL akışı
├── yer_kontrol_arayuzu/
│   └── C# Windows Forms tabanlı telemetri ve yer kontrol arayüzü
├── requirements.txt
└── README.md
```

## Kullanılan teknolojiler

- ArduPilot SITL
- Gazebo Harmonic
- MAVSDK / MAVLink
- Python
- OpenCV
- Ultralytics YOLOv8
- LiDAR
- C# / .NET Framework
- Windows Forms
- GMap.NET

## Python bağımlılıkları

Python paketleri:

```bash
pip install -r requirements.txt
```

Gazebo Transport Python bağlayıcıları, ArduPilot SITL ve Gazebo Harmonic ayrıca kurulmalıdır.

## Yer kontrol arayüzü

`yer_kontrol_arayuzu` klasörü, VTOL ve multikopter telemetrisini izlemek amacıyla geliştirdiğimiz masaüstü arayüzünü içerir. Arayüzde harita üzerinde araç konumu, irtifa, hız, batarya, basınç, sıcaklık ve attitude verilerinin görüntülenmesine yönelik bileşenler bulunmaktadır.

Visual Studio ile açmak için:

```text
yer_kontrol_arayuzu/ihaarayuz.sln
```

Bağımlılıklar `packages.config` üzerinden NuGet ile geri yüklenebilir.

## Not

Bu depo bir yarışma/prototip geliştirme sürecindeki teknik çalışmaları içerir; üretim seviyesinde uçuş yazılımı olarak değerlendirilmemelidir. Fiziksel olarak doğrulanan çalışmalar ile simülasyon/prototip seviyesinde kalan çalışmalar yukarıdaki proje durumu bölümünde özellikle ayrılmıştır.
