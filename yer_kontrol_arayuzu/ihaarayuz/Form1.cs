using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using System.Net;
using System.Net.Sockets;
using System.Threading;

using GMap.NET;
using GMap.NET.MapProviders;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;


namespace ihaarayuz
{
    public class GMapYönlüMarker : GMapMarker
    {
        private Color _renk;
        private float _aci;

        public GMapYönlüMarker(PointLatLng nokta, Color renk, float aci) : base(nokta)
        {
            _renk = renk;
            _aci = aci;
            this.Size = new Size(24, 24);
            this.Offset = new Point(-12, -12); // Tam merkezleme
        }

        public override void OnRender(Graphics g)
        {
            // Çizim kalitesini artırır (Yumuşatma)
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            // Mevcut grafik matrisini koru
            System.Drawing.Drawing2D.GraphicsState state = g.Save();

            // 1. Matrisi marker'ın haritadaki merkez koordinatına taşı
            g.TranslateTransform(LocalPosition.X + 12, LocalPosition.Y + 12);

            // 2. Pusula açısı kadar matrisi döndür (Mission Planner gibi)
            g.RotateTransform(_aci);

            // 3. Döndürülmüş matrisin merkezinde (0,0 bağıl konumunda) İHA/Uçak şeklini çiz
            using (Brush brush = new SolidBrush(_renk))
            {
                // Tam senin tasarladığın askeri havacılık oku/üçgeni koordinatları
                Point[] noktalar = new Point[]
                {
                new Point(0, -12),   // Tepe noktası (Burun yönü)
                new Point(-10, 12),  // Sol kuyruk kanadı
                new Point(0, 6),     // İç çöküntü
                new Point(10, 12)    // Sağ kuyruk kanadı
                };
                g.FillPolygon(brush, noktalar);

                // Mission Planner'daki o ikonun ortasındaki kırmızı artı (+) işaretini çizelim
                using (Pen kırmızıKalem = new Pen(Color.Red, 2))
                {
                    g.DrawLine(kırmızıKalem, -4, 2, 4, 2); // Yatay çizgi
                    g.DrawLine(kırmızıKalem, 0, -2, 0, 6); // Dikey çizgi
                }
            }

            // Grafik matrisini eski haline geri yükle (Haritanın kalanı bozulmasın diye)
            g.Restore(state);
        }
    }

    public partial class Form1 : Form
    {
        // Araç 1 (VTOL İHA) için nesneler
        UdpClient dinleyiciVTOL;
        Thread kanalVTOL;

        // Araç 2 (Drone) için nesneler
        UdpClient dinleyiciDrone;
        Thread kanalDrone;

        bool dinlemeyeDevamEt = false;
        DateTime sonVeriZamani = DateTime.Now;

        float dengeLimiti = 45.0f; // 45 dereceden fazla eğilirse alarm çalacak

        GMapOverlay vtolKatmani = new GMapOverlay("vtol");
        GMapOverlay droneKatmani = new GMapOverlay("drone");
        GMapOverlay hedefKatmani = new GMapOverlay("hedef");

        float vtolAcisi = 0.0f;
        float droneAcisi = 0.0f;

        public Form1()
        {
            InitializeComponent();
            IhaBataryaGostergesi.Yuzde = 0;
            DroneBataryaGostergesi.Yuzde = 0;
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // Harita ayarları
            gMapControl1.MapProvider = GMap.NET.MapProviders.GoogleMapProvider.Instance; // İnternet varsa Google, yoksa OpenStreetMap deneyebilirsin
            GMap.NET.GMaps.Instance.Mode = GMap.NET.AccessMode.ServerOnly;
            gMapControl1.Position = new GMap.NET.PointLatLng(41.015, 28.979); // İstanbul koordinatları ile başlar
            gMapControl1.MinZoom = 5;
            gMapControl1.MaxZoom = 20;
            gMapControl1.Zoom = 13; // Yakınlık seviyesi
            gMapControl1.DragButton = MouseButtons.Left; // Sol tıkla haritayı kaydırabilme

            // Katmanları haritaya bir kez ekliyoruz
            gMapControl1.Overlays.Add(vtolKatmani);
            gMapControl1.Overlays.Add(droneKatmani);
            gMapControl1.Overlays.Add(hedefKatmani);

        }

        private void dateTimePicker1_ValueChanged(object sender, EventArgs e)
        {
            
        }

        private void pictureBox2_Click(object sender, EventArgs e)
        {
        }

        private void chart2_Click(object sender, EventArgs e)
        {
        }

        private void groupBox4_Enter(object sender, EventArgs e)
        {
        }

        // BAŞLA BUTONU KODLARI
        private void button1_Click(object sender, EventArgs e)
        {// Çift tıklama koruması: Zaten dinleme yapıyorsa butonu yoksay (çakışmayı önler)
            if (dinlemeyeDevamEt == true)
            {
                return;
            }

            try
            {
                dinlemeyeDevamEt = true;
                sonVeriZamani = DateTime.Now; // Sayacı tam bağlantı anında sıfırlıyoruz

                // --- VTOL İHA BAĞLANTISI (Port: 14552) ---
                dinleyiciVTOL = new UdpClient();
                // PORT KİLİDİ ÇÖZÜMÜ: Kapat-Aç yapıldığında portun anında tekrar kullanılabilmesini sağlar
                dinleyiciVTOL.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                dinleyiciVTOL.Client.Bind(new IPEndPoint(IPAddress.Any, 14552));

                kanalVTOL = new Thread(() => VeriDinle(dinleyiciVTOL, 1)); // 1: VTOL ID
                kanalVTOL.IsBackground = true;
                kanalVTOL.Start();

                // --- DRONE BAĞLANTISI (Port: 14561) ---
                dinleyiciDrone = new UdpClient();
                // PORT KİLİDİ ÇÖZÜMÜ: Drone portu için de aynısını uyguluyoruz
                dinleyiciDrone.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                dinleyiciDrone.Client.Bind(new IPEndPoint(IPAddress.Any, 14561));

                kanalDrone = new Thread(() => VeriDinle(dinleyiciDrone, 2)); // 2: Drone ID
                kanalDrone.IsBackground = true;
                kanalDrone.Start();
            }
            catch (Exception ex)
            {
                dinlemeyeDevamEt = false;
                MessageBox.Show("Bağlantı başlatılamadı! Portlar meşgul olabilir.\n\nHata Detayı: " + ex.Message, "Bağlantı Hatası", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }


        // ARKA PLANDA VERİ YAKALAMA VE ÇÖZÜMLEME KODLARI
        private void VeriDinle(UdpClient istemci, int aracID)
        {
            IPEndPoint ipep = new IPEndPoint(IPAddress.Any, 0);
            MAVLink.MavlinkParse cozumleyici = new MAVLink.MavlinkParse();

            while (dinlemeyeDevamEt)
            {
                try
                {
                    byte[] gelenVeri = istemci.Receive(ref ipep);
                    sonVeriZamani = DateTime.Now;
                    System.IO.MemoryStream akis = new System.IO.MemoryStream(gelenVeri);
                    MAVLink.MAVLinkMessage paket = cozumleyici.ReadPacket(akis);

                    if (paket != null)
                    {
                        // BATARYA VE ALARM KONTROLÜ (SYS_STATUS)
                        if (paket.msgid == (uint)MAVLink.MAVLINK_MSG_ID.SYS_STATUS)
                        {
                            var status = (MAVLink.mavlink_sys_status_t)paket.data;
                            int pilDegeri = (status.battery_remaining == -1 || status.battery_remaining > 100) ? 0 : (int)status.battery_remaining;

                            this.Invoke((MethodInvoker)delegate {
                                if (aracID == 1) // Sadece İHA (VTOL) Bataryası
                                {
                                    IhaBataryaGostergesi.Yuzde = pilDegeri;
                                    panel7.BackColor = (pilDegeri <= 20 && pilDegeri > 0) ? Color.Red : Color.White;
                                }
                                else if (aracID == 2) // Drone Bataryası
                                {
                                    DroneBataryaGostergesi.Yuzde = pilDegeri;
                                    panel9.BackColor = (pilDegeri <= 20 && pilDegeri > 0) ? Color.Red : Color.White;
                                }
                            });
                        }

                        // TUTUM (ROLL/PITCH) VE UÇUŞ DENGESİZLİĞİ ALARMI
                        else if (paket.msgid == (uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                        {
                            var att = (MAVLink.mavlink_attitude_t)paket.data;
                            float rollDeg = (float)att.roll * (180.0f / (float)Math.PI);
                            float pitchDeg = (float)att.pitch * (180.0f / (float)Math.PI);

                            this.Invoke((MethodInvoker)delegate {
                                if (aracID == 1)
                                {
                                    ufukCizgisi1.Roll = rollDeg;
                                    ufukCizgisi1.Pitch = pitchDeg;

                                    if (Math.Abs(rollDeg) > dengeLimiti || Math.Abs(pitchDeg) > dengeLimiti)
                                    {
                                        panel12.BackColor = Color.Red;
                                    }
                                    else
                                    {
                                        panel12.BackColor = Color.White;
                                    }
                                }
                            });
                        }

                        // HIZ VE PUSULA (VFR_HUD)
                        else if (paket.msgid == (uint)MAVLink.MAVLINK_MSG_ID.VFR_HUD)
                        {
                            var hud = (MAVLink.mavlink_vfr_hud_t)paket.data;
                            this.Invoke((MethodInvoker)delegate {
                                if (aracID == 1)
                                {
                                    analogGosterge2.Deger = hud.airspeed;
                                    ufukCizgisi1.Hiz = hud.airspeed;
                                    ufukCizgisi1.Heading = hud.heading;
                                }
                                else if (aracID == 2)
                                {
                                    analogGosterge1.Deger = hud.airspeed;
                                }
                            });
                        }

                        // BASINÇ VE SICAKLIK KONTROLÜ (SCALED_PRESSURE)
                        else if (paket.msgid == (uint)MAVLink.MAVLINK_MSG_ID.SCALED_PRESSURE)
                        {
                            var basincVerileri = (MAVLink.mavlink_scaled_pressure_t)paket.data;
                            float anlikBasinc = basincVerileri.press_abs;
                            float anlikSicaklik = basincVerileri.temperature / 100.0f;

                            this.Invoke((MethodInvoker)delegate {
                                if (aracID == 1)
                                {
                                    analogGostergeBasinc1.Deger = anlikBasinc;
                                    analogGostergeSicaklik1.Deger = anlikSicaklik;
                                }
                                else if (aracID == 2)
                                {
                                    analogGostergeBasinc2.Deger = anlikBasinc;
                                    analogGostergeSicaklik2.Deger = anlikSicaklik;
                                }
                            });
                        }

                        // KONUM VE KOORDİNATLAR (GLOBAL_POSITION_INT)
                        else if (paket.msgid == (uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT)
                        {
                            var pos = (MAVLink.mavlink_global_position_int_t)paket.data;
                            double lat = pos.lat / 10000000.0;
                            double lon = pos.lon / 10000000.0;
                            float alt = pos.relative_alt / 1000.0f;

                            // Pusula açısını doğrudan konum paketinin içinden çözüyoruz
                            float anlikAci = pos.hdg == 65535 ? 0.0f : pos.hdg / 100.0f;

                            this.Invoke((MethodInvoker)delegate {
                                if (aracID == 1) // Sadece İHA (aracID == 1) haritada işlensin
                                {
                                    txtEnlem.Text = lat.ToString("F7");
                                    txtBoylam.Text = lon.ToString("F7");
                                    analogGostergeIrtifa1.Deger = alt;
                                    ufukCizgisi1.Irtifa = alt;

                                    // HARİTA GÜNCELLEME (Canlı Dönen Mavi İHA Oku)
                                    vtolKatmani.Markers.Clear();
                                    GMapYönlüMarker marker = new GMapYönlüMarker(new PointLatLng(lat, lon), Color.DodgerBlue, anlikAci);
                                    vtolKatmani.Markers.Add(marker);

                                    // Harita kamerasını İHA'nın üzerine kilitler
                                    gMapControl1.Position = new PointLatLng(lat, lon);
                                }
                                else if (aracID == 2) // Drone verileri sadece sayısal kutulara yazsın, haritayı bozmasın
                                {
                                    analogGostergeIrtifa2.Deger = alt;
                                }

                                // Haritayı akıcı bir şekilde arka planda tazeler
                                gMapControl1.Invalidate();
                            });
                        }

                        // HEDEF TESPİT ALARMI
                        else if (paket.msgid == (uint)MAVLink.MAVLINK_MSG_ID.LANDING_TARGET && aracID == 1)
                        {
                            var target = (MAVLink.mavlink_landing_target_t)paket.data;
                            this.Invoke((MethodInvoker)delegate {
                                txtHedefEnlem.Text = target.x.ToString("F7");
                                txtHedefBoylam.Text = target.y.ToString("F7");
                                panel10.BackColor = Color.Green;

                                hedefKatmani.Markers.Clear();
                                GMapMarker hedefMarker = new GMarkerGoogle(new PointLatLng(target.x, target.y), GMarkerGoogleType.red);
                                hedefKatmani.Markers.Add(hedefMarker);

                                gMapControl1.Invalidate();
                            });
                        }

                        // HUD Ekran Tazeleme
                        if (aracID == 1)
                        {
                            this.Invoke((MethodInvoker)delegate { ufukCizgisi1.Invalidate(); });
                        }
                    }
                }
                catch
                {
                    if (!dinlemeyeDevamEt) break;
                }
            }
        }
        private void DurdurmaIslemi()
        {
            dinlemeyeDevamEt = false;

            if (dinleyiciVTOL != null) { dinleyiciVTOL.Close(); dinleyiciVTOL = null; }
            if (dinleyiciDrone != null) { dinleyiciDrone.Close(); dinleyiciDrone = null; }
        }

        // DURDUR BUTONU KODLARI
        private void durdurbtn_Click(object sender, EventArgs e)
        {
            DurdurmaIslemi();
        }

        // PENCERE ÇARPIDAN KAPATILIRSA ÇALIŞACAK KODLAR
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            DurdurmaIslemi();
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            // Saati güncellediğin yer
            dateTimePicker1.Value = DateTime.Now;

            if (dinlemeyeDevamEt)
            {
                TimeSpan fark = DateTime.Now - sonVeriZamani;

                if (fark.TotalSeconds > 3)
                {
                    panel8.BackColor = Color.Red;
                }
                else
                {
                    panel8.BackColor = Color.White;
                }
            }
            else
            {
                // Bağlantı yokken panel beyaz kalsın
                panel8.BackColor = Color.White;
            }
        }
    }
}