using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ihaarayuz
{
    public class DikeyGosterge : Control
    {
        // Basınç genelde deniz seviyesinde 1013 hPa civarıdır, sınırları ona göre belirledik
        public float Maksimum { get; set; } = 1100;
        public float Minimum { get; set; } = 900;
        public string Birim { get; set; } = "hPa";
      

        private float _deger = 1013;
        public float Deger
        {
            get { return _deger; }
            set
            {
                _deger = value;
                this.Invalidate(); // Değer gelince barı anında hareket ettir
            }
        }

        public DikeyGosterge()
        {
            this.DoubleBuffered = true; 
            this.Size = new Size(80, 220); // Dikey ince uzun bir tasarım
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            // Ana Çerçeve (Tüp)
            Rectangle barAlani = new Rectangle(20, 30, this.Width - 40, this.Height - 75);
            g.FillRectangle(Brushes.WhiteSmoke, barAlani);
            g.DrawRectangle(new Pen(Color.DarkSlateGray, 2), barAlani);

           

            // Suyun (Barın) Yükselme Hesabı
            float oran = (Deger - Minimum) / (Maksimum - Minimum);
            oran = Math.Max(0, Math.Min(1, oran)); // Taşmayı engeller

            int dolguYuksekligi = (int)(barAlani.Height * oran);
            Rectangle dolguAlani = new Rectangle(barAlani.X, barAlani.Y + barAlani.Height - dolguYuksekligi, barAlani.Width, dolguYuksekligi);
            
            // Mavi Dolgu Rengi
            g.FillRectangle(Brushes.DodgerBlue, dolguAlani);

            // Alt Kısma Dijital Değeri ve Birimi Yazdırma
            string altYazi = Deger.ToString("0.0") + "\n" + Birim;
            Font altYaziFontu = new Font("Arial", 10, FontStyle.Bold);
            SizeF altYaziBoyutu = g.MeasureString(altYazi, altYaziFontu);
            
            // Metni tam ortalayarak alt kısma yazıyoruz
            g.DrawString(altYazi, altYaziFontu, Brushes.DarkBlue, (this.Width - altYaziBoyutu.Width) / 2, barAlani.Bottom + 5);
        }
    }
}