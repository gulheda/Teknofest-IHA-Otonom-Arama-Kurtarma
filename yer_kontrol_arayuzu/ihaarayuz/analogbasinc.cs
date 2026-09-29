using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ihaarayuz
{
    public class AnalogGostergeBasinc : Control
    {
        // Basınca özel standart sınırlar
        public float Maksimum { get; set; } = 1100;
        public float Minimum { get; set; } = 900;
        public string Birim { get; set; } = "hPa";
        public string Baslik { get; set; } = "BASINÇ";

        // Başlangıçta 0 gelsin diye ayarlıyoruz
        private float _deger = 0;
        public float Deger
        {
            get { return _deger; }
            set
            {
                _deger = value;
                this.Invalidate(); // Veri gelince anında hareket et
            }
        }

        public AnalogGostergeBasinc()
        {
            this.DoubleBuffered = true; 
            this.Size = new Size(180, 180); 
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            Rectangle rect = new Rectangle(10, 10, this.Width - 20, this.Height - 20);

            // Çerçeve
            g.FillEllipse(Brushes.WhiteSmoke, rect);
            g.DrawEllipse(new Pen(Color.DarkSlateGray, 4), rect);

            PointF merkez = new PointF(this.Width / 2f, this.Height / 2f);
            float yaricap = (this.Width - 40) / 2f;

            // --- ÜST KISMA BASINÇ BAŞLIĞI YAZDIRMA ---
            Font baslikFont = new Font("Arial", 9, FontStyle.Bold);
            SizeF baslikBoyut = g.MeasureString(Baslik, baslikFont);
            g.DrawString(Baslik, baslikFont, Brushes.DarkBlue, merkez.X - (baslikBoyut.Width / 2), merkez.Y - yaricap + 15);

            float aciAraligi = 270f;
            float baslangicAcisi = 135f;

            // --- SAYILARI 4 PARÇADA ÇİZME (900, 950, 1000, 1050, 1100) ---
            int bolmeSayisi = 4; 
            Font sayiFontu = new Font("Arial", 8, FontStyle.Bold);

            for (int i = 0; i <= bolmeSayisi; i++)
            {
                float anlikAci = baslangicAcisi + (aciAraligi * i / bolmeSayisi);
                double rad = anlikAci * Math.PI / 180.0;
                float yazilacakDeger = Minimum + ((Maksimum - Minimum) * i / bolmeSayisi);
                
                string sayiMetni = Math.Round(yazilacakDeger).ToString();
                SizeF metinBoyutu = g.MeasureString(sayiMetni, sayiFontu);

                float sayiX = merkez.X + (float)((yaricap + 2) * Math.Cos(rad)) - (metinBoyutu.Width / 2);
                float sayiY = merkez.Y + (float)((yaricap + 2) * Math.Sin(rad)) - (metinBoyutu.Height / 2);

                g.DrawString(sayiMetni, sayiFontu, Brushes.Black, sayiX, sayiY);
            }

            // İbre Açı Hesaplaması
            float oran = (Deger - Minimum) / (Maksimum - Minimum);
            oran = Math.Max(0, Math.Min(1, oran)); 
            float gecerliAci = baslangicAcisi + (oran * aciAraligi);

            // --- İBRE ÇİZİMİ (Mavi Renk) ---
            double ibreRadyan = gecerliAci * Math.PI / 180.0;
            float ibreX = merkez.X + (float)((yaricap - 12) * Math.Cos(ibreRadyan));
            float ibreY = merkez.Y + (float)((yaricap - 12) * Math.Sin(ibreRadyan));

            // Basınç için Mavi ibre daha şık durur
            g.DrawLine(new Pen(Color.DodgerBlue, 3), merkez.X, merkez.Y, ibreX, ibreY);
            g.FillEllipse(Brushes.Black, merkez.X - 6, merkez.Y - 6, 12, 12);

            // Alt Kısma Dijital Veri
            string altYazi = Deger.ToString("0.0") + " " + Birim;
            Font altYaziFontu = new Font("Arial", 10, FontStyle.Bold);
            SizeF altYaziBoyutu = g.MeasureString(altYazi, altYaziFontu);
            g.DrawString(altYazi, altYaziFontu, Brushes.DarkBlue, merkez.X - (altYaziBoyutu.Width / 2), merkez.Y + (yaricap / 2));
        }
    }
}