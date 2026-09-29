using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ihaarayuz
{
    public class UfukCizgisi : Control
    {
        // --- Değişkenler ve Otomatik Yenileme Özellikleri ---
        private float _roll = 0;
        private float _pitch = 0;
        private float _hiz = 0;
        private float _irtifa = 0;
        private float _heading = 0;

        public float Roll
        {
            get => _roll;
            set { if (_roll != value) { _roll = value; Invalidate(); } }
        }

        public float Pitch
        {
            get => _pitch;
            set { if (_pitch != value) { _pitch = value; Invalidate(); } }
        }

        public float Hiz
        {
            get => _hiz;
            set { if (_hiz != value) { _hiz = value; Invalidate(); } }
        }

        public float Irtifa
        {
            get => _irtifa;
            set { if (_irtifa != value) { _irtifa = value; Invalidate(); } }
        }

        public float Heading
        {
            get => _heading;
            set { if (_heading != value) { _heading = value; Invalidate(); } }
        }

        public UfukCizgisi()
        {
            this.DoubleBuffered = true;
            this.Size = new Size(400, 300);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;

            int w = Width, h = Height;
            int cx = w / 2, cy = h / 2;

            // --- 1. ARKA PLAN: GÖKYÜZÜ VE YER ---
            g.TranslateTransform(cx, cy);
            g.RotateTransform(-Roll);
            int pOffset = (int)(Pitch * 6);

            using (var sky = new LinearGradientBrush(new Point(0, -h), new Point(0, pOffset), Color.FromArgb(0, 122, 204), Color.FromArgb(135, 206, 235)))
                g.FillRectangle(sky, -w * 2, -h * 2, w * 4, h * 2 + pOffset);

            using (var ground = new LinearGradientBrush(new Point(0, pOffset), new Point(0, h), Color.FromArgb(107, 142, 35), Color.FromArgb(34, 139, 34)))
                g.FillRectangle(ground, -w * 2, pOffset, w * 4, h * 2);

            // Yunuslama (Pitch) Çizgileri
            Pen whitePen = new Pen(Color.White, 2);
            Font fSmall = new Font("Arial", 8, FontStyle.Bold);
            for (int i = -90; i <= 90; i += 10)
            {
                if (i == 0) continue;
                int y = pOffset - (int)(i * 6);
                int lw = (i % 20 == 0) ? 40 : 20; // Çizgi boylarını kısalttık
                g.DrawLine(whitePen, -lw / 2, y, lw / 2, y);
                g.DrawString(i.ToString(), fSmall, Brushes.White, (lw / 2) + 5, y - 7);
                g.DrawString(i.ToString(), fSmall, Brushes.White, (-lw / 2) - 22, y - 7);
            }
            g.DrawLine(new Pen(Color.White, 2), -w, pOffset, w, pOffset);

            // --- 2. YATIŞ (ROLL) CETVELİ (DAHA FERAH DİZİLİM) ---
            int rollRadius = 150; // Yarıçapı artırarak derecelerin arasını açtık
            int[] rollAngles = { -60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60 };

            foreach (int angle in rollAngles)
            {
                double angleRad = (angle - 90) * (Math.PI / 180.0);

                int xStart = (int)(Math.Cos(angleRad) * rollRadius);
                int yStart = (int)(Math.Sin(angleRad) * rollRadius);

                int tickLen = (angle % 20 == 0) ? 10 : 5;
                int xEnd = (int)(Math.Cos(angleRad) * (rollRadius - tickLen));
                int yEnd = (int)(Math.Sin(angleRad) * (rollRadius - tickLen));

                g.DrawLine(whitePen, xStart, yStart, xEnd, yEnd);

                // Sadece ana derecelere yazı yazarak kalabalığı önleyebiliriz 
                // ya da hepsini yazıp konumu dışa itebiliriz:
                string text = Math.Abs(angle).ToString();
                SizeF size = g.MeasureString(text, fSmall);

                int xText = (int)(Math.Cos(angleRad) * (rollRadius + 20)) - (int)(size.Width / 2);
                int yText = (int)(Math.Sin(angleRad) * (rollRadius + 20)) - (int)(size.Height / 2);

                g.DrawString(text, fSmall, Brushes.White, xText, yText);
            }

            g.ResetTransform();

            // --- 3. SABİT ROLL GÖSTERGESİ (KIRMIZI ÜÇGEN) ---
            Point[] triangle = {
                new Point(cx, cy - rollRadius - 5),
                new Point(cx - 6, cy - rollRadius + 8),
                new Point(cx + 6, cy - rollRadius + 8)
            };
            g.FillPolygon(Brushes.Red, triangle);

            // --- 4. SOL HIZ VE SAĞ İRTİFA CETVELİ (İNCELTİLMİŞ) ---
            var bgBrush = new SolidBrush(Color.FromArgb(100, 0, 0, 0));
            int boxWidth = 35; // Genişliği 40'tan 35'e düşürdük

            // Hız Kutusu
            g.FillRectangle(bgBrush, 12, 60, boxWidth, h - 120);
            g.DrawRectangle(Pens.White, 12, 60, boxWidth, h - 120);
            g.FillRectangle(Brushes.Black, 5, cy - 12, boxWidth + 12, 24);
            g.DrawString(Hiz.ToString("0.0"), fSmall, Brushes.White, 8, cy - 7);

            // İrtifa Kutusu
            g.FillRectangle(bgBrush, w - 47, 60, boxWidth, h - 120);
            g.DrawRectangle(Pens.White, w - 47, 60, boxWidth, h - 120);
            g.FillRectangle(Brushes.Black, w - 52, cy - 12, boxWidth + 12, 24);
            g.DrawString(Irtifa.ToString("0"), fSmall, Brushes.White, w - 45, cy - 7);

            // --- 5. ÜST PUSULA ŞERİDİ ---
            g.FillRectangle(bgBrush, 60, 5, w - 120, 25);
            g.DrawString("HDG: " + Heading.ToString("0") + "°", fSmall, Brushes.Orange, cx - 25, 10);
            g.FillPolygon(Brushes.Red, new Point[] { new Point(cx, 30), new Point(cx - 5, 36), new Point(cx + 5, 36) });

            // --- 6. MERKEZ SABİT SEMBOL ---
            Pen redP = new Pen(Color.Red, 2);
            g.DrawLine(redP, cx - 35, cy, cx - 12, cy);
            g.DrawLine(redP, cx + 12, cy, cx + 35, cy);
            g.DrawLine(redP, cx - 12, cy, cx, cy + 8);
            g.DrawLine(redP, cx, cy + 8, cx + 12, cy);
        }
    }
}