using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ihaarayuz
{
    public class BataryaGostergesi : Control
    {
        // Başlangıç değerini kesin olarak 0 yapıyoruz
        private int _yuzde = 0;

        public int Yuzde
        {
            get => _yuzde;
            set
            {
                // Değer 0-100 arasında sınırlandırılıyor
                _yuzde = Math.Max(0, Math.Min(100, value));
                Invalidate(); // Her değer değişiminde görseli yeniden çizer
            }
        }

        public BataryaGostergesi()
        {
            // Titremeyi önlemek için DoubleBuffered aktif
            this.DoubleBuffered = true;
            this.Size = new Size(80, 40);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;

            int w = Width;
            int h = Height;

            // 1. DIŞ ÇERÇEVE (BEYAZ)
            // Siyah hatlar tamamen kaldırıldı, beyaz çerçeve eklendi
            Pen cercevePen = new Pen(Color.White, 3);
            GraphicsPath path = new GraphicsPath();
            int r = 4; // Köşe yumuşatma yarıçapı

            path.AddArc(2, 2, r, r, 180, 90);
            path.AddArc(w - 15, 2, r, r, 270, 90);
            path.AddArc(w - 15, h - 5, r, r, 0, 90);
            path.AddArc(2, h - 5, r, r, 90, 90);
            path.CloseFigure();
            g.DrawPath(cercevePen, path);

            // Pil başlığı (Sağdaki beyaz çıkıntı)
            g.FillRectangle(Brushes.White, w - 10, h / 3, 6, h / 3);

            // 2. DİŞ (KUTUCUK) HESAPLAMA
            int disSayisi = 0;
            Color disRengi = Color.Gray;

            // %0 durumunda disSayisi 0 kalır ve içi boş görünür
            if (_yuzde > 75) { disSayisi = 4; disRengi = Color.LimeGreen; }
            else if (_yuzde > 50) { disSayisi = 3; disRengi = Color.Yellow; }
            else if (_yuzde > 25) { disSayisi = 2; disRengi = Color.Orange; }
            else if (_yuzde > 0) { disSayisi = 1; disRengi = Color.Red; }

            // 3. DİŞLERİ ÇİZ
            int disGenisligi = (w - 25) / 4;
            int disYuksekligi = h - 12;

            for (int i = 0; i < disSayisi; i++)
            {
                int x = 6 + (i * (disGenisligi + 2));
                int y = 6;

                using (LinearGradientBrush brush = new LinearGradientBrush(
                    new Rectangle(x, y, disGenisligi, disYuksekligi),
                    disRengi,
                    Color.FromArgb(200, disRengi),
                    LinearGradientMode.Vertical))
                {
                    g.FillRectangle(brush, x, y, disGenisligi, disYuksekligi);
                }
            }

            // 4. YÜZDE METNİ
            string text = "%" + _yuzde;
            using (Font font = new Font("Segoe UI", 9, FontStyle.Bold))
            {
                SizeF textSize = g.MeasureString(text, font);

                // Metnin her koşulda okunması için çift katmanlı çizim (Gölge efekti)
                // Arka plan gölgesi
                g.DrawString(text, font, Brushes.Black, (w / 2) - (textSize.Width / 2) - 4, (h / 2) - (textSize.Height / 2) + 1);
                // Ön plan beyaz yazı
                g.DrawString(text, font, Brushes.White, (w / 2) - (textSize.Width / 2) - 5, (h / 2) - (textSize.Height / 2));
            }
        }
    }
}