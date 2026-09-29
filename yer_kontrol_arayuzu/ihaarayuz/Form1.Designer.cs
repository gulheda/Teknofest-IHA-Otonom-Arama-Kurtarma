namespace ihaarayuz
{
    partial class Form1
    {
        /// <summary>
        ///Gerekli tasarımcı değişkeni.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///Kullanılan tüm kaynakları temizleyin.
        /// </summary>
        ///<param name="disposing">yönetilen kaynaklar dispose edilmeliyse doğru; aksi halde yanlış.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer üretilen kod

        /// <summary>
        /// Tasarımcı desteği için gerekli metot - bu metodun 
        ///içeriğini kod düzenleyici ile değiştirmeyin.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.basla = new System.Windows.Forms.Button();
            this.ihakamera = new System.Windows.Forms.PictureBox();
            this.durdurbtn = new System.Windows.Forms.Button();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.txtEnlem = new System.Windows.Forms.TextBox();
            this.txtBoylam = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label12 = new System.Windows.Forms.Label();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.button6 = new System.Windows.Forms.Button();
            this.button5 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.button12 = new System.Windows.Forms.Button();
            this.button7 = new System.Windows.Forms.Button();
            this.button8 = new System.Windows.Forms.Button();
            this.button14 = new System.Windows.Forms.Button();
            this.button10 = new System.Windows.Forms.Button();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.button11 = new System.Windows.Forms.Button();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.backgroundWorker1 = new System.ComponentModel.BackgroundWorker();
            this.panel7 = new System.Windows.Forms.Panel();
            this.panel8 = new System.Windows.Forms.Panel();
            this.panel10 = new System.Windows.Forms.Panel();
            this.panel9 = new System.Windows.Forms.Panel();
            this.panel12 = new System.Windows.Forms.Panel();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.label27 = new System.Windows.Forms.Label();
            this.label26 = new System.Windows.Forms.Label();
            this.label25 = new System.Windows.Forms.Label();
            this.label23 = new System.Windows.Forms.Label();
            this.label24 = new System.Windows.Forms.Label();
            this.label22 = new System.Windows.Forms.Label();
            this.label21 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.timer2 = new System.Windows.Forms.Timer(this.components);
            this.dateTimePicker1 = new System.Windows.Forms.DateTimePicker();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.txtHedefBoylam = new System.Windows.Forms.TextBox();
            this.txtHedefEnlem = new System.Windows.Forms.TextBox();
            this.gMapControl1 = new GMap.NET.WindowsForms.GMapControl();
            this.DroneBataryaGostergesi = new ihaarayuz.BataryaGostergesi();
            this.IhaBataryaGostergesi = new ihaarayuz.BataryaGostergesi();
            this.ufukCizgisi1 = new ihaarayuz.UfukCizgisi();
            this.analogGostergeSicaklik2 = new ihaarayuz.AnalogGostergeSicaklik();
            this.analogGostergeIrtifa2 = new ihaarayuz.AnalogGostergeIrtifa();
            this.analogGostergeBasinc2 = new ihaarayuz.AnalogGostergeBasinc();
            this.analogGosterge1 = new ihaarayuz.AnalogGosterge();
            this.analogGostergeSicaklik1 = new ihaarayuz.AnalogGostergeSicaklik();
            this.analogGostergeIrtifa1 = new ihaarayuz.AnalogGostergeIrtifa();
            this.analogGostergeBasinc1 = new ihaarayuz.AnalogGostergeBasinc();
            this.analogGosterge2 = new ihaarayuz.AnalogGosterge();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.ihakamera)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox7.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.SuspendLayout();
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(418, 61);
            this.textBox1.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(160, 21);
            this.textBox1.TabIndex = 0;
            this.textBox1.Text = "#917756";
            this.textBox1.TextChanged += new System.EventHandler(this.textBox1_TextChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Arial Rounded MT Bold", 10.2F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
            this.label1.Location = new System.Drawing.Point(289, 62);
            this.label1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(121, 16);
            this.label1.TabIndex = 1;
            this.label1.Text = "Takım Numarası:";
            // 
            // pictureBox2
            // 
        //    this.pictureBox2.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("pictureBox2.BackgroundImage")));
            this.pictureBox2.Location = new System.Drawing.Point(54, 27);
            this.pictureBox2.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(148, 99);
            this.pictureBox2.TabIndex = 2;
            this.pictureBox2.TabStop = false;
            this.pictureBox2.Click += new System.EventHandler(this.pictureBox2_Click);
            // 
            // basla
            // 
            this.basla.Location = new System.Drawing.Point(1521, 61);
            this.basla.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.basla.Name = "basla";
            this.basla.Size = new System.Drawing.Size(153, 39);
            this.basla.TabIndex = 3;
            this.basla.Text = "Başla";
            this.basla.UseVisualStyleBackColor = true;
            this.basla.Click += new System.EventHandler(this.button1_Click);
            // 
            // ihakamera
            // 
            this.ihakamera.BackColor = System.Drawing.Color.LightSlateGray;
            this.ihakamera.Location = new System.Drawing.Point(639, 135);
            this.ihakamera.Name = "ihakamera";
            this.ihakamera.Size = new System.Drawing.Size(720, 463);
            this.ihakamera.TabIndex = 4;
            this.ihakamera.TabStop = false;
            // 
            // durdurbtn
            // 
            this.durdurbtn.Location = new System.Drawing.Point(1697, 61);
            this.durdurbtn.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.durdurbtn.Name = "durdurbtn";
            this.durdurbtn.Size = new System.Drawing.Size(144, 37);
            this.durdurbtn.TabIndex = 3;
            this.durdurbtn.Text = "Durdur";
            this.durdurbtn.UseVisualStyleBackColor = true;
            this.durdurbtn.Click += new System.EventHandler(this.durdurbtn_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Arial Rounded MT Bold", 10.2F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
            this.label6.Location = new System.Drawing.Point(1135, 61);
            this.label6.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(104, 16);
            this.label6.TabIndex = 1;
            this.label6.Text = "Drone Batarya";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Arial Rounded MT Bold", 10.2F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
            this.label7.Location = new System.Drawing.Point(905, 61);
            this.label7.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(88, 16);
            this.label7.TabIndex = 1;
            this.label7.Text = "İHA Batarya";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("Arial", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label9.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
            this.label9.Location = new System.Drawing.Point(291, 552);
            this.label9.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(59, 18);
            this.label9.TabIndex = 1;
            this.label9.Text = "Boylam";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Font = new System.Drawing.Font("Arial", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label10.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
            this.label10.Location = new System.Drawing.Point(-745, 1064);
            this.label10.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(84, 15);
            this.label10.TabIndex = 1;
            this.label10.Text = "İHA Kamerası";
            // 
            // txtEnlem
            // 
            this.txtEnlem.Location = new System.Drawing.Point(111, 550);
            this.txtEnlem.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.txtEnlem.Name = "txtEnlem";
            this.txtEnlem.Size = new System.Drawing.Size(139, 21);
            this.txtEnlem.TabIndex = 0;
            this.txtEnlem.TextChanged += new System.EventHandler(this.textBox1_TextChanged);
            // 
            // txtBoylam
            // 
            this.txtBoylam.Location = new System.Drawing.Point(352, 549);
            this.txtBoylam.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.txtBoylam.Name = "txtBoylam";
            this.txtBoylam.Size = new System.Drawing.Size(139, 21);
            this.txtBoylam.TabIndex = 0;
            this.txtBoylam.TextChanged += new System.EventHandler(this.textBox1_TextChanged);
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Arial", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label8.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
            this.label8.Location = new System.Drawing.Point(51, 553);
            this.label8.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(52, 18);
            this.label8.TabIndex = 1;
            this.label8.Text = "Enlem";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.analogGostergeSicaklik1);
            this.groupBox1.Controls.Add(this.analogGostergeIrtifa1);
            this.groupBox1.Controls.Add(this.analogGostergeBasinc1);
            this.groupBox1.Controls.Add(this.analogGosterge2);
            this.groupBox1.Font = new System.Drawing.Font("Arial", 12F, System.Drawing.FontStyle.Bold);
            this.groupBox1.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox1.Location = new System.Drawing.Point(40, 615);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(467, 352);
            this.groupBox1.TabIndex = 9;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "İHA Verileri";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label12.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label12.Location = new System.Drawing.Point(6, 98);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(94, 18);
            this.label12.TabIndex = 0;
            this.label12.Text = "Yeni Konum";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.button6);
            this.groupBox3.Controls.Add(this.button5);
            this.groupBox3.Controls.Add(this.button3);
            this.groupBox3.Font = new System.Drawing.Font("Arial", 12F, System.Drawing.FontStyle.Bold);
            this.groupBox3.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox3.Location = new System.Drawing.Point(1026, 629);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(222, 150);
            this.groupBox3.TabIndex = 10;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "İHA Manual Komutları";
            // 
            // button6
            // 
            this.button6.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button6.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button6.Location = new System.Drawing.Point(73, 84);
            this.button6.Name = "button6";
            this.button6.Size = new System.Drawing.Size(83, 53);
            this.button6.TabIndex = 0;
            this.button6.Text = "Yere İn";
            this.button6.UseVisualStyleBackColor = true;
            // 
            // button5
            // 
            this.button5.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button5.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button5.Location = new System.Drawing.Point(24, 23);
            this.button5.Name = "button5";
            this.button5.Size = new System.Drawing.Size(81, 53);
            this.button5.TabIndex = 0;
            this.button5.Text = "Otonom Mod";
            this.button5.UseVisualStyleBackColor = true;
            // 
            // button3
            // 
            this.button3.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button3.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button3.Location = new System.Drawing.Point(130, 25);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(83, 53);
            this.button3.TabIndex = 0;
            this.button3.Text = "Manuel Mod";
            this.button3.UseVisualStyleBackColor = true;
            // 
            // groupBox4
            // 
            this.groupBox4.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.groupBox4.Controls.Add(this.button12);
            this.groupBox4.Controls.Add(this.button7);
            this.groupBox4.Controls.Add(this.button8);
            this.groupBox4.Font = new System.Drawing.Font("Arial", 12F, System.Drawing.FontStyle.Bold);
            this.groupBox4.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox4.Location = new System.Drawing.Point(1634, 629);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(224, 150);
            this.groupBox4.TabIndex = 10;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Drone Manuel Komutları";
            this.groupBox4.Enter += new System.EventHandler(this.groupBox4_Enter);
            // 
            // button12
            // 
            this.button12.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button12.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button12.Location = new System.Drawing.Point(116, 38);
            this.button12.Name = "button12";
            this.button12.Size = new System.Drawing.Size(81, 48);
            this.button12.TabIndex = 0;
            this.button12.Text = "Görevi Bitir";
            this.button12.UseVisualStyleBackColor = true;
            // 
            // button7
            // 
            this.button7.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button7.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button7.Location = new System.Drawing.Point(71, 92);
            this.button7.Name = "button7";
            this.button7.Size = new System.Drawing.Size(81, 45);
            this.button7.TabIndex = 0;
            this.button7.Text = "Yere İn";
            this.button7.UseVisualStyleBackColor = true;
            // 
            // button8
            // 
            this.button8.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button8.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button8.Location = new System.Drawing.Point(16, 38);
            this.button8.Name = "button8";
            this.button8.Size = new System.Drawing.Size(80, 48);
            this.button8.TabIndex = 0;
            this.button8.Text = "Ayrıl";
            this.button8.UseVisualStyleBackColor = true;
            // 
            // button14
            // 
            this.button14.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button14.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button14.Location = new System.Drawing.Point(18, 30);
            this.button14.Name = "button14";
            this.button14.Size = new System.Drawing.Size(94, 56);
            this.button14.TabIndex = 1;
            this.button14.Text = "Havada Hazır Bekle";
            this.button14.UseVisualStyleBackColor = true;
            // 
            // button10
            // 
            this.button10.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button10.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button10.Location = new System.Drawing.Point(132, 30);
            this.button10.Name = "button10";
            this.button10.Size = new System.Drawing.Size(101, 56);
            this.button10.TabIndex = 0;
            this.button10.Text = "Görevi Duraklat";
            this.button10.UseVisualStyleBackColor = true;
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.button14);
            this.groupBox5.Controls.Add(this.button11);
            this.groupBox5.Controls.Add(this.button10);
            this.groupBox5.Font = new System.Drawing.Font("Arial", 12F, System.Drawing.FontStyle.Bold);
            this.groupBox5.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox5.Location = new System.Drawing.Point(1329, 629);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(249, 150);
            this.groupBox5.TabIndex = 10;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "Manuel Operasyon Komutları";
            // 
            // button11
            // 
            this.button11.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.button11.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.button11.Location = new System.Drawing.Point(68, 92);
            this.button11.Name = "button11";
            this.button11.Size = new System.Drawing.Size(101, 45);
            this.button11.TabIndex = 0;
            this.button11.Text = "Yardımı Durdur";
            this.button11.UseVisualStyleBackColor = true;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.analogGostergeSicaklik2);
            this.groupBox2.Controls.Add(this.analogGostergeIrtifa2);
            this.groupBox2.Controls.Add(this.analogGostergeBasinc2);
            this.groupBox2.Controls.Add(this.analogGosterge1);
            this.groupBox2.Font = new System.Drawing.Font("Arial", 12F, System.Drawing.FontStyle.Bold);
            this.groupBox2.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox2.Location = new System.Drawing.Point(526, 620);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(467, 352);
            this.groupBox2.TabIndex = 9;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Drone Verileri";
            // 
            // panel7
            // 
            this.panel7.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.panel7.Location = new System.Drawing.Point(132, 28);
            this.panel7.Name = "panel7";
            this.panel7.Size = new System.Drawing.Size(55, 49);
            this.panel7.TabIndex = 14;
            // 
            // panel8
            // 
            this.panel8.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.panel8.Location = new System.Drawing.Point(356, 28);
            this.panel8.Name = "panel8";
            this.panel8.Size = new System.Drawing.Size(55, 49);
            this.panel8.TabIndex = 14;
            // 
            // panel10
            // 
            this.panel10.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.panel10.Location = new System.Drawing.Point(30, 30);
            this.panel10.Name = "panel10";
            this.panel10.Size = new System.Drawing.Size(55, 49);
            this.panel10.TabIndex = 14;
            // 
            // panel9
            // 
            this.panel9.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.panel9.Location = new System.Drawing.Point(462, 29);
            this.panel9.Name = "panel9";
            this.panel9.Size = new System.Drawing.Size(55, 49);
            this.panel9.TabIndex = 14;
            // 
            // panel12
            // 
            this.panel12.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.panel12.Location = new System.Drawing.Point(238, 29);
            this.panel12.Name = "panel12";
            this.panel12.Size = new System.Drawing.Size(55, 50);
            this.panel12.TabIndex = 14;
            // 
            // groupBox7
            // 
            this.groupBox7.Controls.Add(this.panel12);
            this.groupBox7.Controls.Add(this.panel9);
            this.groupBox7.Controls.Add(this.panel10);
            this.groupBox7.Controls.Add(this.panel8);
            this.groupBox7.Controls.Add(this.panel7);
            this.groupBox7.Controls.Add(this.label27);
            this.groupBox7.Controls.Add(this.label26);
            this.groupBox7.Controls.Add(this.label25);
            this.groupBox7.Controls.Add(this.label23);
            this.groupBox7.Controls.Add(this.label24);
            this.groupBox7.Controls.Add(this.label22);
            this.groupBox7.Controls.Add(this.label21);
            this.groupBox7.Controls.Add(this.label19);
            this.groupBox7.Controls.Add(this.label12);
            this.groupBox7.Font = new System.Drawing.Font("Arial", 12F, System.Drawing.FontStyle.Bold);
            this.groupBox7.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox7.Location = new System.Drawing.Point(1026, 807);
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.Size = new System.Drawing.Size(552, 165);
            this.groupBox7.TabIndex = 15;
            this.groupBox7.TabStop = false;
            this.groupBox7.Text = "Alarm Sistemi";
            // 
            // label27
            // 
            this.label27.AutoSize = true;
            this.label27.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label27.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label27.Location = new System.Drawing.Point(459, 117);
            this.label27.Name = "label27";
            this.label27.Size = new System.Drawing.Size(66, 18);
            this.label27.TabIndex = 0;
            this.label27.Text = "Batarya";
            // 
            // label26
            // 
            this.label26.AutoSize = true;
            this.label26.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label26.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label26.Location = new System.Drawing.Point(450, 98);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(90, 18);
            this.label26.TabIndex = 0;
            this.label26.Text = "Drone Zayıf";
            // 
            // label25
            // 
            this.label25.AutoSize = true;
            this.label25.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label25.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label25.Location = new System.Drawing.Point(340, 110);
            this.label25.Name = "label25";
            this.label25.Size = new System.Drawing.Size(91, 18);
            this.label25.TabIndex = 0;
            this.label25.Text = "Zayıf Sinyal";
            // 
            // label23
            // 
            this.label23.AutoSize = true;
            this.label23.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label23.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label23.Location = new System.Drawing.Point(244, 98);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(45, 18);
            this.label23.TabIndex = 0;
            this.label23.Text = "Uçuş";
            // 
            // label24
            // 
            this.label24.AutoSize = true;
            this.label24.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label24.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label24.Location = new System.Drawing.Point(222, 120);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(94, 18);
            this.label24.TabIndex = 0;
            this.label24.Text = "Dengesizliği";
            // 
            // label22
            // 
            this.label22.AutoSize = true;
            this.label22.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label22.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label22.Location = new System.Drawing.Point(129, 120);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(66, 18);
            this.label22.TabIndex = 0;
            this.label22.Text = "Batarya";
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label21.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label21.Location = new System.Drawing.Point(127, 98);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(78, 18);
            this.label21.TabIndex = 0;
            this.label21.Text = "İHA Zayıf ";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Font = new System.Drawing.Font("Arial Black", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label19.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.label19.Location = new System.Drawing.Point(27, 116);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(56, 18);
            this.label19.TabIndex = 0;
            this.label19.Text = "Tespiti";
            // 
            // textBox2
            // 
            this.textBox2.Location = new System.Drawing.Point(729, 61);
            this.textBox2.Name = "textBox2";
            this.textBox2.Size = new System.Drawing.Size(100, 21);
            this.textBox2.TabIndex = 18;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Arial Rounded MT Bold", 10.2F);
            this.label2.ForeColor = System.Drawing.SystemColors.ButtonHighlight;
            this.label2.Location = new System.Drawing.Point(636, 64);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(77, 16);
            this.label2.TabIndex = 19;
            this.label2.Text = "Yük Sayısı";
            // 
            // timer2
            // 
            this.timer2.Enabled = true;
            this.timer2.Tick += new System.EventHandler(this.timer2_Tick);
            // 
            // dateTimePicker1
            // 
            this.dateTimePicker1.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.dateTimePicker1.Location = new System.Drawing.Point(1607, 27);
            this.dateTimePicker1.Name = "dateTimePicker1";
            this.dateTimePicker1.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.dateTimePicker1.Size = new System.Drawing.Size(222, 21);
            this.dateTimePicker1.TabIndex = 8;
            this.dateTimePicker1.Value = new System.DateTime(2026, 3, 15, 0, 0, 0, 0);
            this.dateTimePicker1.ValueChanged += new System.EventHandler(this.dateTimePicker1_ValueChanged);
            // 
            // groupBox6
            // 
            this.groupBox6.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.groupBox6.Controls.Add(this.label4);
            this.groupBox6.Controls.Add(this.label3);
            this.groupBox6.Controls.Add(this.txtHedefBoylam);
            this.groupBox6.Controls.Add(this.txtHedefEnlem);
            this.groupBox6.Font = new System.Drawing.Font("Arial", 12F, System.Drawing.FontStyle.Bold);
            this.groupBox6.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox6.Location = new System.Drawing.Point(1634, 809);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(224, 163);
            this.groupBox6.TabIndex = 10;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "Hedef Verileri";
            this.groupBox6.Enter += new System.EventHandler(this.groupBox4_Enter);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(12, 107);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(67, 19);
            this.label4.TabIndex = 2;
            this.label4.Text = "Boylam";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(12, 48);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(57, 19);
            this.label3.TabIndex = 2;
            this.label3.Text = "Enlem";
            // 
            // txtHedefBoylam
            // 
            this.txtHedefBoylam.Location = new System.Drawing.Point(94, 104);
            this.txtHedefBoylam.Name = "txtHedefBoylam";
            this.txtHedefBoylam.Size = new System.Drawing.Size(100, 26);
            this.txtHedefBoylam.TabIndex = 1;
            // 
            // txtHedefEnlem
            // 
            this.txtHedefEnlem.Location = new System.Drawing.Point(94, 48);
            this.txtHedefEnlem.Name = "txtHedefEnlem";
            this.txtHedefEnlem.Size = new System.Drawing.Size(100, 26);
            this.txtHedefEnlem.TabIndex = 0;
            // 
            // gMapControl1
            // 
            this.gMapControl1.Bearing = 0F;
            this.gMapControl1.CanDragMap = true;
            this.gMapControl1.EmptyTileColor = System.Drawing.Color.Navy;
            this.gMapControl1.GrayScaleMode = false;
            this.gMapControl1.HelperLineOption = GMap.NET.WindowsForms.HelperLineOptions.DontShow;
            this.gMapControl1.LevelsKeepInMemory = 5;
            this.gMapControl1.Location = new System.Drawing.Point(54, 135);
            this.gMapControl1.MarkersEnabled = true;
            this.gMapControl1.MaxZoom = 2;
            this.gMapControl1.MinZoom = 2;
            this.gMapControl1.MouseWheelZoomEnabled = true;
            this.gMapControl1.MouseWheelZoomType = GMap.NET.MouseWheelZoomType.MousePositionAndCenter;
            this.gMapControl1.Name = "gMapControl1";
            this.gMapControl1.NegativeMode = false;
            this.gMapControl1.PolygonsEnabled = true;
            this.gMapControl1.RetryLoadTile = 0;
            this.gMapControl1.RoutesEnabled = true;
            this.gMapControl1.ScaleMode = GMap.NET.WindowsForms.ScaleModes.Integer;
            this.gMapControl1.SelectedAreaFillColor = System.Drawing.Color.FromArgb(((int)(((byte)(33)))), ((int)(((byte)(65)))), ((int)(((byte)(105)))), ((int)(((byte)(225)))));
            this.gMapControl1.ShowTileGridLines = false;
            this.gMapControl1.Size = new System.Drawing.Size(546, 379);
            this.gMapControl1.TabIndex = 20;
            this.gMapControl1.Zoom = 0D;
            // 
            // DroneBataryaGostergesi
            // 
            this.DroneBataryaGostergesi.Location = new System.Drawing.Point(1246, 51);
            this.DroneBataryaGostergesi.Name = "DroneBataryaGostergesi";
            this.DroneBataryaGostergesi.Size = new System.Drawing.Size(81, 31);
            this.DroneBataryaGostergesi.TabIndex = 17;
            this.DroneBataryaGostergesi.Text = "bataryaGostergesi2";
            this.DroneBataryaGostergesi.Yuzde = 100;
            // 
            // IhaBataryaGostergesi
            // 
            this.IhaBataryaGostergesi.Location = new System.Drawing.Point(1000, 51);
            this.IhaBataryaGostergesi.Name = "IhaBataryaGostergesi";
            this.IhaBataryaGostergesi.Size = new System.Drawing.Size(81, 31);
            this.IhaBataryaGostergesi.TabIndex = 16;
            this.IhaBataryaGostergesi.Text = "bataryaGostergesi1";
            this.IhaBataryaGostergesi.Yuzde = 100;
            // 
            // ufukCizgisi1
            // 
            this.ufukCizgisi1.Heading = 0F;
            this.ufukCizgisi1.Hiz = 0F;
            this.ufukCizgisi1.Irtifa = 0F;
            this.ufukCizgisi1.Location = new System.Drawing.Point(1405, 135);
            this.ufukCizgisi1.Name = "ufukCizgisi1";
            this.ufukCizgisi1.Pitch = 0F;
            this.ufukCizgisi1.Roll = 0F;
            this.ufukCizgisi1.Size = new System.Drawing.Size(470, 463);
            this.ufukCizgisi1.TabIndex = 6;
            this.ufukCizgisi1.Text = "ufukCizgisi1";
            // 
            // analogGostergeSicaklik2
            // 
            this.analogGostergeSicaklik2.Baslik = "SICAKLIK";
            this.analogGostergeSicaklik2.Birim = "°C";
            this.analogGostergeSicaklik2.Deger = 0F;
            this.analogGostergeSicaklik2.Location = new System.Drawing.Point(266, 189);
            this.analogGostergeSicaklik2.Maksimum = 80F;
            this.analogGostergeSicaklik2.Minimum = -20F;
            this.analogGostergeSicaklik2.Name = "analogGostergeSicaklik2";
            this.analogGostergeSicaklik2.Size = new System.Drawing.Size(148, 148);
            this.analogGostergeSicaklik2.TabIndex = 5;
            this.analogGostergeSicaklik2.Text = "analogGostergeSicaklik2";
            // 
            // analogGostergeIrtifa2
            // 
            this.analogGostergeIrtifa2.Baslik = "İRTİFA";
            this.analogGostergeIrtifa2.Birim = "m";
            this.analogGostergeIrtifa2.Deger = 0F;
            this.analogGostergeIrtifa2.Location = new System.Drawing.Point(266, 35);
            this.analogGostergeIrtifa2.Maksimum = 150F;
            this.analogGostergeIrtifa2.Minimum = 0F;
            this.analogGostergeIrtifa2.Name = "analogGostergeIrtifa2";
            this.analogGostergeIrtifa2.Size = new System.Drawing.Size(148, 148);
            this.analogGostergeIrtifa2.TabIndex = 4;
            this.analogGostergeIrtifa2.Text = "analogGostergeIrtifa2";
            // 
            // analogGostergeBasinc2
            // 
            this.analogGostergeBasinc2.Baslik = "BASINÇ";
            this.analogGostergeBasinc2.Birim = "hPa";
            this.analogGostergeBasinc2.Deger = 0F;
            this.analogGostergeBasinc2.Location = new System.Drawing.Point(39, 193);
            this.analogGostergeBasinc2.Maksimum = 1100F;
            this.analogGostergeBasinc2.Minimum = 900F;
            this.analogGostergeBasinc2.Name = "analogGostergeBasinc2";
            this.analogGostergeBasinc2.Size = new System.Drawing.Size(148, 148);
            this.analogGostergeBasinc2.TabIndex = 3;
            this.analogGostergeBasinc2.Text = "analogGostergeBasinc2";
            // 
            // analogGosterge1
            // 
            this.analogGosterge1.Baslik = "HIZ";
            this.analogGosterge1.Birim = "m/s";
            this.analogGosterge1.Deger = 0F;
            this.analogGosterge1.Location = new System.Drawing.Point(39, 35);
            this.analogGosterge1.Maksimum = 50F;
            this.analogGosterge1.Minimum = 0F;
            this.analogGosterge1.Name = "analogGosterge1";
            this.analogGosterge1.Size = new System.Drawing.Size(148, 148);
            this.analogGosterge1.TabIndex = 2;
            this.analogGosterge1.Text = "analogGosterge1";
            // 
            // analogGostergeSicaklik1
            // 
            this.analogGostergeSicaklik1.Baslik = "SICAKLIK";
            this.analogGostergeSicaklik1.Birim = "°C";
            this.analogGostergeSicaklik1.Deger = 0F;
            this.analogGostergeSicaklik1.Location = new System.Drawing.Point(265, 189);
            this.analogGostergeSicaklik1.Maksimum = 80F;
            this.analogGostergeSicaklik1.Minimum = -20F;
            this.analogGostergeSicaklik1.Name = "analogGostergeSicaklik1";
            this.analogGostergeSicaklik1.Size = new System.Drawing.Size(148, 148);
            this.analogGostergeSicaklik1.TabIndex = 22;
            this.analogGostergeSicaklik1.Text = "analogGostergeSicaklik1";
            // 
            // analogGostergeIrtifa1
            // 
            this.analogGostergeIrtifa1.Baslik = "İRTİFA";
            this.analogGostergeIrtifa1.Birim = "m";
            this.analogGostergeIrtifa1.Deger = 0F;
            this.analogGostergeIrtifa1.Location = new System.Drawing.Point(265, 35);
            this.analogGostergeIrtifa1.Maksimum = 150F;
            this.analogGostergeIrtifa1.Minimum = 0F;
            this.analogGostergeIrtifa1.Name = "analogGostergeIrtifa1";
            this.analogGostergeIrtifa1.Size = new System.Drawing.Size(148, 148);
            this.analogGostergeIrtifa1.TabIndex = 21;
            this.analogGostergeIrtifa1.Text = "analogGostergeIrtifa1";
            // 
            // analogGostergeBasinc1
            // 
            this.analogGostergeBasinc1.Baslik = "BASINÇ";
            this.analogGostergeBasinc1.Birim = "hPa";
            this.analogGostergeBasinc1.Deger = 0F;
            this.analogGostergeBasinc1.Location = new System.Drawing.Point(53, 193);
            this.analogGostergeBasinc1.Maksimum = 1100F;
            this.analogGostergeBasinc1.Minimum = 900F;
            this.analogGostergeBasinc1.Name = "analogGostergeBasinc1";
            this.analogGostergeBasinc1.Size = new System.Drawing.Size(148, 148);
            this.analogGostergeBasinc1.TabIndex = 20;
            this.analogGostergeBasinc1.Text = "analogGostergeBasinc1";
            // 
            // analogGosterge2
            // 
            this.analogGosterge2.Baslik = "HIZ";
            this.analogGosterge2.Birim = "m/s";
            this.analogGosterge2.Deger = 0F;
            this.analogGosterge2.Location = new System.Drawing.Point(53, 35);
            this.analogGosterge2.Maksimum = 100F;
            this.analogGosterge2.Minimum = 0F;
            this.analogGosterge2.Name = "analogGosterge2";
            this.analogGosterge2.Size = new System.Drawing.Size(148, 148);
            this.analogGosterge2.TabIndex = 16;
            this.analogGosterge2.Text = "analogGosterge2";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 15F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.CornflowerBlue;
            this.ClientSize = new System.Drawing.Size(1924, 1031);
            this.Controls.Add(this.gMapControl1);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.DroneBataryaGostergesi);
            this.Controls.Add(this.IhaBataryaGostergesi);
            this.Controls.Add(this.ufukCizgisi1);
            this.Controls.Add(this.groupBox7);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.dateTimePicker1);
            this.Controls.Add(this.ihakamera);
            this.Controls.Add(this.durdurbtn);
            this.Controls.Add(this.basla);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.txtBoylam);
            this.Controls.Add(this.txtEnlem);
            this.Controls.Add(this.textBox1);
            this.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.Margin = new System.Windows.Forms.Padding(4, 3, 4, 3);
            this.Name = "Form1";
            this.RightToLeftLayout = true;
            this.Text = "Misya Yer İstasyon Arayüzü";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.Load += new System.EventHandler(this.Form1_Load);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.ihakamera)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.groupBox4.ResumeLayout(false);
            this.groupBox5.ResumeLayout(false);
            this.groupBox2.ResumeLayout(false);
            this.groupBox7.ResumeLayout(false);
            this.groupBox7.PerformLayout();
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.Button basla;
        private System.Windows.Forms.PictureBox ihakamera;
        private System.Windows.Forms.Button durdurbtn;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox txtEnlem;
        private System.Windows.Forms.TextBox txtBoylam;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Button button6;
        private System.Windows.Forms.Button button5;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Button button7;
        private System.Windows.Forms.Button button8;
        private System.Windows.Forms.Button button10;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.Button button11;
        private System.Windows.Forms.Button button12;
        private System.Windows.Forms.Button button14;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.ComponentModel.BackgroundWorker backgroundWorker1;
        private System.Windows.Forms.Panel panel7;
        private System.Windows.Forms.Panel panel8;
        private System.Windows.Forms.Panel panel10;
        private System.Windows.Forms.Panel panel9;
        private System.Windows.Forms.Panel panel12;
        private System.Windows.Forms.GroupBox groupBox7;
        private System.Windows.Forms.Label label24;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.Label label25;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.Label label27;
        private System.Windows.Forms.Label label26;
        private AnalogGosterge analogGosterge2;
        private AnalogGostergeBasinc analogGostergeBasinc1;
        private AnalogGostergeIrtifa analogGostergeIrtifa1;
        private AnalogGostergeSicaklik analogGostergeSicaklik1;
        private AnalogGostergeSicaklik analogGostergeSicaklik2;
        private AnalogGostergeIrtifa analogGostergeIrtifa2;
        private AnalogGostergeBasinc analogGostergeBasinc2;
        private AnalogGosterge analogGosterge1;
        private UfukCizgisi ufukCizgisi1;
        private BataryaGostergesi IhaBataryaGostergesi;
        private BataryaGostergesi DroneBataryaGostergesi;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Timer timer2;
        private System.Windows.Forms.DateTimePicker dateTimePicker1;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox txtHedefBoylam;
        private System.Windows.Forms.TextBox txtHedefEnlem;
        private GMap.NET.WindowsForms.GMapControl gMapControl1;
    }
}

