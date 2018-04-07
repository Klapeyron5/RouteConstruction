using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RouteConstruction
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            pictureBox1.SizeMode = PictureBoxSizeMode.StretchImage;
            pictureBox1.MouseUp += pictureBox1_MouseUp;
        }

        Bitmap bitmap;
        Bitmap clearMap;
        Bitmap preprocessedMap;

        ushort startX;
        ushort startY;
        ushort finishX;
        ushort finishY;

        bool BitmapLoaded = false;
        bool RouteConstructed = true;

        static Color startColor = Color.FromArgb(94, 255, 0);
        static Color routeColor = Color.FromArgb(255, 51, 0);
        static Color finishColor = Color.FromArgb(0, 222, 255);
        public static int robotRadius = 6;


        private void buttonLoad_Click(object sender, EventArgs e)
        {
            using (OpenFileDialog dlg = new OpenFileDialog())
            {
                dlg.Title = "Open Image";
                dlg.Filter = "bmp files (*.bmp)|*.bmp";

                if (dlg.ShowDialog() == DialogResult.OK)
                {
                    bitmap = new Bitmap(dlg.FileName);
                    clearMap = new Bitmap(bitmap);
                    preprocessedMap = MapPreprocessing.getProcessedBitmap(ref bitmap);
                    pictureBox1.Image = bitmap;
                    BitmapLoaded = true;
                }
            }
        }

        private void pictureBox1_MouseUp(object sender, MouseEventArgs e)
        {
            try
            {
                if ((BitmapLoaded) && (preprocessedMap.GetPixel(e.X * preprocessedMap.Width / pictureBox1.Size.Width,
                        e.Y * preprocessedMap.Height / pictureBox1.Size.Height) != MapPreprocessing.wallColor))
                {
                    if (RouteConstructed)
                    {
                        bitmap = new Bitmap(clearMap);
                        SolidBrush brush = new SolidBrush(startColor);
                        Graphics graphics = Graphics.FromImage(bitmap);
                        startX = (ushort)(e.X * bitmap.Width / pictureBox1.Size.Width);
                        startY = (ushort)(e.Y * bitmap.Height / pictureBox1.Size.Height);
						Console.WriteLine("Start "+startX+", "+startY);
                        graphics.FillEllipse(brush, startX - robotRadius, startY - robotRadius, 2 * robotRadius, 2 * robotRadius);
                        pictureBox1.Image = bitmap;
                        RouteConstructed = false;
                    }
                    else
                    {
                        finishX = (ushort)(e.X * bitmap.Width / pictureBox1.Size.Width);
                        finishY = (ushort)(e.Y * bitmap.Height / pictureBox1.Size.Height);
						Console.WriteLine("Finish " + finishX + ", " + finishY);
						Astar astar = new RouteConstruction.Astar(preprocessedMap, startX, startY, finishX, finishY, 2);
                        List<ushort[]> route = astar.getRoute();
                        for (int i = 0; i < route.Count; i++)
                            bitmap.SetPixel(route[i][0], route[i][1], routeColor);
                        SolidBrush brush = new SolidBrush(finishColor);
                        Graphics graphics = Graphics.FromImage(bitmap);
                        graphics.FillEllipse(brush, finishX - robotRadius, finishY - robotRadius, 2 * robotRadius, 2 * robotRadius);
                        pictureBox1.Image = bitmap;
                        RouteConstructed = true;
                    }
                }
            }
            catch (Exception ex) {}
            finally { }
        }
    }
}