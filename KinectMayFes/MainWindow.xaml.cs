using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using MathNet.Numerics.LinearAlgebra.Double;

namespace KinectMayFes
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    /// 
    public partial class MainWindow : Window
    {
        private const ColorImageFormat rgbFormat
            = ColorImageFormat.RgbResolution640x480Fps30;
        private const DepthImageFormat depthFormat
            = DepthImageFormat.Resolution320x240Fps30;

        private byte[] pixelBuffer = null;

        private DepthImagePixel[] depthBuffer = null;

        private ColorImagePoint[] clrPntBuffer = null;

        private byte[] depthMaskBuffer = null;

        private RenderTargetBitmap bmpBuffer = null;

        private DrawingVisual drawVisual = new DrawingVisual();

        private const int z0 = 765, w = 1200, h = 900, span = 2;    //z0を変更する

        private const double fall_height = 0;

        private double time = 0;

        private int faze_count = 0;

        private int f_count = 0;

        private int[,] target_track = new int[5,3] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

        public MainWindow()
        {
            InitializeComponent();
        }

        private int[] calculate_locus()
        {
            int[] fall_point = { 0, 0 };
            int[] error = { 0, 0, 0};
            double[] velocity = { 0,0};
            double[] v_x = new double[4];
            double[] v_y = new double[4];
            for(int j = 0; j < 4; j++)
            {
                v_x[j] = target_track[j+1, 0] - target_track[j, 0];
                v_y[j] = target_track[j+1, 1] - target_track[j, 1];
            }

            //if (Math.Abs(v_x1 - v_x2) >= 150 || Math.Abs(v_y1 - v_y2) >= 150)
            //   return error;
            for (int j = 0; j < 4; j++)
            {
                velocity[0] += v_x[j];
                velocity[1] += v_y[j];
            }

            velocity[0] /= 4.0;
            velocity[1] /= 4.0;

            int sigma_t1 = 0, sigma_t2 = 0,sigma_t3 = 0,sigma_t4 = 0;
            int sigma_t2_z = 0, sigma_t1_z = 0, sigma_z = 0;
            int i = 0;
            for (i = 0; i < 5; i++)
            {
                sigma_t1 += 1 + span * i;
                sigma_t2 += (int)Math.Pow(1 + span * i, 2);
                sigma_t3 += (int)Math.Pow(1 + span * i, 3);
                sigma_t4 += (int)Math.Pow(1 + span * i, 4);
                sigma_t2_z += (int)Math.Pow(1 + span * i, 2) * target_track[i, 2];
                sigma_t1_z += (1 + span * i) * target_track[i, 2];
                sigma_z += target_track[i, 2];
            }
            var M1 = DenseMatrix.OfArray(new double[,]
                { { sigma_t4, sigma_t3, sigma_t2 },
                { sigma_t3, sigma_t2, sigma_t1 },
                { sigma_t2, sigma_t1, i } });
            var M2 = DenseMatrix.OfArray(new double[,]
                { { sigma_t2_z },
                { sigma_t1_z },
                { sigma_z } });
            var ansM = M1.Inverse() * M2;

            double a = ansM[0,0], b = ansM[1,0], c = ansM[2,0];
            double answer_t = (-b + Math.Sqrt(Math.Pow(b, 2.0) - 4.0 * a * (c - fall_height))) / (2.0 * a);

            //if (answer_t <= 1 + span * 2)
            //    return error;

            fall_point[0] = (int)(target_track[4, 0] + velocity[0] * (answer_t - 1 - span * 4+2));
            fall_point[1] = (int)(target_track[4, 1] + velocity[1] * (answer_t - 1 - span * 4+2));
            time = answer_t;

            return fall_point;
        }

        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            KinectSensor kinect = KinectSensor.KinectSensors[0];

            ColorImageStream clrStream = kinect.ColorStream;
            clrStream.Enable(rgbFormat);
            DepthImageStream depStream = kinect.DepthStream;
            depStream.Enable(depthFormat);

            kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

            pixelBuffer = new byte[kinect.ColorStream.FramePixelDataLength];
            depthBuffer = new DepthImagePixel[depStream.FramePixelDataLength];
            clrPntBuffer = new ColorImagePoint[depStream.FramePixelDataLength];
            depthMaskBuffer = new byte[clrStream.FramePixelDataLength];

            bmpBuffer = new RenderTargetBitmap(clrStream.FrameWidth,clrStream.FrameHeight,
                96, 96, PixelFormats.Default);
            rgbImage.Source = bmpBuffer;

            kinect.AllFramesReady += AllFramesReady;

            textBox1.Clear();

            kinect.Start();

            kinect.ElevationAngle = 0;
        }

        private void AllFramesReady(object sender,AllFramesReadyEventArgs e)
        {

            KinectSensor kinect = sender as KinectSensor;

            using (ColorImageFrame imageFrame = e.OpenColorImageFrame())
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (imageFrame != null)
                {
                    fillBitmap(kinect, imageFrame, depthFrame);
                }
            }
        }

        private void fillBitmap(KinectSensor kinect, ColorImageFrame imgFrame, DepthImageFrame depthFrame)
        {
            var drawContext = drawVisual.RenderOpen();
            int frmWidth = imgFrame.Width;
            int frmHeight = imgFrame.Height;
            int depWidth = depthFrame.Width;
            int depHeight = depthFrame.Height;

            imgFrame.CopyPixelDataTo(pixelBuffer);
            depthFrame.CopyDepthImagePixelDataTo(depthBuffer);

            kinect.CoordinateMapper.MapDepthFrameToColorFrame(depthFormat, depthBuffer, rgbFormat, clrPntBuffer);

            Array.Clear(depthMaskBuffer, 0, depthMaskBuffer.Length);

            int[] sum = { 0, 0 }, center = { 0, 0 };
            int point = 0;
            int target_depth = 0, target_depth_point = 0;

            for (int depIdx = 0; depIdx < depthBuffer.Length; ++depIdx)
            {
                if (depthBuffer[depIdx].Depth >= 600 && depthBuffer[depIdx].Depth <= 1300)
                {
                    ColorImagePoint clrPoint = clrPntBuffer[depIdx];
                    int clrIdx = clrPoint.Y * frmWidth + clrPoint.X;
                    depthMaskBuffer[clrIdx * 4] = 255;
                    depthMaskBuffer[clrIdx * 4 + 1] = 0;
                    depthMaskBuffer[clrIdx * 4 + 2] = 0;
                    depthMaskBuffer[clrIdx * 4 + 3] = 128;
                    sum[0] += clrPoint.X;
                    sum[1] += clrPoint.Y;
                    point++;
                    target_depth += depthBuffer[depIdx].Depth;
                    target_depth_point++;
                }
            }
            textBox.Clear();
            if (point != 0)
            {
                center[0] = sum[0] / point;
                center[1] = sum[1] / point;
                int centerIdx;
                int target_x, target_y, target_z;

                target_depth /= target_depth_point;

                for (int i = -4; i < 5; i++)
                {
                    for (int j = -4; j < 5; j++)
                    {
                        centerIdx = (center[1] + j) * frmWidth + center[0] + i;
                        depthMaskBuffer[centerIdx * 4] = 0;
                        depthMaskBuffer[centerIdx * 4 + 1] = 255;
                        depthMaskBuffer[centerIdx * 4 + 2] = 0;
                        depthMaskBuffer[centerIdx * 4 + 3] = 128;
                    }
                }

                //target_depth = depthBuffer[center[1] * depHeight * depWidth / frmHeight + center[0] * depWidth / frmWidth].Depth;
                target_x = (int)((double)w * (double)target_depth / 1000.0 / 2.0 * (2.0 * (double)center[0] / (double)frmWidth - 1.0));
                target_y = target_depth;
                target_z = (int)((double)h * (double)target_depth / 1000.0 / 2.0 * (1.0 - 2.0 * (double)center[1] / (double)frmHeight) + (double)z0);

                textBox.AppendText("(" + target_x + "," + target_y + "," + target_z + ")");
                //textBox.AppendText("(" + target_depth + ")");

                faze_count++;
                if (faze_count == 1 || faze_count == 1+span || faze_count == 1 + span*2 || faze_count == 1+span*3 ) {
                    target_track[f_count, 0] = target_x;
                    target_track[f_count, 1] = target_y;
                    target_track[f_count, 2] = target_z;
                    f_count++;
                }
                else if(faze_count == 1 + span*4)
                {
                    target_track[f_count, 0] = target_x;
                    target_track[f_count, 1] = target_y;
                    target_track[f_count, 2] = target_z;

                    int[] fall_point = calculate_locus();

                    textBox1.Clear();
                    textBox1.AppendText("(" + fall_point[0] + "," + fall_point[1] + ")" + time);
                    faze_count = 0;
                    f_count = 0;
                }
            }else
            {
                faze_count = 0;
                f_count = 0;
            }

            for (int i = -9; i < 10; i++)
            {
                int centerIdx = frmHeight / 2 * frmWidth + frmWidth / 2 + i;
                depthMaskBuffer[centerIdx * 4] = 0;
                depthMaskBuffer[centerIdx * 4 + 1] = 0;
                depthMaskBuffer[centerIdx * 4 + 2] = 255;
                depthMaskBuffer[centerIdx * 4 + 3] = 128;
            }
            for (int i = -9; i < 10; i++)
            {
                int centerIdx = (frmHeight / 2 + i) * frmWidth + frmWidth / 2;
                depthMaskBuffer[centerIdx * 4] = 0;
                depthMaskBuffer[centerIdx * 4 + 1] = 0;
                depthMaskBuffer[centerIdx * 4 + 2] = 255;
                depthMaskBuffer[centerIdx * 4 + 3] = 128;
            }

            var bgMask = new WriteableBitmap(frmWidth, frmHeight, 96, 96, PixelFormats.Bgra32, null);
            bgMask.WritePixels(new Int32Rect(0, 0, frmWidth, frmHeight), depthMaskBuffer, frmWidth * 4, 0);

            var bgImg = new WriteableBitmap(frmWidth, frmHeight, 96, 96, PixelFormats.Bgr32, null);
            bgImg.WritePixels(new Int32Rect(0, 0, frmWidth, frmHeight), pixelBuffer, frmWidth * 4, 0);

            Rect frmRect = new Rect(0, 0, frmWidth, frmHeight);
            drawContext.DrawImage(bgImg, frmRect);
            drawContext.DrawImage(bgMask, frmRect);

            drawContext.Close();
            bmpBuffer.Render(drawVisual);

            bgImg.Freeze();
            bgMask.Freeze();
        }

    }
}
