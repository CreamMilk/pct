using System;
using System.Text;
using System.Runtime.InteropServices;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;

namespace Test
{

    class Program
    {
        [DllImport("ocr.dll")]
        public static extern int Loading(int i);

        [DllImport("ocr.dll")]
        public static extern int OCR(byte[] buff, int ImgBufLen, StringBuilder s);

        [DllImport("ocr.dll")]
        public static extern int OCR2(byte[] buff, StringBuilder s);

        private static object obj = new object();
        private static string GetKey(byte[] key)
        {
            lock (obj)
            {
                try
                {
                    StringBuilder sb = new StringBuilder(1024);
                    int i = OCR(key, key.Length, sb);
                    return sb.ToString();
                }
                catch { return ""; }

            }
        }
        static void Main(string[] args)
        {
            //             System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
            //             stopwatch.Start(); //  开始监视代码运行时间
            // 
            // 
            //             var files = Directory.GetFiles(args[0], "*.*", SearchOption.AllDirectories).Where(s => s.ToLower().EndsWith(".jpg")
            //                 || s.ToLower().EndsWith(".png") || s.ToLower().EndsWith(".bmp") || s.ToLower().EndsWith(".jpeg"));
            // 
            //              foreach (var file in files)
            //              {
            string file = args[0];
            int i = Loading(100);
            //Console.Out.WriteLine("分析照片：" + file);
            Image image = Image.FromFile(file);

            ImageFormat format = image.RawFormat;
            using (MemoryStream ms = new MemoryStream())
            {
                image.Save(ms, ImageFormat.Jpeg);
                StringBuilder sb = new StringBuilder(1024);
                int j = OCR(ms.ToArray(), ms.ToArray().Length, sb);
                //第一个是起始坐标位置，第二个是识别类别。
                Console.Out.WriteLine("识别结果：" + sb.ToString());
            }

            image.Dispose();

            //              }
            //              Console.Out.WriteLine("可见光图像分析完成。");
            // 
            //              stopwatch.Stop(); //  停止监视

            System.Diagnostics.Process.GetCurrentProcess().Kill();
        }
    }
}
