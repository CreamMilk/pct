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
           int i = Loading(100);
           Console.Out.WriteLine(i);
           Image image = Image.FromFile("F:/project/powerline/solution/pct/鸟巢程序/Test/bin/Debug/00284.JPG"/*args[0]*/);

            ImageFormat format = image.RawFormat;
            using (MemoryStream ms = new MemoryStream())
            {
                image.Save(ms, ImageFormat.Jpeg);
                StringBuilder sb = new StringBuilder(1024);
               int j = OCR(ms.ToArray(), ms.ToArray().Length, sb);
                //第一个是起始坐标位置，第二个是识别类别。
                Console.Out.WriteLine(sb.ToString());
            }
                Console.Write("Press any key to continue . . . ");
           Console.ReadKey(true);
        }
    }
}
