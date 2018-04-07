using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Drawing.Imaging;

namespace ChatClientWinForms
{
	/// <summary>
	/// A container of mutable pixel data.</summary>
	public class PixelMap
	{
		/// <summary>
		/// Creates a blank PixelMap of desired width and height.</summary>
		public PixelMap(int width,int height)
		{
			Width = width;
			Height = height;
			map = new Pixel[Width,Height];
			BPP = 4;
			format = PixelFormat.Format32bppArgb;
		}

		/// <summary>
		/// Creates a blank PixelMap of desired width and height with one color.</summary>
		public PixelMap(int width,int height,int R,int G,int B)
		{
			Width = width;
			Height = height;
			map = new Pixel[Width,Height];
			BPP = 4;
			format = PixelFormat.Format32bppArgb;

			for (int x = 0; x < Width; x++)
			{
				for (int y = 0; y < Height; y++)
				{
					map[x,y] = new Pixel(255,R,G,B);
				}
			}
		}

		/// <summary>
		/// Clones a PixelMap.</summary>
		public PixelMap(PixelMap original)
		{
			Width = original.Width;
			Height = original.Height;
			map = new Pixel[Width,Height];

			BPP = original.BPP;
			format = original.format;

			for (int x = 0; x < Width; x++)
			{
				for (int y = 0; y < Height; y++)
				{
					this[x,y] = original[x,y];
				}
			}
		}

		/// <summary>
		/// Quickly creates a PixelMap from a file path.</summary>
		public PixelMap(string s) : this(new Bitmap(s))
		{ }

		/// <summary>
		/// Quickly creates a PixelMap from a Bitmap.</summary>
		/// <seealso cref="PixelMap.SlowLoad">
		/// Copies a Bitmap through slow pixel-by-pixel reads. </seealso>
		public PixelMap(Bitmap b)
		{
			Width = b.Width;
			Height = b.Height;
			map = new Pixel[Width,Height];
			format = b.PixelFormat;


			var data = b.LockBits(new Rectangle(0,0,Width,Height),ImageLockMode.ReadOnly,format);

			switch (b.PixelFormat)
			{
				case PixelFormat.Format32bppArgb:
				case PixelFormat.Format32bppRgb:
					BPP = 4;
					break;
				case PixelFormat.Format24bppRgb:
					BPP = 3;
					break;
				default:
					throw new FormatException("PixelFormat cannot be loaded. Try PixelMap.SlowLoad instead.");
			}


			int bytes = Math.Abs(data.Stride) * b.Height;

			byte[] raw = new byte[bytes];
			System.Runtime.InteropServices.Marshal.Copy(data.Scan0,raw,0,bytes);

			for (int x = 0; x < Width; x++)
			{
				for (int y = 0; y < Height; y++)
				{
					int offset = (y * data.Width + x) * BPP;

					byte B = raw[offset];
					offset++;
					byte G = raw[offset];
					offset++;
					byte R = raw[offset];

					byte A = 0;
					if (BPP == 4)
					{
						offset++;
						A = raw[offset];
					}

				//	map[x,y] = new Pixel(A,R,G,B);
					map[x,y] = new Pixel(255,0,255,0);


				}
			}

			b.UnlockBits(data);
		}

		/// <summary>
		/// The width of the PixelMap in pixels.</summary>
		public readonly int Width;
		/// <summary>
		/// The height of the PixelMap in pixels.</summary>
		public readonly int Height;


		private readonly Pixel[,] map;

		private readonly int BPP;
		private readonly PixelFormat format;

		/// <summary>
		/// Access a Pixel of the PixelMap from its X and Y coordinates.</summary>
		public Pixel this[int x,int y]
		{
			get
			{
				if (Inside(new Point(x,y)))
					return map[x,y];
				return map[Math.Max(Math.Min(x,Width - 1),0),Math.Max(Math.Min(y,Height - 1),0)];
			}
			set
			{
				if (Inside(new Point(x,y)))
					map[x,y] = value;
			}
		}

		/// <summary>
		/// Access a Pixel of the PixelMap from its X and Y coordinates contained within a Point.</summary>
		public Pixel this[Point p]
		{
			get { return this[p.X,p.Y]; }
			set { this[p.X,p.Y] = value; }
		}

		/// <summary>
		/// Access a Pixel of the PixelMap from its flattened index.</summary>
		public Pixel this[int i]
		{
			get { return this[i / Height,i % Height]; }
			set { this[i / Height,i % Height] = value; }
		}

		/// <summary>
		/// Determine if a point is within this PixelMap.</summary>
		public bool Inside(Point p)
		{
			return (p.X >= 0 && p.Y >= 0 && p.X < Width && p.Y < Height);
		}

		/// <summary>
		/// Produce a Bitmap from this PixelMap.</summary>
		public Bitmap GetBitmap()
		{
			var bitmap = new Bitmap(Width,Height);

			var data = bitmap.LockBits(new Rectangle(0,0,Width,Height),ImageLockMode.WriteOnly,format);
			int bytes = Math.Abs(data.Stride) * bitmap.Height;

			byte[] raw = new byte[bytes];

			for (int x = 0; x < Width; x++)
			{
				for (int y = 0; y < Height; y++)
				{
					int offset = (y * data.Width + x) * BPP;

					Pixel p = this[x,y];

					raw[offset] = p.B;//BLUE
					offset++;
					raw[offset] = p.G;//GREEN
					offset++;
					raw[offset] = p.R;//RED
					if (BPP == 4)
					{
						offset++;
						raw[offset] = p.A;//ALPHA
					}

				}
			}

			System.Runtime.InteropServices.Marshal.Copy(raw,0,data.Scan0,bytes);
			bitmap.UnlockBits(data);

			return bitmap;
		}
		/// <summary>
		/// Load a Bitmap pixel-by-pixel, slowly.</summary>
		public static PixelMap SlowLoad(Bitmap b)
		{
			PixelMap m = new PixelMap(b.Width,b.Height);

			for (int x = 0; x < b.Width; x++)
			{
				for (int y = 0; y < b.Height; y++)
				{
					m[x,y] = new Pixel(b.GetPixel(x,y));
				}
			}

			return m;
		}

	}
}
