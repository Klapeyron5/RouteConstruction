using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Drawing;

namespace RouteConstruction
{
    class Astar
    {
        public Astar(Bitmap bmp, ushort startx, ushort starty, ushort finishx, ushort finishy, byte m)
        {
            bitmap = bmp;
            bmpWidth = bitmap.Width;
            bmpHeight = bitmap.Height;

            startX = startx;
            startY = starty;
            finishX = finishx;
            finishY = finishy;

			mode = m;

            researchPoints.AddFirst(new ushort[2] { startX, startY});
            route.Add(new ushort[2] { finishX, finishY});

			ushort n;
			switch(m)
			{
				case 1:
					n = 1;
					break;
				case 2:
					n = 10;
					break;
				default:
					n = 1;
					break;
			}

			pointsInfo = new ushort[bmpWidth,bmpHeight,5];
			for (int i = 0; i < bmpWidth; i++)
				for (int j = 0; j < bmpHeight; j++)
				{
					pointsInfo[i,j,0] = 0;
					if (bitmap.GetPixel(i,j) == MapPreprocessing.colors[2])
						pointsInfo[i,j,1] = (ushort)(14*n);
					else if (bitmap.GetPixel(i,j) == MapPreprocessing.colors[1])
						pointsInfo[i,j,1] = (ushort)(12*n);
					else if (bitmap.GetPixel(i,j) == MapPreprocessing.colors[0])
						pointsInfo[i,j,1] = (ushort)(10*n);
					else pointsInfo[i,j,1] = 0;
					pointsInfo[i,j,2] = 0;
					pointsInfo[i,j,3] = 0;
					pointsInfo[i,j,4] = 0;
				}

			pointsInfo[startX,startY,0] = 1;
		}
		
		byte mode;

        private Bitmap bitmap;
        private int bmpWidth; //TODO: short Exception
        private int bmpHeight;
        
        private ushort startX;
        private ushort startY;
        private ushort finishX;
        private ushort finishY;
        private List<ushort[]> route = new List<ushort[]>();
        
        private ushort[,,] pointsInfo; //[x,y,k]; k = 0-closed; 1-F; 2-G; 3-parentX; 4-parentY; (mode 2: 5-sit at researchPoints;)
		private LinkedList<ushort[]> researchPoints = new LinkedList<ushort[]>();
        private bool FinishHasReached = false;
        private bool flag = true;
		private ushort G;
		private ushort Gbuffer;

		public List<ushort[]> getRoute()
		{
				switch(mode)
				{
					case 1:
						return getNPiIn2Route();
					case 2:
						return getNPiIn4Route();
					default:
						return getNPiIn4Route();
				}
		}

        private List<ushort[]> getNPiIn2Route()
        {
            ushort X, Y;
            while((researchPoints.Count!=0)&&(!FinishHasReached))
            {
                X = researchPoints.First.Value[0];
                Y = researchPoints.First.Value[1];
                researchPoints.RemoveFirst();
				ushort p = (ushort)(X+1);
				getNPiIn2RouteRepeatedCode(ref p, ref Y, ref X, ref Y);
				p = (ushort)(X-1);
				getNPiIn2RouteRepeatedCode(ref p, ref Y, ref X, ref Y);
				p = (ushort)(Y+1);
				getNPiIn2RouteRepeatedCode(ref X, ref p, ref X, ref Y);
				p = (ushort)(Y-1);
				getNPiIn2RouteRepeatedCode(ref X, ref p, ref X, ref Y);
            }

            if (FinishHasReached)
            {
                X = route[route.Count - 1][0];
                Y = route[route.Count - 1][1];
                while (!((X == startX) && (Y == startY)))
                {
                    route.Add(new ushort[2] { pointsInfo[X, Y, 3], pointsInfo[X, Y, 4] });
                    X = route[route.Count - 1][0];
                    Y = route[route.Count - 1][1];
                }
                route.Reverse();
            }
            else MessageBox.Show("Robot can not reach the target.");
            
            return route;
        }

        private void getNPiIn2RouteRepeatedCode(ref ushort X, ref ushort Y, ref ushort parentX, ref ushort parentY)
        {
            if ((bitmap.GetPixel(X,Y)!=MapPreprocessing.wallColor)&&
                    (pointsInfo[X,Y,0]==0)&&(!((X==finishX)&&(Y==finishY))))
			{
				pointsInfo[X,Y,0] = 1;
				pointsInfo[X,Y,3] = parentX;
				pointsInfo[X,Y,4] = parentY;
				pointsInfo[X,Y,2] = (ushort)(pointsInfo[parentX,parentY,2]+1);
				pointsInfo[X,Y,1] += (ushort)(pointsInfo[X,Y,2]+Math.Abs(X-finishX)+Math.Abs(Y-finishY));

				flag=true;
				LinkedListNode<ushort[]> current = researchPoints.First;
				foreach (ushort[] p in researchPoints)
				{
					if (pointsInfo[X,Y,1]<pointsInfo[p[0],p[1],1])
					{
						researchPoints.AddBefore(current, new ushort[] {X,Y});
						flag=false;
						break;
					}
					current=current.Next;
				}
				if (flag) researchPoints.AddLast(new ushort[] {X,Y});
			} else if ((X==finishX)&&(Y==finishY))
			{
				pointsInfo[X,Y,0]=1;
				pointsInfo[X,Y,3]=parentX;
				pointsInfo[X,Y,4]=parentY;
				FinishHasReached=true;
			}
		}

		private List<ushort[]> getNPiIn4Route()
        {
			Console.WriteLine("getNPiIn4Route");
            ushort X, Y;
            while((researchPoints.Count != 0) && (!FinishHasReached))
            {
                X = researchPoints.First.Value[0];
                Y = researchPoints.First.Value[1];
                researchPoints.RemoveFirst();
				pointsInfo[X,Y,0] = 2;
				//открываем квадрат
				//для каждой точки квадрата (от минимальной):
				//добавляем ее в закрытый
				//добавляем в открытый те, которые еще не в открытом (с текущим родителем)
				//для каждой, что уже в открытом, проверяем не короче ли путь к ней через текущую (сморим только на G)
				//если так, то меняем родителя и обновляем веса, удаляем и заново добавляем ее в LinkedList

				ushort p1 = (ushort)(X+1);
				ushort p2 = (ushort)(Y+1);
				getNPiIn4RouteRepeatedCodeNormal(ref p1,ref Y,ref X,ref Y);
				getNPiIn4RouteRepeatedCodeDiagonal(ref p1,ref p2,ref X,ref Y);
				getNPiIn4RouteRepeatedCodeNormal(ref X,ref p2,ref X,ref Y);
				p1 = (ushort)(X-1);
				getNPiIn4RouteRepeatedCodeNormal(ref p1,ref Y,ref X,ref Y);
				getNPiIn4RouteRepeatedCodeDiagonal(ref p1,ref p2,ref X,ref Y);
				p2 = (ushort)(Y-1);
				getNPiIn4RouteRepeatedCodeNormal(ref X,ref p2,ref X,ref Y);
				getNPiIn4RouteRepeatedCodeDiagonal(ref p1,ref p2,ref X,ref Y);
				p1 = (ushort)(X+1);
				getNPiIn4RouteRepeatedCodeDiagonal(ref p1,ref p2,ref X,ref Y);
			}

			if (FinishHasReached)
			{
				X = route[route.Count - 1][0];
				Y = route[route.Count - 1][1];
				while (!((X == startX) && (Y == startY)))
				{
					route.Add(new ushort[2] { pointsInfo[X,Y,3],pointsInfo[X,Y,4] });
					X = route[route.Count - 1][0];
					Y = route[route.Count - 1][1];
				}
				route.Reverse();
			}
			else MessageBox.Show("Robot can not reach the target.");

			return route;
        }

		private void getNPiIn4RouteRepeatedCodeNormal(ref ushort X, ref ushort Y, ref ushort parentX, ref ushort parentY)
		{
			G = 10;
			if (bitmap.GetPixel(X,Y)!=MapPreprocessing.wallColor)
				getNPiIn4RouteRepeatedCode(ref X,ref Y,ref parentX,ref parentY,ref G);
		}

		private void getNPiIn4RouteRepeatedCodeDiagonal(ref ushort X,ref ushort Y,ref ushort parentX,ref ushort parentY)
		{
			G = 14;
			if ((bitmap.GetPixel(X,parentY) != MapPreprocessing.wallColor) &&
				(bitmap.GetPixel(parentX,Y) != MapPreprocessing.wallColor))
				getNPiIn4RouteRepeatedCode(ref X,ref Y,ref parentX,ref parentY,ref G);
		}

		private void getNPiIn4RouteRepeatedCode(ref ushort X,ref ushort Y,ref ushort parentX,ref ushort parentY, ref ushort G)
		{
			if (!((X == finishX) && (Y == finishY)))
			{
				if (pointsInfo[X,Y,0] == 0)
				{
					pointsInfo[X,Y,0] = 1;
					pointsInfo[X,Y,3] = parentX;
					pointsInfo[X,Y,4] = parentY;
					pointsInfo[X,Y,2] = (ushort)(pointsInfo[parentX,parentY,2] + G);
					pointsInfo[X,Y,1] += (ushort)(pointsInfo[X,Y,2] + Math.Abs(X - finishX) + Math.Abs(Y - finishY));

					flag = true;
					LinkedListNode<ushort[]> current = researchPoints.First;
					foreach (ushort[] p in researchPoints)
					{
						if (pointsInfo[X,Y,1] < pointsInfo[p[0],p[1],1])
						{
							researchPoints.AddBefore(current,new ushort[] { X,Y });
							flag = false;
							break;
						}
						current = current.Next;
					}
					if (flag) researchPoints.AddLast(new ushort[] { X,Y });
				} else if (pointsInfo[X,Y,0] == 1)
				{
					Gbuffer = (ushort)(pointsInfo[parentX,parentY,2]+G);
					if (Gbuffer < pointsInfo[X,Y,2])
					{
						pointsInfo[X,Y,1] = (ushort)(pointsInfo[X,Y,1] - pointsInfo[X,Y,2] + Gbuffer);
						pointsInfo[X,Y,2] = Gbuffer;
						
						LinkedListNode<ushort[]> current = researchPoints.First;
						foreach (ushort[] p in researchPoints)
						{
							if ((p[0] == X) && (p[1] == Y))
							{
								researchPoints.Remove(current);
								break;
							}
							current = current.Next;
						}
						current = researchPoints.First;
						foreach (ushort[] p in researchPoints)
						{
							if (pointsInfo[X,Y,1] < pointsInfo[p[0],p[1],1])
							{
								researchPoints.AddBefore(current,new ushort[] { X,Y });
								break;
							}
							current = current.Next;
						}
					}
				}
			} else if ((X == finishX) && (Y == finishY))
			{
				pointsInfo[X,Y,0] = 1;
				pointsInfo[X,Y,3] = parentX;
				pointsInfo[X,Y,4] = parentY;
				FinishHasReached = true;
			}
		}

		public void printMap()
		{
			for (int j = 0; j<bmpHeight; j++)
			{
				for (int i = 0; i<bmpWidth; i++)
					Console.Write("\t"+pointsInfo[i, j, 1]);
				Console.WriteLine();
			}
		}
	}

    class MapPreprocessing
    {
        public MapPreprocessing()
        {
            for (int k = rPenalty; k >= 0; k--)
                radiuses[rPenalty - k] = rRobot + k;
        }

        private static int rRobot = Form1.robotRadius;
        private static int rPenalty = 3;
        private static int[] radiuses = new int[rPenalty+1];

        public static Color wallColor = Color.FromArgb(255, 255, 255);
        private static int penaltyColorRedStep = 70;
        public static Color[] colors = { Color.FromArgb(penaltyColorRedStep * 3, 200, 0),
            Color.FromArgb(penaltyColorRedStep*2,200,0),Color.FromArgb(penaltyColorRedStep*1,200,0),wallColor};

        public Bitmap getProcessedBitmap(ref Bitmap bitmap)
        {
            Bitmap bmp = new Bitmap(bitmap);
            Graphics graphics = Graphics.FromImage(bmp);

            for (int k = 0; k < colors.Length; k++)
            {
                for (int i = 0; i < bitmap.Width; i++)
                    for (int j = 0; j < bitmap.Height; j++)
                        if (bitmap.GetPixel(i, j) == wallColor)
                        {
                            graphics.FillRectangle(new SolidBrush(colors[k]), //DO NOT USE DrawRectangle!
                                i - radiuses[k], j - radiuses[k], 2 * radiuses[k] + 1, 2 * radiuses[k] + 1);
                        }
            }
	//		bmp.Save(@"C:\Adocuments\Library\Clapeyron_ind\task3 построение пути\preprocessed.bmp");
			return bmp;
        }
    }
}
