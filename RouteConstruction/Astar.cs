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

            researchPoints.AddFirst(new ushort[2] { startX, startY}); //добавляем в точки для исследования первую
            route.Add(new ushort[2] { finishX, finishY}); //добавляем в маршрут первой точкой финиш (строим маршрут с конца)

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
			for (int i = 0; i < bmpWidth; i++) //если точка из штрафной зоны, то добавляем ей штрафы на каждый цвет - свой
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

			pointsInfo[startX,startY,0] = 1; //говорим, что старт уже в закрытом списке (т.е. мы уже решили добавлять его в маршрут
                                             //или нет (добавлять, конечно))
		}
		
		byte mode;

        private Bitmap bitmap;
        private int bmpWidth; //TODO: short Exception
        private int bmpHeight;
        
        private ushort startX;
        private ushort startY;
        private ushort finishX;
        private ushort finishY;

        /// <summary>
        /// Итоговый маршрут
        /// </summary>
        private List<ushort[]> route = new List<ushort[]>();
        
        /// <summary>
        /// Информация о точках: pointsInfo[x,y,k];
        /// k = 
        /// 0 - находится в закрытом списке (0 или 1);
        /// 1 - F метрика: общая метрика = G + |x-finishX| + |y-finishY| + имеющиеся штрафы на перемещение;
        /// 2 - G метрика: 10*кол-во прямых элем. путей + 14*кол-во диагональных элем. путей (элем. путь - соединяет две соседние клетки);
        /// 3 - parentX;
        /// 4 - parentY;
        /// (mode 2: 5-sit at researchPoints;)
        /// </summary>
        private ushort[,,] pointsInfo;

        /// <summary>
        /// Точки - кандидаты для включения в маршрут.
        /// </summary>
		private LinkedList<ushort[]> researchPoints = new LinkedList<ushort[]>();

        /// <summary>
        /// True, если в researchPoints попала финишная точка.
        /// </summary>
        private bool FinishHasReached = false;

        private bool flag = true;
		private ushort G;
		private ushort Gbuffer;

        /// <summary>
        /// Возвращает маршрут от старта до финиша (заданных через конструктор)
        /// Или пишет, что невозможно достигнуть финиша.
        /// </summary>
        /// <returns></returns>
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

        /// <summary>
        /// Рассчитывает путь с возможностью движения по направлениям, кратным pi/4.
        /// </summary>
        /// <returns></returns>
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

                //проверяем все 8 точек, окружающих текущую
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

			if (FinishHasReached)   //если был достигнут финиш, то идем с финиша к родителю,   
            {                       //затем от родителя финиша к его родителю и т.д. до конца списка
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
			else MessageBox.Show("Robot can not reach the target."); //Финиш закрыт препятствиями

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

        /// <summary>
        /// Добавляет точку (X,Y) в список кандидатов на включение в маршрут в порядке возрастания метрики F
        /// </summary>
        /// <param name="X"></param>
        /// <param name="Y"></param>
        /// <param name="parentX"></param>
        /// <param name="parentY"></param>
        /// <param name="G"></param>
		private void getNPiIn4RouteRepeatedCode(ref ushort X,ref ushort Y,ref ushort parentX,ref ushort parentY, ref ushort G)
		{
			if (!((X == finishX) && (Y == finishY))) //если рассматриваемая точка не финиш
            { 
                if (pointsInfo[X,Y,0] == 0)     //если точка не в закрытом списке, то добавляем точку(X, Y) в список
                {                               //кандидатов на включение в маршрут в порядке возрастания метрики F
                    pointsInfo[X,Y,0] = 1; //говорим, что теперь она в закрытом списке
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
				} else if (pointsInfo[X,Y,0] == 1) //если точка в закрытом списке
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
			} else if ((X == finishX) && (Y == finishY)) //если рассматриваемая точка - финиш
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


    /// <summary>
    /// Рисует вокруг каждой белой точки белый квадрат со стороной 2robotRadius+1,
    /// затем добавляет вокруг квадрата штрафные зоны (такие же квадраты, только длина стороны+2, следующая зона еще+2 и т.д.)
    /// попиксельно с разными цветами.</summary>
    static class MapPreprocessing
    {
        static MapPreprocessing()
        {
            for (int k = rPenalty; k >= 0; k--)
                radiuses[rPenalty - k] = rRobot + k; //[9,8,7,6]
        }

        private static int rRobot = Form1.robotRadius; //6
        private static int rPenalty = 3;
        private static int[] radiuses = new int[rPenalty+1]; //6+3 (1.5 robotRadius)

        public static Color wallColor = Color.FromArgb(255, 255, 255);
        private static int penaltyColorRedStep = 70;

        //норм, не очень, совсем не очень, стена
        public static Color[] colors = { Color.FromArgb(penaltyColorRedStep * 3, 200, 0),
            Color.FromArgb(penaltyColorRedStep*2,200,0),Color.FromArgb(penaltyColorRedStep*1,200,0),wallColor};

        /// <summary>
        /// Возвращает битмап, где отмечены зоны для езды робота, включая штрафные.
        /// </summary>
        /// <param name="bitmap">Ссылка на битмап - карту помещения.</param>
        /// <returns></returns>
        public static Bitmap getProcessedBitmap(ref Bitmap bitmap)
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
