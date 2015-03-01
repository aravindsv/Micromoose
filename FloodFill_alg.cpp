const int INFINITE_VAL = 100000000;

const int NORTH = 0;
const int EAST = 1;
const int SOUTH = 2;
const int WEST = 3;

const int MAZE_W = 15;
const int MAZE_L = 15;

int dist_array[MAZE_L][MAZE_W];

struct Coord 
{
	public:
		Coord(int distance_val)
		{
			m_isProcessed = false;
			m_distance = distance_val;
		}
		int r_val()
		{
			return m_rVal;
		}
		int c_val()
		{
			return m_cVal;
		}
		void process()
		{
			m_isProcessed = true;
		}
		bool isProcessed()
		{
			return m_isProcessed;
		}
	private:
		int m_rVal;
		int m_cVal;
		bool m_isProcessed;
		int m_distance;
};

int dist(Coord pos)
{
	int x = pos.r_val();
	int y = pos.c_val();
	return dist_array[x][y];
}

void setDist(Coord pos, int distance)
{
	int x = pos.r_val();
	int y = pos.c_val();
	dist_array[x][y] = distance;
}

void floodFill(Coord curPos)
{
	Stack stack = new Stack<Coord>;
	stack.push(curPos);
	while (!stack.isEmpty())
	{
		Coord cur = stack.pop();
		cur.process();
		if (dist(cur) == 0)
		{
			return;
		}
		int shortest = INFINITE_VAL;
		for (int i = 0; i < 4; i++)
		{
			Coord neighbor = cur.moveDir(i);
			if (!cur.isWallBetween(neighbor))
			{
				if (dist(neighbor) < shortest)
				{
					shortest = dist(neighbor);
				}
				if (!neighbor.isProcessed())
				{
					stack.push(neighbor);
				}
			}
		}
		if (shortest == INFINITE_VAL)
		{
			continue;
		}
		if (dist(cur) == (shortest+1))
		{
			continue;
		}
		setDist(cur, shortest+1);
		for (int i = 0; i < 4; i++)
		{
			Coord neighbor = cur.moveDir(i);
			if (!cur.isWallBetween(neighbor))
			{
				stack.push(neighbor);
			}
		}
	}
}