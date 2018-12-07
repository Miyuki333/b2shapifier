module dbox.collision.shapes.b2shapifier;

import dbox.common;
import dbox.collision.shapes.b2polygonshape;

import std.math;

//generate an array of b2PolygonShapes from a 32-bit image with a transparency channel
class b2Shapifier
{
	
	static:

	b2PolygonShape[] convert(const(ubyte)[] image, uint width, uint height, b2Vec2 scale = b2Vec2(1.0f, 1.0f), float32 tolerance = 1.0f)
	{	
		b2Vec2[] points;
		b2Vec2 point;
		uint[2] coords;
		b2Vec2 center = b2Vec2(cast(float32) width / 2.0f, cast(float32) height / 2.0f);

		//create a tracer to outline the image
		Tracer tracer = new Tracer(image, width, height);

		//find the first pixel above the tracer's threshold to use as a starting point
		coords = tracer.first();
		point = b2Vec2(cast(float32) coords[0], cast(float32) coords[1]);
		points ~= point;

		//complete the outline until we cycle back to the first point
		do
		{
			coords = tracer.next();
			point = b2Vec2(cast(float32) coords[0], cast(float32) coords[1]);
			points ~= point;
		} while (point != points[0]);

		//simplify points to tolerance level
		points = simplify(points, tolerance);

		//convert points to triangles
		points = Triangulator.convert(points[0..$-1]);

		//check for triangulation failure
		if (points.length % 3) return [];

		//convert triangles to shapes
		b2PolygonShape shape;
		b2PolygonShape[] shapes;

		for (uint i = 0; i < points.length; i += 3)
		{
			b2Vec2 a = (points[i] - center) * scale;
			b2Vec2 b = (points[i + 1] - center) * scale;
			b2Vec2 c = (points[i + 2] - center) * scale;

			shape = new b2PolygonShape();
			shape.Set([a, b, c]);
			shapes ~= shape;
		}

		return shapes;
	}

	b2Vec2[] simplify(b2Vec2[] points, float32 tolerance)
	{
		Segment segment = new Segment(points, tolerance);
		Segment current;
		bool resolved = false;

		//split segment using the tolerance value until it is completely resolved
		while (resolved != true)
		{
			current = segment;
			resolved = true;

			while(current !is null)
			{
				current.split();
				if(current.resolved != true) resolved = false;
				current = current.next;
			}
		}

		//collect points from resolved segments into an array and return it
		b2Vec2 last = points[$-1];
		points = null;
		current = segment;

		while(current !is null)
		{
			points ~= current.points[0];
			current = current.next;
		}

		//make sure to append the final point to the point array before returning
		return points ~ last;
	}

	//this class is used to split a segment of points at the point
	//which is most out of line with the other points on the segment
	class Segment
	{
		b2Vec2[] points;
		float32 tolerance;
		bool resolved = false;
		protected Segment m_next;

		this(b2Vec2[] points, float32 tolerance)
		{
			this.points = points;
			this.tolerance = tolerance;
			this.resolved = false;
		}

		Segment split()
		{
			//return early if the segment is already resolved
			if (resolved == true) return null;
			if (points.length <= 2) { resolved = true; return null; }

			//treat the first and last points in the segment as a line
			b2Vec2 first = points[0];
			b2Vec2 last = points[$-1];

			//look for the outlier point with the farthest distance from that line
			float max = tolerance;
			uint max_index = 0;

			foreach(uint i, b2Vec2 point; points[1..$-1])
			{
				float distance = distance(point.x, point.y, first.x, first.y, last.x, last.y);
				if (distance > max) { max = distance; max_index = i + 1; }
			}

			if (max_index == 0) { resolved = true; return null; }

			next = new Segment(points[max_index..$], tolerance);
			points = points[0..max_index+1];

			return next;
		}

		Segment next() @property
		{
			return m_next;
		}

		Segment next(Segment segment) @property
		{
			if (m_next !is null) segment.next = m_next;
			return m_next = segment;
		}

		//find the distance between a point ([x, y]) and a line ([x1, y1] to [x2, y2])
		//copypasta from: http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
		protected float distance(float x, float y, float x1, float y1, float x2, float y2)
		{
			float A = x - x1;
			float B = y - y1;
			float C = x2 - x1;
			float D = y2 - y1;

			float dot = A * C + B * D;
			float len_sq = C * C + D * D;
			float param = len_sq == 0 ? -1 : dot / len_sq;

			float xx, yy;
			if (param < 0) { xx = x1; yy = y1; }
			else if (param > 1) { xx = x2; yy = y2; }
			else { xx = x1 + param * C; yy = y1 + param * D; }

			float dx = x - xx;
			float dy = y - yy;
			return sqrt(dx * dx + dy * dy);
		}

	}

	//takes an image with an alpha channel and creates
	//an array with all points on it's outline
	class Tracer
	{

		ubyte[] alpha;
		uint[2] dimensions;
		uint[2] position = [0, 0];
		ubyte threshold = 0x01;
		ubyte direction = 4;
		ubyte[9] rotate;

		protected static ubyte[9] rotate_clockwise = [3, 0, 1, 6, 4, 2, 7, 8, 5];
		protected static ubyte[9] rotate_counterclockwise = [1, 2, 5, 0, 4, 8, 3, 6, 7];

		this(const(ubyte)[] image, uint width, uint height, bool clockwise = false)
		{
			dimensions = [width, height];

			//extract the image's alpha channel
			alpha = new ubyte[](width * height);
			for (int i = 0; i < width * height; ++i) alpha[i] = image[(i * 4) + 3];

			//set the rotation direction
			rotate = clockwise == true ? rotate_clockwise : rotate_counterclockwise;
		}

		//search for the first pixel above the threshhold and initialize the trace direction
		uint[2] first(uint start = 0)
		{
			for (uint i = start; i < alpha.length; ++i)
			{
				if(alpha[i] > threshold)
				{
					position = [i % dimensions[0], i / dimensions[0]];
					direction = 5;
					return position;
				}
			}
				
			position = [0, 0];
			direction = 4;
			return position;
		}

		uint[2] next()
		{
			//set up variables
			uint position_x = position[0];
			uint position_y = position[1];
			uint width = dimensions[0];
			uint height = dimensions[1];
			uint x;
			uint y;

			//invert the direction so that the next search start one pixel after the last found pixel
			direction = cast(ubyte) (8 - direction);

			//rotate the direction clockwise until a pixel above the threshold is found
			for (uint i = 0; i < 9; ++i)
			{
				direction = rotate[direction];

				switch (direction)
				{
					case 0: y = position_y + 1; x = position_x - 1; break;
					case 1: y = position_y + 1; x = position_x; break;
					case 2: y = position_y + 1; x = position_x + 1; break;
					case 3: y = position_y; x = position_x - 1; break;
					case 4: y = position_y; x = position_x; break;
					case 5: y = position_y; x = position_x + 1; break;
					case 6: y = position_y - 1; x = position_x - 1; break;
					case 7: y = position_y - 1; x = position_x; break;
					case 8: y = position_y - 1; x = position_x + 1; break;
					default: y = position_y; x = position_x; break;
				}

				//check over/underflow
				if (x >= width || y >= height) { x = position_x; y = position_y; continue; }

				//find offset of pixel and check it's value
				uint offset = y * width + x;
				if(alpha[offset] > threshold) break;
			}

			//advance to the position of the found pixel and return the new position
			return position = [x, y];
		}

	}

	//converts an array of points into polygons
	//based on "Efficient Polygon Triangulation" by John W. Ratcliff
	//http://www.flipcode.com/archives/Efficient_Polygon_Triangulation.shtml
	class Triangulator
	{

		static:

		const float EPSILON = 0.0000000001f;

		float area(b2Vec2[] contour)
		{
			int n = contour.length;

			float a = 0.0f;

			for(int p = n-1, q = 0; q < n; p = q++)
			{
				a += contour[p].x * contour[q].y - contour[q].x * contour[p].y;
			}

			return a * 0.5f;
		}

		//inside_triangle decides if a point P is Inside of the triangle defined by A, B, C
		bool inside_triangle(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float Px, float Py)
		{
			float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
			float cCROSSap, bCROSScp, aCROSSbp;

			ax = Cx - Bx;  ay = Cy - By;
			bx = Ax - Cx;  by = Ay - Cy;
			cx = Bx - Ax;  cy = By - Ay;
			apx= Px - Ax;  apy= Py - Ay;
			bpx= Px - Bx;  bpy= Py - By;
			cpx= Px - Cx;  cpy= Py - Cy;

			aCROSSbp = ax*bpy - ay*bpx;
			cCROSSap = cx*apy - cy*apx;
			bCROSScp = bx*cpy - by*cpx;

			return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
		};

		bool snip(b2Vec2[] contour, int u, int v, int w, int n, int[] V)
		{
			int p;
			float Ax, Ay, Bx, By, Cx, Cy, Px, Py;

			Ax = contour[V[u]].x;
			Ay = contour[V[u]].y;

			Bx = contour[V[v]].x;
			By = contour[V[v]].y;

			Cx = contour[V[w]].x;
			Cy = contour[V[w]].y;

			if ( EPSILON > ((Bx-Ax)*(Cy-Ay) - (By-Ay)*(Cx-Ax)) ) return false;

			for (p=0; p < n; p++)
			{
				if( (p == u) || (p == v) || (p == w) ) continue;
				Px = contour[V[p]].x;
				Py = contour[V[p]].y;
				if (inside_triangle(Ax, Ay, Bx, By, Cx, Cy, Px, Py)) return false;
			}

			return true;
		}

		b2Vec2[] convert(b2Vec2[] contour)
		{
			b2Vec2[] result;

			/* allocate and initialize list of Vertices in polygon */

			int n = contour.length;
			if ( n < 3 ) return contour;

			int[] V = new int[](n);

			/* we want a counter-clockwise polygon in V */

			if ( 0.0f < area(contour) )
				for (int v = 0; v < n; v++) V[v] = v;
			else
				for(int v = 0; v < n; v++) V[v] = (n - 1) - v;

			int nv = n;

			/*  remove nv-2 Vertices, creating 1 triangle every time */
			int count = 2 * nv; /* error detection */

			for(int m = 0, v = nv  -1; nv > 2; )
			{
				/* if we loop, it is probably a non-simple polygon */
				if (0 >= (count--))
				{
					throw new Exception("Bad or complex polygon.");
				}

				/* three consecutive vertices in current polygon, <u,v,w> */
				int u = v; if (nv <= u) u = 0; /* previous */
				v = u + 1; if (nv <= v) v = 0; /* new v */
				int w = v + 1; if (nv <= w) w = 0; /* next */

				if ( snip(contour, u, v, w, nv, V) )
				{
					int a, b, c, s, t;

					/* true names of the vertices */
					a = V[u]; b = V[v]; c = V[w];

					/* output Triangle */
					result ~= contour[a];
					result ~= contour[b];
					result ~= contour[c];

					m++;

					/* remove v from remaining polygon */
					for(s = v, t = v + 1; t < nv; s++, t++) { V[s] = V[t]; }
					nv--;

					/* resest error detection counter */
					count = 2 * nv;
				}
			}

			return result;
		}


	}

}
