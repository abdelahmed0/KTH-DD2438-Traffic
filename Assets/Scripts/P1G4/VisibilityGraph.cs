using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Graphs {
    public class VisibilityGraphManager {
        public static List<Point> allVertices = new List<Point>();
        public static List<Tuple<Point, Point>> visibleEdges = new List<Tuple<Point, Point>>();
        public static List<Point> GetAllVertices() {
            return new List<Point>(allVertices);
        }
        public static List<Tuple<Point, Point>> GetVisibleEdges() {
            return new List<Tuple<Point, Point>>(visibleEdges);
        }
        public static void AddEdge(Point x, Point y) {
            Tuple<Point, Point> edge = new Tuple<Point, Point>(x, y);
            Tuple<Point, Point> reverseEdge = new Tuple<Point, Point>(y, x);
            // Add the edge if neither this nor its reverse already exist
            if (!visibleEdges.Contains(edge) && !visibleEdges.Contains(reverseEdge)) {
                visibleEdges.Add(edge);
            }
        }
    }
    public class Point {
        public Point () {
            this.x = 0;
            this.z = 0;
        }
        public Point (float x, float z) {
            this.x = x;
            this.z = z;
        }

        public Point(Vector3 p) {
            this.x = p.x;
            this.z = p.z;
        }

        static private bool CCW(Point a, Point b, Point c) {
            return (c.z-a.z)*(b.x-a.x) > (b.z-a.z)*(c.x-a.x);
        }

        static public bool Intersect(Point a, Point b, Point c, Point d) {
            if (a == c || b == c || a == d || b == d) return false;
            return CCW(a,c,d) != CCW(b,c,d) && CCW(a,b,c) != CCW(a,b,d);
        }

        public Vector3 ToVector3() {
            return new Vector3(this.x, 0.05f, this.z);
        }

        public Vector2 ToVector2() {
            return new Vector2(this.x, this.z);
        }
        public static Point Lerp(Point a, Point b, float t)
        {
            return new Point(
                a.x + (b.x - a.x) * t,
                a.z + (b.z - a.z) * t);
        }

        // Function to add two points
        public static Point operator +(Point a, Point b)
        {
            return new Point(a.x + b.x, a.z + b.z);
        }

        // Function to subtract two points
        public static Point operator -(Point a, Point b)
        {
            return new Point(a.x - b.x, a.z - b.z);
        }

            // Function to multiply a point by a scalar
        public static Point operator *(Point a, float scalar)
        {
            return new Point(a.x * scalar, a.z * scalar);
        }

            // Function to divide a point by a scalar
        public static Point operator /(Point a, float scalar)
        {
            return new Point(a.x / scalar, a.z / scalar);
        }

            // Function to get the magnitude of a point/vector
        public float Magnitude()
        {
            return (float)Math.Sqrt(this.x * this.x + this.z * this.z);
        }

        // Function to normalize a point/vector
        public Point Normalize()
        {
            float magnitude = Magnitude();
            return new Point(this.x / magnitude, this.z / magnitude);
        }

        public static Point Interpolate(Point p0, Point p1, Point p2, float t)
        {
            float t1 = 1.0f - t;
            float t1Sqr = t1 * t1;
            float tSqr = t * t;

            float newX = t1Sqr * p0.x + 2 * t1Sqr * t * p1.x + tSqr * p2.x;
            float newZ = t1Sqr * p0.z + 2 * t1Sqr * t * p1.z + tSqr * p2.z;

            return new Point(newX, newZ);
        }

        internal float x,z;
    }

    class Obstacle {
        public Obstacle(Point a, Point b, Point c, Point d) {
            this.points = new Point[4];
            this.points[0] = a;
            this.points[1] = b;
            this.points[2] = c;
            this.points[3] = d;

            this.CalculateMinMax();

            AddVerticesToGlobalList();
        }

        public Obstacle(Collider objectCollider, bool right, float high=0.01f) {
            var bounds = objectCollider.bounds;
            RaycastHit hit;
            this.xMax = bounds.max.x;
            this.zMax = bounds.max.z;
            this.xMin = bounds.min.x;
            this.zMin = bounds.min.z;
            if (right) {
                Physics.Raycast(new Vector3(bounds.center.x, high, bounds.center.z), Vector3.right, out hit, 10f);
                this.xMin = hit.point.x;
            } else {
                Physics.Raycast(new Vector3(bounds.center.x, high, bounds.center.z), Vector3.left, out hit, 10f);
                this.xMax = hit.point.x;
            }

            this.CalculatePoints();

            AddVerticesToGlobalList();
        }

        public Obstacle(Collider objectCollider, float highMax=0.01f, float highMin=0.01f, bool gate=false, Bounds testBounds=new Bounds()) {
            var bounds = objectCollider.bounds;
            if (gate)
            {
                bounds = testBounds;
            }
            float center_y = (highMax + highMin) / 2;
            Vector3 radius = new Vector3((bounds.max.x - bounds.min.x) / 2, (highMax - highMin) / 2, (bounds.max.z - bounds.min.z) / 2);
            List<float> x_axis = new List<float>();
            List<float> z_axis = new List<float>();
            RaycastHit hit;
            float margin = 0.2f;

            if (Physics.BoxCast(new Vector3(bounds.max.x + margin, center_y, (bounds.max.z + bounds.min.z) / 2), new Vector3(-0.1f, radius.y, radius.z), Vector3.left, out hit, Quaternion.identity, radius.x * 2 + margin)) {
                Debug.DrawLine(new Vector3(bounds.max.x + margin, center_y, (bounds.max.z + bounds.min.z) / 2), hit.point, Color.blue, 10000f);
                if (hit.collider == objectCollider) {
                    x_axis.Add(hit.point.x);
                    z_axis.Add(hit.point.z);
                }
            }
            if (Physics.BoxCast(new Vector3(bounds.min.x - margin, center_y, (bounds.max.z + bounds.min.z) / 2), new Vector3(0.1f, radius.y, radius.z), Vector3.right, out hit, Quaternion.identity, radius.x * 2 + margin)) {
                Debug.DrawLine(new Vector3(bounds.min.x - margin, center_y, (bounds.max.z + bounds.min.z) / 2), hit.point, Color.blue, 10000f);
                if (hit.collider == objectCollider) {
                    x_axis.Add(hit.point.x);
                    z_axis.Add(hit.point.z);
                }
            }
            if (Physics.BoxCast(new Vector3((bounds.max.x + bounds.min.x) / 2, center_y, bounds.max.z + margin), new Vector3(radius.x, radius.y, -0.1f), Vector3.back, out hit, Quaternion.identity, radius.z * 2 + margin)) {
                Debug.DrawLine(new Vector3((bounds.max.x + bounds.min.x) / 2, center_y, bounds.max.z + margin), hit.point, Color.blue, 10000f);
                if (hit.collider == objectCollider) {
                    x_axis.Add(hit.point.x);
                    z_axis.Add(hit.point.z);
                }
            }
            if (Physics.BoxCast(new Vector3((bounds.max.x + bounds.min.x) / 2, center_y, bounds.min.z - margin), new Vector3(radius.x, radius.y, 0.1f), Vector3.forward, out hit, Quaternion.identity, radius.z * 2 + margin)) {
                Debug.DrawLine(new Vector3((bounds.max.x + bounds.min.x) / 2, center_y, bounds.min.z - margin), hit.point, Color.blue, 10000f);
                if (hit.collider == objectCollider) {
                    x_axis.Add(hit.point.x);
                    z_axis.Add(hit.point.z);
                }
            }

            if (Physics.Raycast(new Vector3(bounds.min.x - margin, highMin, (bounds.min.z + bounds.max.z) / 2), Vector3.right, out hit, radius.x * 2 + margin)) {
                x_axis.Add(hit.point.x);
            }
            if (Physics.Raycast(new Vector3(bounds.min.x - margin, highMax, (bounds.min.z + bounds.max.z) / 2), Vector3.right, out hit, radius.x * 2 + margin)) {
                x_axis.Add(hit.point.x);
            }

            if (Physics.Raycast(new Vector3(bounds.max.x + margin, highMin, (bounds.min.z + bounds.max.z) / 2), Vector3.left, out hit, radius.x * 2 + margin)) {
                x_axis.Add(hit.point.x);
            }
            if (Physics.Raycast(new Vector3(bounds.max.x + margin, highMax, (bounds.min.z + bounds.max.z) / 2), Vector3.left, out hit, radius.x * 2 + margin)) {
                x_axis.Add(hit.point.x);
            }

            if (Physics.Raycast(new Vector3((bounds.min.x + bounds.max.x) / 2, highMin, bounds.max.z + margin), Vector3.back, out hit, radius.z * 2 + margin)) {
                z_axis.Add(hit.point.z);
            }
            if (Physics.Raycast(new Vector3((bounds.min.x + bounds.max.x) / 2, highMax, bounds.max.z + margin), Vector3.back, out hit, radius.z * 2 + margin)) {
                z_axis.Add(hit.point.z);
            }

            if (Physics.Raycast(new Vector3((bounds.min.x + bounds.max.x) / 2, highMin, bounds.min.z - margin), Vector3.forward, out hit, radius.z * 2 + margin)) {
                z_axis.Add(hit.point.z);
            }
            if (Physics.Raycast(new Vector3((bounds.min.x + bounds.max.x) / 2, highMax, bounds.min.z - margin), Vector3.forward, out hit, radius.z * 2 + margin)) {
                z_axis.Add(hit.point.z);
            }

            this.xMax = x_axis.Count > 0 ? x_axis.Max() : bounds.max.x;
            this.xMin = x_axis.Count > 0 ? x_axis.Min() : bounds.min.x;
            this.zMax = z_axis.Count > 0 ? z_axis.Max() : bounds.max.z;
            this.zMin = z_axis.Count > 0 ? z_axis.Min() : bounds.min.z;

            this.CalculatePoints();

            AddVerticesToGlobalList();

        }


        // public Obstacle(Collider objectCollider, float highMax=0.01f, float highMin=0.01f) {
        //     var bounds = objectCollider.bounds;

        //     Vector3 min_edge = new Vector3(bounds.min.x, highMin, bounds.min.z);
        //     Vector3 max_edge = new Vector3(bounds.max.x, highMax, bounds.max.z);

        //     float distance = Vector3.Distance(min_edge, max_edge);

        //     RaycastHit hit;

        //     List<float> Min_x = new List<float>();

        //     if (Physics.Raycast(new Vector3(min_edge.x - 0.1f, min_edge.y, (min_edge.z + max_edge.z) / 2), Vector3.right, out hit, distance)) {
        //         Min_x.Add(hit.point.x);
        //     }
        //     if (Physics.Raycast(new Vector3(min_edge.x - 0.1f, max_edge.y, (min_edge.z + max_edge.z) / 2), Vector3.right, out hit, distance)) {
        //         Min_x.Add(hit.point.x);
        //     }

        //     List<float> Max_x = new List<float>();

        //     if (Physics.Raycast(new Vector3(max_edge.x + 0.1f, min_edge.y, (min_edge.z + max_edge.z) / 2), Vector3.left, out hit, distance)) {
        //         Max_x.Add(hit.point.x);
        //     }
        //     if (Physics.Raycast(new Vector3(max_edge.x + 0.1f, max_edge.y, (min_edge.z + max_edge.z) / 2), Vector3.left, out hit, distance)) {
        //         Max_x.Add(hit.point.x);
        //     }

        //     List<float> Max_z = new List<float>();

        //     if (Physics.Raycast(new Vector3((min_edge.x + max_edge.x) / 2, min_edge.y, max_edge.z + 0.1f), Vector3.back, out hit, distance)) {
        //         Max_z.Add(hit.point.z);
        //     }
        //     if (Physics.Raycast(new Vector3((min_edge.x + max_edge.x) / 2, max_edge.y, max_edge.z + 0.1f), Vector3.back, out hit, distance)) {
        //         Max_z.Add(hit.point.z);
        //     }

        //     List<float> Min_z = new List<float>();

        //     if (Physics.Raycast(new Vector3((min_edge.x + max_edge.x) / 2, min_edge.y, min_edge.z - 0.1f), Vector3.forward, out hit, distance)) {
        //         Min_z.Add(hit.point.z);
        //     }
        //     if (Physics.Raycast(new Vector3((min_edge.x + max_edge.x) / 2, max_edge.y, min_edge.z - 0.1f), Vector3.forward, out hit, distance)) {
        //         Min_z.Add(hit.point.z);
        //     }
            
        //     if (Physics.Raycast(new Vector3(max_edge.x, max_edge.y, max_edge.z), new Vector3(min_edge.x - max_edge.x, 0f, min_edge.z - max_edge.z), out hit, distance)) {
        //         Max_x.Add(hit.point.x);
        //         Max_z.Add(hit.point.z);
        //     }
        //     if (Physics.Raycast(new Vector3(min_edge.x, max_edge.y, min_edge.z), new Vector3(max_edge.x - min_edge.x, 0f, max_edge.z - min_edge.z), out hit, distance)) {
        //         Min_x.Add(hit.point.x);
        //         Min_z.Add(hit.point.z);
        //     }

        //     if (Physics.Raycast(new Vector3(max_edge.x, min_edge.y, max_edge.z), new Vector3(min_edge.x - max_edge.x, 0f, min_edge.z - max_edge.z), out hit, distance)) {
        //         Max_x.Add(hit.point.x);
        //         Max_z.Add(hit.point.z);
        //     }
        //     if (Physics.Raycast(new Vector3(min_edge.x, min_edge.y, min_edge.z), new Vector3(max_edge.x - min_edge.x, 0f, max_edge.z - min_edge.z), out hit, distance)) {
        //         Min_x.Add(hit.point.x);
        //         Min_z.Add(hit.point.z);
        //     }

        //     this.xMax = Max_x.Count > 0 ? Max_x.Max() : bounds.max.x;
        //     this.xMin = Min_x.Count > 0 ? Min_x.Min() : bounds.min.x;
        //     this.zMax = Max_z.Count > 0 ? Max_z.Max() : bounds.max.z;
        //     this.zMin = Min_z.Count > 0 ? Min_z.Min() : bounds.min.z;

        //     if (MathF.Sqrt(((this.xMax - this.xMin) * (this.xMax - this.xMin)) + ((this.zMax - this.zMin) * (this.zMax - this.zMin))) > distance) {
        //         this.xMax = bounds.max.x;
        //         this.xMin = bounds.min.x;
        //         this.zMax = bounds.max.z;
        //         this.zMin = bounds.min.z;
        //     }
            

        //     this.CalculatePoints();

        //     AddVerticesToGlobalList();
        // }

        private void AddVerticesToGlobalList() {
        foreach (Point vertex in this.points) {
            VisibilityGraphManager.allVertices.Add(vertex);
        }
    }

        public void CalculateVisibleEdges(List<Obstacle> listObstacles) {
            for (int i = 0; i < this.points.Length; i++) {
                GetVisibilityPoints(this.points[i], listObstacles, this);
            }
        }

        private void CalculateMinMax() {
            this.xMax = this.xMin = this.points[0].x;
            this.zMax = this.zMin = this.points[0].z;
            for (int i = 1; i < this.points.Length; i++) {
                this.xMax = MathF.Max(this.xMax, this.points[i].x);
                this.xMin = MathF.Min(this.xMin, this.points[i].x);
                this.zMax = MathF.Max(this.zMax, this.points[i].z);
                this.zMin = MathF.Min(this.zMin, this.points[i].z);
            }
        }

        private void CalculatePoints() {
            this.points = new Point[4];

            this.points[0] = new Point(this.xMax, this.zMax);
            this.points[1] = new Point(this.xMax, this.zMin);
            this.points[2] = new Point(this.xMin, this.zMin);
            this.points[3] = new Point(this.xMin, this.zMax);
        }

        public void AddCarSpace(float carSpace) {
            this.points[0].x += carSpace;
            this.points[1].x += carSpace;
            this.points[2].x -= carSpace;
            this.points[3].x -= carSpace;

            this.points[0].z += carSpace;
            this.points[1].z -= carSpace;
            this.points[2].z -= carSpace;
            this.points[3].z += carSpace;

            this.xMax += carSpace;
            this.xMin -= carSpace;
            this.zMax += carSpace;
            this.zMin -= carSpace;
        }

        public List<Point> GetEdgesByPoint(Point a) {
            int i = Array.FindIndex(this.points, p => p == a);
            return this.GetEdgesByIndex(i);
        }

        public List<Point> GetEdgesByIndex(int i) {
            return new List<Point> {
                this.points[(i + 1) % 4],
                this.points[(i + 3) % 4]
            };
        }

        public void ShowObstacle(Color c) {
            for (int i = 0; i < this.points.Length; i++) {
                Debug.DrawLine(this.points[i].ToVector3(), this.points[(i+1) % this.points.Length].ToVector3(), c, 1000f);
            }
        }

        public bool Contains(Point point) {
            return point.x >= this.xMin && point.x <= this.xMax && point.z >= this.zMin && point.z <= this.zMax;
        }

        private static bool IsEdgeVisible(Point x, Point y, List<Obstacle> listObstacles, Obstacle obstacleFromX=null, Obstacle obstacleFromY=null, bool ignoreObstacles=false) {
            if (!ignoreObstacles && obstacleFromX != null && obstacleFromX.Contains(y)) {
                return false;
            }
            if (!ignoreObstacles && obstacleFromY != null && obstacleFromY.Contains(x)) {
                return false;
            }
            foreach (var obstacle in listObstacles) {
                if (obstacle != obstacleFromX && obstacle != obstacleFromY && (obstacle.Contains(x) || obstacle.Contains(y))) {
                    return false;
                }
                for (int j = 0; j < obstacle.points.Length; j++) {
                    if (Point.Intersect(x, y, obstacle.points[j], obstacle.points[(j + 1) % obstacle.points.Length])) {
                        return false;
                    }
                }
            }
            return true;
        }

        public void GetVisibilityPoints(int index, List<Obstacle> listObstacles) {
            Point x = this.points[index];

            foreach (var y in this.GetEdgesByIndex(index)) {
                if (IsEdgeVisible(x, y, listObstacles, this, this, true)) {
                    VisibilityGraphManager.AddEdge(x, y);
                }
            }

            foreach (var obstacle in listObstacles) {
                if (this != obstacle) {
                    int s_points = obstacle.points.Length;
                    for (int i = 0; i < s_points; i++) {
                        Point y = obstacle.points[i];
                        if (IsEdgeVisible(x, y, listObstacles, this, obstacle)) {
                            VisibilityGraphManager.AddEdge(x, y);
                        }
                    }
                }
            }
        }

        public static bool GetVisibilityPoints(Point x, List<Obstacle> listObstacles, Obstacle obstacleFromX=null) {
            bool added = false;
            if (obstacleFromX != null) {
                foreach (var y in obstacleFromX.GetEdgesByPoint(x)) {
                    if (IsEdgeVisible(x, y, listObstacles, obstacleFromX, obstacleFromX, true)) {
                        VisibilityGraphManager.AddEdge(x, y);
                        added = true;
                    }
                }
            }

            foreach (var obstacle in listObstacles) {
                if (obstacleFromX != obstacle) {
                    int s_points = obstacle.points.Length;
                    for (int i = 0; i < s_points; i++) {
                        Point y = obstacle.points[i];
                        if (IsEdgeVisible(x, y, listObstacles, obstacleFromX, obstacle)) {
                            VisibilityGraphManager.AddEdge(x, y);
                            added = true;
                        }
                    }
                }
            }
            return added;
        }

        public Point[] points;
        float xMin, xMax, zMin, zMax;
    }
}