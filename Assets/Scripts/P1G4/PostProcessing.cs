using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Graphs;

namespace PostProcessing {
    public class PathPartition
    {
        public float Value { get; set; } = float.PositiveInfinity;
        public int Parts { get; set; } = 0;
        public int End { get; set; } = -1;
    }

    public class PathSmoothing {

        public LayerMask obstacleLayer; // Set the obstacle layer in the Inspector

        public static float getSpace(Vector3 center, Vector3 direction, Bounds vehicleBound) {
            RaycastHit hit;
            if (Physics.BoxCast(center, vehicleBound.extents, direction, out hit)) {
                if (hit.distance < 5) {
                    Debug.DrawLine(center, hit.point, Color.red, 10000f);
                } else if (hit.distance < 10) {
                    Debug.DrawLine(center, hit.point, Color.yellow, 10000f);
                } else if (hit.distance < 15) {
                    Debug.DrawLine(center, hit.point, Color.white, 10000f);
                }
                
                return hit.distance;
            }
            return float.MaxValue;
        }

        public static List<float> biggerPathVel(List<int> path, List<Point> vertices, Bounds vehicleBound, float accel, List<float> val) {
            List<float> velocities = Enumerable.Repeat(0f, path.Count).ToList();
            velocities[path.Count - 1] = 150f;
            float distance_previous = 0f;
            float velocity_past = 0f;

            for (int i = path.Count - 2; i > 0; i--) {
                float velocity = 40f;
                Vector3 point = vertices[path[i]].ToVector3();
                point.y = vehicleBound.extents.y;
                Vector3 forward = vertices[path[i+1]].ToVector3() - vertices[path[i]].ToVector3();
                Vector3 backward = vertices[path[i]].ToVector3() - vertices[path[i-1]].ToVector3();
                float distance_past = backward.magnitude;
                float distance_future = forward.magnitude;
                forward = forward.normalized;
                backward = backward.normalized;
                float angle = 180 - Vector3.Angle(backward, forward);
                float distance_backward = getSpace(point, backward, vehicleBound);
                float distance_bf = getSpace(point, backward+forward, vehicleBound);

                // float time = (-velocity + MathF.Sqrt(velocity * velocity + 2 * accel * distance_past)) / accel;
                // velocity += accel * time;

                // if (angle < 150) velocity /= 2;

                bool low_dist = false;
                if (distance_backward < 15 && angle < 155) {
                    velocity = Math.Min(15f, velocity);
                }
                if ((distance_backward < 10 || distance_bf < 15) && angle < 155) {
                    velocity = Math.Min(10f, velocity);
                }
                if (angle < 130) {
                    velocity = Math.Min(15f, velocity);
                }
                if (distance_backward < 5 && angle < 155) {
                    velocity = Math.Min(1f, velocity);
                }
                if (angle < 145 && distance_future < 10 && distance_backward < 15) {
                    velocity = Math.Min(1f, velocity);
                }
                if (distance_backward < 5 && distance_bf < 5) {
                    velocity = Math.Min(5f, velocity);
                }
                if (angle < 145 && distance_future < 15) {
                    velocity = Math.Min(5f, velocity);
                }
                if (angle < 145 && distance_backward < 15) {
                     velocity = Math.Min(10f, velocity);
                }

                if (velocity_past > 0f) {
                    velocity = MathF.Min(velocity, velocity_past);
                }

                float timeToBreak = (velocity - 30f) / (-accel);
                float distanceToBreak = (30f * timeToBreak) + ((-accel) * timeToBreak * timeToBreak / 2) - distance_past;
                if (distance_backward < distanceToBreak){
                    distance_past = distance_backward;
                    velocity_past = velocity;
                } else {
                    distance_past = 0f;
                    velocity_past = 0f;
                }

                if (low_dist && distance_backward < 5) {
                    val[i] = 80;
                } else {
                    val[i] = 30;
                }

                float move_forward = 0f;
                // if (angle < 145 && distance_forward > 5) {
                //     move_forward = MathF.Min(2.5f, distance_forward/2);
                // }

                float move_backward = 0f;
                // if (distance_backward > 5 && distance_future < 5) {
                //     move_backward = MathF.Min(5f, distance_backward/2);
                // }

                velocities[i] = velocity;

                Vector3 result = point + (move_backward * backward) + (move_forward * -forward);

                
                vertices[path[i]] = new Point(result);

                
            }

            return velocities;
        }

        public static List<Point> smoothPath2(List<int> rawPath, List<Point> vertices, float smoothness, List<float> val) {
            List<Point> smoothedPath = new List<Point>();
            Vector3[] controlPoints = new Vector3[rawPath.Count];
            for (int i = 0; i < rawPath.Count; i++)
            {
                controlPoints[i] = vertices[rawPath[i]].ToVector3();
            }

            for (int i = 0; i < controlPoints.Length - 1; i++)
            {
                Vector3 p0 = i > 0 ? controlPoints[i - 1] : controlPoints[i];
                Vector3 p1 = controlPoints[i];
                Vector3 p2 = controlPoints[i + 1];
                Vector3 p3 = i + 2 < controlPoints.Length ? controlPoints[i + 2] : p2;

                // Adjust control points for tighter turns
                float tightnessFactor = CalculateTightnessFactor(p0, p1, p2);
                Vector3 adjustedP1 = AdjustControlPoint1(p1, p2, tightnessFactor);
                Vector3 adjustedP2 = AdjustControlPoint1(p2, p1, tightnessFactor);

                // Calculate distance between p1 and p2
                float distance = Vector3.Distance(p1, p2);

                // Determine tension based on distance (customize this function as needed)
                float tension = CalculateTensionBasedOnDistance(distance, val[i]);
                // tension = 0.6f

                smoothedPath.Add(vertices[rawPath[i]]);

                for (float t = 0 + smoothness; t < 1; t += smoothness)
                {
                    Vector3 catmullRomPos = CatmullRom(p0, p1, p2, p3, t);
                    Vector3 linearPos = Vector3.Lerp(p1, p2, t);
                    Vector3 position = InterpolatedPath(linearPos, catmullRomPos, tension);

                    smoothedPath.Add(new Point(position.x, position.z));
                }
            }

            

            return smoothedPath;
        }

        private static float CalculateTensionBasedOnDistance(float distance, float maxDistance) {
            // Lower tension value for longer distances, and higher for shorter distances
            return Mathf.Clamp01(1 - (distance / maxDistance)); // MaxDistance is a constant that you define based on your needs
        }

        private static float CalculateTightnessFactor(Vector3 p0, Vector3 p1, Vector3 p2)
        {
            // Calculate angle between vectors
            Vector3 v1 = p1 - p0;
            Vector3 v2 = p2 - p1;
            float angle = Vector3.Angle(v1, v2);

            // Adjust tightness based on angle (example logic)
            return angle > 90 ? 0.5f : 1f; // Tighter curve for sharp angles
        }

        private static Vector3 AdjustControlPoint1(Vector3 current, Vector3 next, float tightnessFactor)
        {
            Vector3 direction = (next - current).normalized;
            float distance = Vector3.Distance(current, next);

            return current + direction * distance * tightnessFactor;
        }

        public static List<Point> smoothPath1(List<int> rawPath, List<Point> vertices, float smoothness)
        {
            List<Point> smoothedPath = new List<Point>();
            for (int i = 0; i < rawPath.Count - 1; i++)
            {
                Point p0 = vertices[rawPath[Mathf.Max(i - 1, 0)]];
                Point p1 = vertices[rawPath[i]];
                Point p2 = vertices[rawPath[i + 1]];
                Point p3 = vertices[rawPath[Mathf.Min(i + 2, rawPath.Count - 1)]];

                for (float t = 0; t < 1; t += smoothness)
                {
                    Vector3 position = CatmullRom(p0.ToVector3(), p1.ToVector3(), p2.ToVector3(), p3.ToVector3(), t);

                    /* if (PredictCollision(position, p2.ToVector3()))
                    {
                        position = AdjustPositionAwayFromObstacles(position, rawPath, i, vertices);
                    } */

                    smoothedPath.Add(new Point(position.x, position.z));
                }
            }

            for (int i = 0; i < smoothedPath.Count-1; i++) {

                drawSquare(smoothedPath[i], Color.magenta);

                Debug.DrawLine(smoothedPath[i].ToVector3(), smoothedPath[i+1].ToVector3(), Color.magenta, 20000f);
            }

            return smoothedPath;
        }

        private static Vector3 InterpolatedPath(Vector3 linear, Vector3 catmullRom, float tension)
        {
            // Ensure tension is within the range [0, 1]
            tension = Mathf.Clamp01(tension);

            // Interpolate between the linear path and the Catmull-Rom path
            return Vector3.Lerp(catmullRom, linear, tension);
        }

        private static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            // Catmull-Rom spline formula
            Vector3 a = 0.5f * (2f * p1);
            Vector3 b = 0.5f * (p2 - p0);
            Vector3 c = 0.5f * (2f * p0 - 5f * p1 + 4f * p2 - p3);
            Vector3 d = 0.5f * (-p0 + 3f * p1 - 3f * p2 + p3);

            Vector3 pos = a + b * t + c * t * t + d * t * t * t;

            return pos;
        }

        private static bool PredictCollision(Vector3 currentPosition, Vector3 nextPosition)
        {
            Vector3 direction = (nextPosition - currentPosition).normalized;
            RaycastHit hit;

            if (Physics.Raycast(currentPosition, direction, out hit, 30f))
            {
                Debug.Log($"At position ({currentPosition.x}, {currentPosition.z}) we detected a future collision");
                // Collision predicted with an obstacle
                return true;
            }

            return false;
        }
        
        private static Vector3 AdjustPositionAwayFromObstacles(Vector3 position, List<int> rawPath, int currentIndex, List<Point> vertices)
        {
            if (currentIndex <= 0 || currentIndex >= rawPath.Count - 1) {
                // If the current index is out of range, return the position as is.
                return position;
            }

            Vector3 pointA = vertices[rawPath[currentIndex - 1]].ToVector3();
            Vector3 pointB = vertices[rawPath[currentIndex]].ToVector3();

            Vector3 edgeDirection = (pointB - pointA).normalized;
            float adjustmentMagnitude = 2.0f; // Adjust this value as needed
            Vector3 adjustedPosition = position - edgeDirection * adjustmentMagnitude;

            return adjustedPosition;
        }
        public static Point CreatePointPk(Point p1, Point p2, Point p3, float distanceFromP2){
            // Create vectors from points
            Point v1 = p1 - p2;
            Point v2 = p3 - p2;

            // Normalize the vectors
            Point dir1 = v1.Normalize();
            Point dir2 = v2.Normalize();

            // Find the average direction (bisector)
            Point bisector = (dir1 + dir2).Normalize();

            // Calculate the position of Pk by moving from P2 along the bisector
            Point pk = p2 - bisector * distanceFromP2;

            return pk;
        }

        public static void drawSquare(Point p, Color color){
            Point corner1 = new Point();
            Point corner2 = new Point();
            Point corner3 = new Point();
            Point corner4 = new Point();
            
            corner1.x = p.x - 1;
            corner1.z = p.z - 1;

            corner2.x = p.x - 1;
            corner2.z = p.z + 1;

            corner3.x = p.x + 1;
            corner3.z = p.z - 1;

            corner4.x = p.x + 1;
            corner4.z = p.z + 1;

            Debug.DrawLine(corner1.ToVector3(), corner2.ToVector3(), color, 20000f);
            Debug.DrawLine(corner3.ToVector3(), corner1.ToVector3(), color, 20000f);
            Debug.DrawLine(corner2.ToVector3(), corner4.ToVector3(), color, 20000f);
            Debug.DrawLine(corner3.ToVector3(), corner4.ToVector3(), color, 20000f);
        }
    }
}