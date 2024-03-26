using System;
using UnityEngine;
using Graphs;
using System.Runtime.CompilerServices;
using System.Collections.Generic;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using System.Linq;

namespace PD {
    class PDDrone {
        public static List<int> path;
        public static List<Point> vertices;

        public static List<float> velocities;

        private static int index = 1;

        public static float HeuristicsAStar(List<Point> vertices, int v1, int v2, int v3) {
            if (v1 == -1 || v2 == -1 || v3 == -1) return 0;
            float angle = Vector3.Angle(vertices[v2].ToVector3() - vertices[v1].ToVector3(), vertices[v3].ToVector3() - vertices[v2].ToVector3());
            return 7.5f * angle / 90;
        }

        public static Tuple<float, float> Drive(Vector3 position, Vector3 velocity, float maxAcc) {
            float k_v = 0.75f, k_p = 1f;
            Vector3 targetPosition = vertices[path[index]].ToVector3();
            Vector3 distance_v = targetPosition - position;
            float distance = MathF.Sqrt((distance_v.x * distance_v.x) + (distance_v.z * distance_v.z));
            float timeToTarget = distance / velocity.magnitude;
            Vector3 targetVelocity = Vector3.zero;

            float angle;
            if (index + 1 < path.Count) {
                angle = 180 - Vector3.Angle(targetPosition - position, vertices[path[index + 1]].ToVector3() - targetPosition);
            } else {
                angle = 180;
            }

            Debug.Log(angle);

            // Improve this by the velocity needed by the angle
            float velocityToBreak = 7.5f * angle / 150;
            float timeToBreak = (velocityToBreak - velocity.magnitude) / (-maxAcc);
            float distanceToBreak = (velocity.magnitude * timeToBreak) + ((-maxAcc) * timeToBreak * timeToBreak / 2);
            float distanceLeft = distance - distanceToBreak;

            // Debug.Log(distanceLeft);
            
            if (distance <= 3) {
                index = Math.Min(index + 1, path.Count - 1);
            } 
            if (angle < 150 && distanceLeft < 1f && distanceLeft > -1f) {
                k_p = 0f;
                k_v = 1f;
            }

            Vector3 position_error = (targetPosition - position).normalized;
            Vector3 velocity_error = (targetVelocity - velocity).normalized;
            Vector3 desired_acceleration = (k_p * position_error + k_v * velocity_error) * 15;
            
            return new Tuple<float, float>(desired_acceleration.x, desired_acceleration.z);
        }

        public static void GetDistanceNeededBreak(float acc, float v, float d) {

        }

        public static void DrawSolution() {
            for (int i = 1; i < path.Count; i++) {
                Debug.DrawLine(vertices[path[i-1]].ToVector3(), vertices[path[i]].ToVector3(), Color.cyan, 10000f);
            }
        } 
    }

    class PDCar {
        public static List<int> path;

        public static List<Point> vertices;

        public static List<Point> smoothedPath;

        public static List<float> velocities;


        private static int index = 0;

        private static int index_v = 0;

        private static float distance_before = float.MaxValue;

        public (float k_v, float k_p) tightness = AnalyzeVisibilityGraph();

        public static float HeuristicsAStar(List<Point> vertices, int v1, int v2, int v3) {
            if (v1 == -1 || v2 == -1 || v3 == -1) return 0;
            float angle = Vector3.Angle(vertices[v2].ToVector3() - vertices[v1].ToVector3(), vertices[v3].ToVector3() - vertices[v2].ToVector3());
            if (angle > 100) return 1000f;
            return 30f * (angle / 90);
        }

        public static void GetVelocities(List<Point> vertices) {
            velocities = Enumerable.Repeat(-1f, path.Count).ToList();
            float angle_past = 180;
            float distance_past = 0f;
            for (int i = smoothedPath.Count - 2; i > 0; i--) {
                float angle = 180 - Vector3.Angle(vertices[path[i]].ToVector3() - vertices[path[i]].ToVector3(), vertices[path[i]].ToVector3() - vertices[path[i]].ToVector3());
                float distance = 0;
                if (angle >= 150) {
                    distance += distance_past;
                }
                angle = MathF.Min(angle, angle_past);
                if (angle < 150) {
                    distance += Vector3.Distance(smoothedPath[i].ToVector3(), smoothedPath[i-1].ToVector3());
                    float velocityToBreak = 7.5f * angle / 150;
                    if (angle > 130) velocityToBreak += 7.5f;
                    float maxAcc = 8f;
                    float timeToBreak = (velocityToBreak - 22.5f) / (-maxAcc);
                    float distanceToBreak = (22.5f * timeToBreak) + ((-maxAcc) * timeToBreak * timeToBreak / 2);
                    float distanceLeft = distance - distanceToBreak;
                    if (distanceLeft > 0) {
                        angle_past = 180;
                        distance_past = 0f;
                    } else {
                        angle_past = angle;
                        distance_past += distance;
                    }
                    velocities[i] = velocityToBreak;
                }
                
            }
        }

        public static void Drive(Rigidbody rigidbody, CarController m_Car) {
            Vector3 position = rigidbody.position;
            Vector3 velocity = rigidbody.velocity;

            (float k_v, float k_p) tightness = AnalyzeVisibilityGraph();
            Debug.Log($"k_v is {tightness.k_v}, k_p is {tightness.k_p}");

            float k_v = tightness.k_v;
            float k_p = 1.0f;

            // float k_v = 0.6f, k_p = 1.0f;
            Vector3 targetPosition = smoothedPath[index].ToVector3();
            Debug.Log("L: " + index_v);
            
            if (index_v == path.Count - 1) {
                targetPosition = vertices[path[index_v]].ToVector3();
                Debug.Log("LAST");
            }
            Vector3 distance_v = targetPosition - position;
            float distance = MathF.Sqrt((distance_v.x * distance_v.x) + (distance_v.z * distance_v.z));
            Vector3 distance_v2 = vertices[path[index_v]].ToVector3() - position;
            float distance2 = MathF.Sqrt((distance_v2.x * distance_v2.x) + (distance_v2.z * distance_v2.z));

            // float angle;
            // if (index + 1 < smoothedPath.Count) {
            //     angle = 180 - Vector3.Angle(targetPosition - position, smoothedPath[index + 1].ToVector3() - targetPosition);
            // } else {
            //     angle = 180;
            // }

            // Improve this by the velocity needed by the angle
            float velocityToBreak = velocities[index_v];
            float timeToBreak = (velocityToBreak - velocity.magnitude) / (-8f);
            float distanceToBreak = (velocity.magnitude * timeToBreak) + ((-8f) * timeToBreak * timeToBreak / 2);
            float distanceLeft = distance2 - distanceToBreak;

            Vector3 targetVelocity;
            if (distanceLeft < 2) {
                targetVelocity = (smoothedPath[index].ToVector3()- smoothedPath[Math.Max(0, index-1)].ToVector3()).normalized * velocities[index_v]; 
            } else {
                targetVelocity = (smoothedPath[index].ToVector3()- smoothedPath[Math.Max(0, index-1)].ToVector3()).normalized * 50;
            }


            Debug.Log(" Velocity: " + rigidbody.velocity.magnitude + " T: " + velocityToBreak);

            // if (rigidbody.velocity.magnitude < 10) {
            //     k_v = 0.5f;
            // }

            if (distance <= 5f) {
                index = Math.Min(index + 1, smoothedPath.Count - 1);
            }
            if (distance2 <= 6f) {
                Point p = vertices[path[index_v]];
                index = Math.Max(smoothedPath.FindIndex(p2 => p2.x == p.x && p2.z == p.z) + 1, index);
                index = Math.Min(index, smoothedPath.Count - 1);
                index_v = Math.Min(index_v + 1, path.Count - 1);
            }
            distance_before = distance;
                
            Vector3 position_error = (targetPosition - position);
            Vector3 velocity_error = (targetVelocity - velocity);
            Vector3 desired_acceleration = (k_p * position_error + k_v * velocity_error) * 15;
            
            float steering = Vector3.Dot(desired_acceleration, m_Car.transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, m_Car.transform.forward);

            Debug.Log("Steering: " + steering + " Acc: " + acceleration);

            if (acceleration < 0 && velocity.magnitude - velocities[index_v] > 5) {
                m_Car.Move(steering, 0f, acceleration, -acceleration);
            } else if (acceleration < 0) {
                m_Car.Move(steering, 0f, acceleration, 0f);
            } 
            else {
                m_Car.Move(steering, acceleration, 0f, 0f);
            }
            
        }

        public static float GetDistanceNeededBreak(Rigidbody rigidbody, float velocityTarget=0f) {
            float torque = 40000f;
            if (velocityTarget > rigidbody.velocity.magnitude) return 0f;
            float vel_diff = (rigidbody.velocity.magnitude * rigidbody.velocity.magnitude) - (velocityTarget * velocityTarget);
            return rigidbody.inertiaTensor.magnitude * vel_diff / torque;
        }

        public static void DrawSolution() {
            for (int i = 1; i < smoothedPath.Count; i++) {
                Debug.DrawLine(smoothedPath[i-1].ToVector3(), smoothedPath[i].ToVector3(), Color.cyan, 10000f);
            }
        }
        
        
        public static (float k_v, float k_p) AnalyzeVisibilityGraph()
        {
            List<Point> vertices = Graphs.VisibilityGraphManager.allVertices;
            List<Tuple<Point, Point>> edges = Graphs.VisibilityGraphManager.visibleEdges;
            // Analyze edge lengths
            float totalEdgeLength = 0;
            float maxEdgeLength = 0;
            foreach (var edge in edges)
            {
                float length = Vector3.Distance(edge.Item1.ToVector3(), edge.Item2.ToVector3());
                totalEdgeLength += length;
                if (length > maxEdgeLength) maxEdgeLength = length;
            }
            float averageEdgeLength = totalEdgeLength / edges.Count;
            // Calculate graph density
            float density = (float)edges.Count / vertices.Count;
            float k_v = MapValue(density, 0, 3f /* adjust based on expected max density */, 0.1f, 0.9f);
            float normalized = (k_v - 0.1f) / 0.95f;
            float scaled = Mathf.Pow(normalized, 1.55f);
            k_v = 1 - 0.1f - scaled * 1.0f;
            float k_p = 0.9f;
            return (k_v, k_p);
        }
        private static float MapValue(float value, float fromSource, float toSource, float fromTarget, float toTarget)
        {
            return Mathf.Clamp((value - fromSource) / (toSource - fromSource) * (toTarget - fromTarget) + fromTarget, fromTarget, toTarget);
        }
    }
}