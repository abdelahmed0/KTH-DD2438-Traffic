using Scripts.Map;

namespace PathPlanning
{
    using System.Collections;
    using System.Collections.Generic;
    using UnityEngine;
    using System;
    using System.Linq;
    using UnityEditor.XR.LegacyInputHelpers;
    using Random = UnityEngine.Random;
    using UnityEditor;
    using System.Runtime.CompilerServices;


    public class AStar
    {
        private ObstacleMap cMap;
        private float ht;
        private float scale = 4;
        private Vector2 InitialDirection;

        private Vector2 startPos;
        private Vector2 goalPos;
            
        [SerializeField] private float tolerance = 5f;
        private float toleranceScale;
        [SerializeField] private double maximumComputeTime = 10;

        [SerializeField] private HeuristicType heuristic = HeuristicType.Euclidean;

        [SerializeField] private float octileOrthogonalCost = 1f;
        [SerializeField] private float octileDiagonalCost = 100 * Mathf.Sqrt(2);

        [SerializeField] private bool allowSidewaysTurns = true;

        [SerializeField] private bool penalizeTurns = true;
        private int turnPenalty = 3;

        [SerializeField] private bool condensePath = false;
        [SerializeField] private bool interpolatePath = false;

        [SerializeField] private float interpolationFactor = 0.1f;

        [SerializeField] private float vMax = 40;
        [SerializeField] private float vTurn = 10;
        [SerializeField] private int lookAhead = 5;

        private ObstacleMapManager mapManager;

        private float[] velocities;
        

        enum Vehicle
        {
            Car,
            Drone
        }

        enum HeuristicType
        {
            Manhattan,
            Euclidean,
            Chebyshev,
            Octile,
            PenalizeTurns,
        }

        private Vehicle vehicleType;

        private bool drawPath;

        private Vector3 cellToWorld(Vector2 vec)
        {
            return new Vector3(vec.x, ht, vec.y);
        }
        
        public AStar(Vector3 startPos, Vector3 goalPos, ObstacleMapManager mapManager, Vector2 InitialDirection)
        {
            this.mapManager = mapManager;
            cMap = mapManager.ObstacleMap;
            this.InitialDirection = InitialDirection;
            ht = startPos.y;
            
            //System.Diagnostics.Debug.WriteLine("THIS IS A TEST");
            InitBannedDirections(allowSideways:allowSidewaysTurns);

            this.startPos = new Vector2(startPos.x, startPos.z);
            this.goalPos = new Vector2(goalPos.x, goalPos.z);

            // toleranceScale = cMap.GetCellSize().magnitude * Mathf.Sqrt(2) / 10f; // Scaled so that grid size of 5 gives given tolerance
            toleranceScale = 1;
        }

        public void ResetDebug() // used for GUI drawing
        {
            debugPath = null;
        }

        private float CalculateHeuristic(Vector2 curr, Vector2 goal, HeuristicType hType)
        {
            float res;
            switch (hType)
            {
                case HeuristicType.Manhattan:
                    res = Mathf.Abs(curr.x - goal.x) + Mathf.Abs(curr.y - goal.y);
                    break;

                case HeuristicType.Euclidean:
                    res = Mathf.Sqrt(Mathf.Pow(curr.x - goal.x, 2) + Mathf.Pow(curr.y - goal.y, 2));
                    break;
                    
                case HeuristicType.Chebyshev:
                    res = Mathf.Max(Mathf.Abs(curr.x - goal.x), Mathf.Abs(curr.y - goal.y));
                    break;
                    
                case HeuristicType.Octile:
                    // Tunable Parameters
                    float orthogonalCost = octileOrthogonalCost;
                    float diagonalCost = octileDiagonalCost;
                        
                    float dx = Mathf.Abs(curr.x - goal.x);
                    float dy = Mathf.Abs(curr.y - goal.y);
                    res = (orthogonalCost * (dx + dy)) + (diagonalCost - (2 * orthogonalCost)) * Mathf.Min(dx, dy);
                    break;
                case HeuristicType.PenalizeTurns:
                    res = (curr - goal).magnitude;
                    // maybe add a turn counter and append that to the cost
                    break;

                default:
                    throw new Exception("Heuristic Type Undefined");
            }

            return res;
        }

        public float[] GetVelocities()
        {
            if (velocities == null)
            {
                throw new Exception("Cannot get velocities unless A* is run");
            }

            return velocities;

        }

        private static Vector2[] directions = new[]
        {
            new Vector2(-1, 0), new Vector2(0, -1), new Vector2(1, 0), new Vector2(0, 1),
            new Vector2(-1, 1), new Vector2(-1, -1), new Vector2(1, -1), new Vector2(1, 1)
        };

        private static HashSet<Vector2>[] bannedDirections = new HashSet<Vector2>[8];

        private void InitBannedDirections(bool allowSideways=true)
        {
            for (int i = 0; i < 8; i++)
            {
                bannedDirections[i] = new HashSet<Vector2>();
            }
                
            bannedDirections[0].Add(directions[2]);
            bannedDirections[0].Add(directions[6]);
            bannedDirections[0].Add(directions[7]);
            
            bannedDirections[1].Add(directions[3]);
            bannedDirections[1].Add(directions[4]);
            bannedDirections[1].Add(directions[7]);
                
            bannedDirections[2].Add(directions[0]);
            bannedDirections[2].Add(directions[4]);
            bannedDirections[2].Add(directions[5]);
                
            bannedDirections[3].Add(directions[1]);
            bannedDirections[3].Add(directions[5]);
            bannedDirections[3].Add(directions[6]);
                
            bannedDirections[4].Add(directions[1]);
            bannedDirections[4].Add(directions[2]);
            bannedDirections[4].Add(directions[6]);
                
            bannedDirections[5].Add(directions[2]);
            bannedDirections[5].Add(directions[3]);
            bannedDirections[5].Add(directions[7]);
                
            bannedDirections[6].Add(directions[0]);
            bannedDirections[6].Add(directions[3]);
            bannedDirections[6].Add(directions[4]);
                
            bannedDirections[7].Add(directions[0]);
            bannedDirections[7].Add(directions[1]);
            bannedDirections[7].Add(directions[5]);

            if (!allowSidewaysTurns)
            {
                bannedDirections[0].Add(directions[1]);
                bannedDirections[0].Add(directions[3]);
                
                bannedDirections[1].Add(directions[0]);
                bannedDirections[1].Add(directions[2]);
                
                bannedDirections[2].Add(directions[1]);
                bannedDirections[2].Add(directions[3]);
                
                bannedDirections[3].Add(directions[0]);
                bannedDirections[3].Add(directions[2]);
                
                bannedDirections[4].Add(directions[5]);
                bannedDirections[4].Add(directions[7]);
                
                bannedDirections[5].Add(directions[4]);
                bannedDirections[5].Add(directions[6]);
                
                bannedDirections[6].Add(directions[5]);
                bannedDirections[6].Add(directions[7]);
                
                bannedDirections[7].Add(directions[4]);
                bannedDirections[7].Add(directions[6]);
                
            }
        }

        public Vector3[] FindPath(bool drawPath=true)
        {
            InitBannedDirections(allowSideways:allowSidewaysTurns);
                
            List<Vector2> cellPath = PerformAStar(startPos, goalPos);

            if (cellPath == null)
            {
                Debug.Log("No path found!");
                return new Vector3[0];
            }
            
            Vector3[] res = new Vector3[cellPath.Count];
            
            for (int i = 0; i < cellPath.Count; i++)
            {
                res[i] = cellToWorld(cellPath[i]);
            }
            
            return res;
        }

        private bool IsValidDirection(Vector2 dir, Vector2 prev)
        {
            float angle = Vector2.Angle(dir, prev);
            if (angle >= 90)
            {
                return false;
            }
            return true;
            // int r = -1;
            //     
            // for (int i = 0; i < 8; i++)
            // {
            //     Vector2 d = directions[i] * scale;
            //     if (VecEquals(prev, directions[i]))
            //     {
            //         r = i;
            //         break;
            //     }
            // }
            //
            // if (r == -1)
            // {
            //     throw new Exception("Valid Direction Not Found");
            // }
            //
            // return !bannedDirections[r].Contains(dir);

        }

        private bool VecEquals(Vector2 a, Vector2 b)
        {
            float epsilon = 0.0001f;
            Vector2 d = a - b;
            return (Mathf.Abs(d.x) < epsilon && Mathf.Abs(d.y) < epsilon);
        }
            
        private class PriorityQueue<T>
        {
            private List<(T, float)> elements = new List<(T, float)>();

            public void Enqueue(T item, float priority)
            {
                elements.Add((item, priority));
            }

            public T Dequeue()
            {
                int best_idx = 0;

                for (int i = 1; i < elements.Count; i++)
                {
                    if (elements[i].Item2 < elements[best_idx].Item2)
                    {
                        best_idx = i;
                    }
                }

                T best_item = elements[best_idx].Item1;
                elements.RemoveAt(best_idx);
                return best_item;
            }

            public int Count()
            {
                return elements.Count;
            }
        }

        private List<Vector2> PerformAStar(Vector2 start, Vector2 end)
        {
            PriorityQueue<Vector2> open_set = new PriorityQueue<Vector2>();
            Dictionary<Vector2, Vector2> prev = new Dictionary<Vector2, Vector2>();
            Dictionary<Vector2, float> g_score = new Dictionary<Vector2, float>();

            prev[start] = start - (InitialDirection * scale);
            prev[prev[start]] = prev[start] - (InitialDirection * scale);
            prev[prev[prev[start]]] = prev[prev[start]] - (InitialDirection * scale);
                
            open_set.Enqueue(start, 0);
            g_score[start] = 0;
            // Vector2 bounds = cMap.GetMapSize();

            DateTime start_time = DateTime.Now;
            while (open_set.Count() > 0 && (start_time - DateTime.Now).TotalSeconds < maximumComputeTime)
            {
                var curr = open_set.Dequeue();

                if (CalculateHeuristic(curr, end, HeuristicType.Euclidean) <= tolerance / toleranceScale)
                {
                    prev[end] = curr;
                    List<Vector2> path = ReconstructPath(prev, start, end);
                    if (condensePath)
                    {
                        List<Vector2> condensedPath = new List<Vector2>();
                        condensedPath.Add(start);
                        for (int a = 1; a < path.Count() - 1; a++)
                        {
                            if (!VecEquals(path[a] - path[a - 1], path[a + 1] - path[a]))
                            {
                                condensedPath.Add(path[a]);
                            }
                        }
                        condensedPath.Add(end);
                        path = condensedPath;
                    }

                    velocities = new float[path.Count()];

                    for (int i = 0; i < path.Count() - lookAhead; i++)
                    {
                        // print(i);
                        bool turn = false;
                        for (int j = 1; j < lookAhead && j < path.Count() && !turn; j++)
                        {
                            // print(i);
                            // Vector2 junk = path[j - 1];
                            // junk = prev[path[j - 1]];
                            // junk = path[j];
                            
                            
                            if (!VecEquals(path[i + j - 1] - prev[path[i + j - 1]], path[i + j] - path[i + j - 1]))
                            {
                                // print(path[i+j]);
                                // print("Turning");
                                turn = true;
                                // break;
                            }
                        }

                        if (turn)
                        {
                            velocities[i] = vTurn;
                        }
                        else
                        {
                            velocities[i] = vMax;
                        }
                    }

                    for (int i = path.Count() - lookAhead; i < path.Count(); i++)
                    {
                        velocities[i] = vMax;
                    }
                    
                    
                    
                    return path;
                }

                for (int i = 0; i < 8; i++)
                {
                    Vector2 new_pos = curr + (directions[i] * scale);

                    // if (!IsValidDirection(directions[i], curr - prev[curr]) || new_pos.x > bounds.x || new_pos.y > bounds.y || new_pos.x < 0 || new_pos.y < 0) // Checking that the dir is valid and not out of bounds
                    if (!IsValidDirection(directions[i], curr - prev[curr]) || new_pos.x < 0 || new_pos.y < 0)
                    { 
                        continue;
                    }

                    Vector3 worldPos = cellToWorld(new_pos);
                    Vector3 localPos = mapManager.grid.WorldToLocal(worldPos);
                    Vector2Int localCell = new Vector2Int(Mathf.RoundToInt(localPos.x), Mathf.RoundToInt(localPos.z));
                    ObstacleMap.Traversability trav = cMap.IsGlobalPointTraversable(worldPos);
                    if (trav == ObstacleMap.Traversability.Free ||
                        trav == ObstacleMap.Traversability.Partial)
                    // if (cMap.GetCellState(new_pos) == OurCollisionMap.State.Free)
                    {
                        float tent_g = g_score[curr] + (directions[i] * scale).magnitude;
                        
                        if (!VecEquals(curr - prev[curr], directions[i]) 
                            && !VecEquals(curr - prev[curr], prev[curr] - prev[prev[curr]])
                            && !VecEquals(prev[curr] - prev[prev[curr]], prev[prev[curr]] - prev[prev[prev[curr]]]))
                        {
                            tent_g += 5000000;
                            // continue;
                        }
                            

                        if (!g_score.ContainsKey(new_pos) || tent_g < g_score[new_pos])
                        {
                            g_score[new_pos] = tent_g;

                            float h = CalculateHeuristic(new_pos, end, heuristic);
                            float f = tent_g + h;

                            open_set.Enqueue(new_pos, f);
                            prev[new_pos] = curr;
                        }

                    }

                }

            }
                
            //System.Diagnostics.Debug.WriteLine("C");

            return null;
        }

        private List<Vector2> ReconstructPath(Dictionary<Vector2, Vector2> prev, Vector2 start, Vector2 end)
        {
            List<Vector2> path = new List<Vector2>();
            var curr = end;

            while (!(curr.x.Equals(start.x) && curr.y.Equals(start.y)))
            {
                path.Add(curr);
                curr = prev[curr];
            }

            path.Add(start);

            path.Reverse();
            return path;
        }

        private Vector3[] debugPath = null;
        private void OnDrawGizmos()
        {
            /// Used to draw the collision map
            if (drawPath)
            {

                if (debugPath.Length <= 0)
                    return;
                // Draw Path
                Vector3 prevPoint = debugPath[0];
                prevPoint.y += 1f;
                Vector3 currPoint;
                for (int i = 1; i < debugPath.Length; i++)
                {
                    currPoint = debugPath[i];
                    currPoint.y += 1f;
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(prevPoint, currPoint);
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(prevPoint, 0.5f);
                    prevPoint = currPoint;
                }
            }
        }
    }

}