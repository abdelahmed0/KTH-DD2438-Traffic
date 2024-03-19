﻿using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;
using System;
    
using System.Collections.Generic;

using aStar;
using UnityEngine.Serialization;
using PathPlanning;
using Search;
using PD;
using Graphs;
using PostProcessing;
using UnityEngine.UIElements;

[RequireComponent(typeof(CarController))]
public class AIP3FormationCar : MonoBehaviour
{
    private PathType pathType = PathType.lineOfSight;
    public float colliderResizeFactor = 2f;
    public int numberSteeringAngles = 5;
    public bool allowReversing = false;   
    public bool smoothPath = false;     
    private Vector3 targetVelocity;
    private float k_p = 2;
    private float k_d = 3;
    private float gatePaddingMagnitude = 3f;
    public float followThrough = 1.5f;
    
    private CarController m_Car; 
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    
    private BoxCollider m_Collider;
    private static CollisionDetector m_Detector = null;

    private int goalIdx = 0;

    private static bool StaticInitDone = false;

    private List<AStarNode> nodePath = new();
    private int currentNodeIdx;

    private Rigidbody my_rigidbody;

    private Vector3[,] pathSet;
    private int currPathIdx = 1;

    private List<Vector3> path = new List<Vector3>();
    private float pointFrequency = 4;

    private bool isLeader = false;
    private static Vector3 leaderPosition;

    private Vector3 relativePosition;

    private int leaderIdx = 4;

    private List<Vector3> gatesList = new List<Vector3>();
    private static List<Vector3> leaderGatesList = new List<Vector3>();
    private int gateIdx = 0;

    private static bool[] update;

    private float ghostLeader = 8f;

    private Vector3 tgt = new Vector3(0, 0, 0);
    
    private Vector3 height = new Vector3(0, 1, 0);

    private Vector3 ghostPos = new Vector3(0, 0, 0);

    private float velocity = 5f;

    private List<Vector3> astarPath;

    private static float[] carDistances;
    private static bool[] isSlowing;

    private List<Vector3> rrtPath;

    private float carLength;

    private static bool staticAstarInitDone = false;
    private static List<Obstacle> listObstacles;

    private static int[] gateIdxs;
    
    enum PathType
    {
        astar,
        lineOfSight,
        hybridAstar
    }
    
    
    private void Start()
    {
        m_Car = GetComponent<CarController>();
        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap; //Is not a MonoBehavior, so cannot fetch in the same fashion!
        
        // print(m_ObstacleMap);

        m_OtherCars = GameObject.FindGameObjectsWithTag("Player");
        
        // print("Num other cars");
        // print(m_OtherCars.Length);
        
        // Figure out which car we are
        int idx = -1;
        for (int i = 0; i < m_OtherCars.Length; i++)
        {
            if (m_OtherCars[i].transform.position == transform.position)
            {
                idx = i;
            }
        }
        // print("I am car " + idx + " at " + transform.position);

        int left = 0;
        int right = 0;
        for (int i = 0; i < m_OtherCars.Length; i++)
        {
            if (i == idx)
                continue;

            Vector2 a = new Vector2(transform.position.x, transform.position.z);
            Vector2 b = new Vector2(m_OtherCars[i].transform.position.x, m_OtherCars[i].transform.position.z);

            float angle = Vector2.SignedAngle(a, b);
            // print(angle);

            // if (angle >= -90 && angle <= 90)
            // {
            //     right++;
            // }
            // else
            // {
            //     left++;
            // }

            if (transform.position.x > m_OtherCars[i].transform.position.x)
            {
                left++;
            }
            else
            {
                right++;
            }

        }

        goalIdx = left;
        
        m_Collider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();

        my_rigidbody = GetComponent<Rigidbody>();
        
        // For finding individual gates
        var gateGroup = m_MapManager.GetTargetObjects();
        // gateGroup = GameObject.FindGameObjectsWithTag("Target").ToList();
        
        var individualGates = GameObject.FindGameObjectsWithTag("SubTarget").ToList(); // Gives all individual gates on map... but this might not be so useful

        int numGates = 0;
        
        foreach (var group in gateGroup)
        {
            foreach (Transform child in group.transform) // I know this looks odd, but Transform implements Enumerable. When you iterate it, you iterate it's children.
            {
                // Should iterate over child gates left to right. 
                // 
                numGates++;
            }

            break;
        }

        update = new bool[numGates];
        for (int i = 0; i < update.Length; i++)
        {
            update[i] = false;
        }

        carDistances = new float[numGates];
        for (int i = 0; i < carDistances.Length; i++)
        {
            carDistances[i] = 10000;
        }

        isSlowing = new bool[numGates];
        for (int i = 0; i < isSlowing.Length; i++)
        {
            isSlowing[i] = false;
        }

        gateIdxs = new int[numGates];
        for (int i = 0; i < gateIdxs.Length; i++)
        {
            gateIdxs[i] = 0;
        }
        
        Debug.Log("HELLOOOOOOOOOOOOOO");

        Transform[] gates = new Transform[numGates];

        int ctr = 0;

        Debug.Log(numGates);
        Debug.Log(gates.Length);
        Debug.Log(gateGroup[0]);

        foreach (Transform child in gateGroup[0].transform)
        {
            gates[ctr] = child;
            ctr++;
        }

        // // Get target gate for current car based on distance to gate
        // int curr = 0;
        // float val = Vector3.Distance(transform.position, gates[0].position);
        // for (int i = 1; i < gates.Length; i++)
        // {
        //     float temp = Vector3.Distance(transform.position, gates[i].position);
        //     if (temp < val)
        //     {
        //         curr = i;
        //         val = temp;
        //     }
        // }
        // goalIdx = curr;
        
        // Set middle car as leader
        if (goalIdx == leaderIdx)
        {
            isLeader = true;
            leaderPosition = transform.position;
        }
        
        // Create array of paths between gates
        pathSet = new Vector3[gateGroup.Count, 2];
        pathSet[0, 0] = transform.position;
        pathSet[0, 1] = gateGroup[0].transform.GetChild(goalIdx).position;
        
        // Move target vector to center of gate
        Quaternion rotation = gateGroup[0].transform.rotation * Quaternion.Euler(0, 180f, 0);
        Vector3 padding = new Vector3(gatePaddingMagnitude, 0, 0);
        padding = rotation * padding;
        Debug.DrawLine(pathSet[0, 1], pathSet[0, 1] + padding, Color.magenta, 100f);
        pathSet[0, 1] += padding;

        if (!isLeader)
        {
            relativePosition = (gateGroup[0].transform.GetChild(goalIdx).position + padding) -
                               (gateGroup[0].transform.GetChild(leaderIdx).position + padding);
        }
            
        // Calculate paths between gates and store them
        for (int i = 1; i < gateGroup.Count; i++)
        {
            pathSet[i, 0] = pathSet[i - 1, 1];
            
            // // TODO: Temporary Fix
            // if (i >= 6)
            // {
            //     pathSet[i, 1] = gateGroup[i].transform.GetChild(Mathf.Abs(goalIdx - 4)).position;
            // }
            // else
            // {
            pathSet[i, 1] = gateGroup[i].transform.GetChild(goalIdx).position;
            // }
            
            // Adjust vector to point to center of gate
            rotation = gateGroup[i].transform.rotation * Quaternion.Euler(0, 180f, 0);
            padding = new Vector3(gatePaddingMagnitude, 0, 0);
            padding = rotation * padding;
            pathSet[i, 1] += padding;
        }
        
        if (pathType == PathType.lineOfSight)
        {
            float rotAngle = 90;
            for (int i = 0; i < gateGroup.Count; i++)
            {
                Quaternion rot = gateGroup[i].transform.rotation * Quaternion.Euler(0, 90, 0);
                Vector3 pad = new Vector3(4, 0, 0);
                pathSet[i, 1] = pathSet[i, 1] + (rot * pad);
            }
        }

        // Create path and add points every pointFrequency steps along line
        path.Add(pathSet[0,0]);
        bool cont = true;
        Vector3 cur = pathSet[0, 0];
        int id = 0;
        while (id < gateGroup.Count)
        {
            Vector3 dir = Vector3.Normalize(pathSet[id, 1] - cur);
            
            if (Vector3.Distance(pathSet[id, 1], cur) <= Vector3.Distance(cur, cur + (dir * pointFrequency)))
            {
                cur = pathSet[id, 1];
                id++;
                path.Add(cur);
                continue;
            }
            
            cur += (dir * pointFrequency);
            path.Add(cur);
        }
        
        // Store gates in gatesList
        for (int i = 0; i < gateGroup.Count; i++)
        {
            gatesList.Add(pathSet[i, 1]);
        }

        if (isLeader)
        {
            leaderGatesList = gatesList;
        }
        
        
        if (!StaticInitDone)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            // Rescale grid to have square shaped grid cells with size proportional to the car length
            float gridCellSize = m_Collider.transform.localScale.z;
            Vector3 gridScale = m_ObstacleMap.mapGrid.transform.localScale;
        
            m_ObstacleMapManager.grid.cellSize = new Vector3(
                gridCellSize / gridScale.x,
                gridCellSize / gridScale.z,
                gridCellSize / gridScale.y);
            m_MapManager.Initialize();
            m_ObstacleMapManager.Initialize();
            m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;
            StaticInitDone = true;
            Debug.Log($"Grid rescaling: {sw.ElapsedMilliseconds} ms");
        
            sw.Restart();
            m_Detector = new CollisionDetector(m_ObstacleMap, margin: colliderResizeFactor * m_Collider.transform.localScale.x);
            Debug.Log($"Detector init: {sw.ElapsedMilliseconds} ms");
        
            StaticInitDone = true;
            sw.Stop();
        }

        if (pathType == PathType.hybridAstar)
        {
            // Generate path with Hybrid A*
            currentNodeIdx = 0;
            HybridAStarGenerator pathFinder = new HybridAStarGenerator(m_ObstacleMap.mapGrid, m_ObstacleMap, m_Car.m_MaximumSteerAngle, m_Collider,
                colliderResizeFactor, false, allowReversing, 2f);
        
            var localStart = m_ObstacleMap.mapGrid.WorldToLocal(pathSet[0, 0]);
            var localGoal = m_ObstacleMap.mapGrid.WorldToLocal(pathSet[0, 1]);
        
            nodePath = pathFinder.GeneratePath(
                new Vector3(localStart.x, 0.05f, localStart.z),
                new Vector3(localGoal.x, 0.05f, localGoal.z),
                transform.eulerAngles.y,
                numberSteeringAngles);
        
            for (int i = 1; i < gateGroup.Count; i++)
            {
                localStart = m_ObstacleMap.mapGrid.WorldToLocal(pathSet[i, 0]);
                localGoal = m_ObstacleMap.mapGrid.WorldToLocal(pathSet[i, 1]);
            
                // Pad start so it follows through the gate
                Vector3 prevDir = nodePath[nodePath.Count - 1].LocalPosition - nodePath[nodePath.Count - 2].LocalPosition;
                prevDir = Vector3.Normalize(prevDir) * followThrough;
                localStart += prevDir;
        
                List<AStarNode> tempList = pathFinder.GeneratePath(
                    new Vector3(localStart.x, 0.05f, localStart.z),
                    new Vector3(localGoal.x, 0.05f, localGoal.z),
                    transform.eulerAngles.y,
                    numberSteeringAngles);
                // tempList.RemoveAt(0);
                nodePath.AddRange(tempList);
            }
        
            nodePath = smoothPath ? pathFinder.SmoothPath(nodePath) : nodePath;
        
            Vector3 old_wp = m_ObstacleMap.mapGrid.WorldToLocal(pathSet[0, 0]);
            foreach (var wp in nodePath)
            {
                Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(old_wp), m_ObstacleMap.mapGrid.LocalToWorld(wp.LocalPosition), Color.blue, 100f);
                old_wp = wp.LocalPosition;
            }
        }
        
        // Make A* Path
        if (pathType == PathType.astar)
        {
            float rotAngle = 90;
            astarPath = new List<Vector3>();
            Quaternion testRot = gateGroup[0].transform.rotation * Quaternion.Euler(0, 90, 0);
            Vector3 pad = new Vector3(0, 0, 0);
            Vector3 testEnd = pathSet[0, 1] + (testRot * pad);
            AStarr astar = new AStarr(pathSet[0, 0], testEnd, m_ObstacleMapManager, GetInitialDirection());
            astarPath.AddRange(astar.FindPath());
            
            for (int i = 1; i < gateGroup.Count; i++)
            {
                Vector3 currStart = pathSet[i, 0];
                Vector3 currGoal = pathSet[i, 1];
                Vector3 idee = astarPath[astarPath.Count - 1] - astarPath[astarPath.Count - 2];
                Vector2 initDir = new Vector2(idee.x, idee.z);
                
                Quaternion rot = gateGroup[i].transform.rotation * Quaternion.Euler(0, rotAngle, 0);
                Vector3 realGoal = currGoal + (rot * pad);
                
                if (i == gateGroup.Count - 1)
                {
                    realGoal = currGoal;
                }
            
                AStarr currAstar = new AStarr(currStart, realGoal, m_ObstacleMapManager, initDir);
                astarPath.AddRange(currAstar.FindPath());
            }

            // if (!staticAstarInitDone)
            // {
            //     var carBounds = transform.Find("SkyCar").transform.Find("SkyCarBody").GetComponent<Renderer>().bounds;
            //     var cSpaceSize = 0.4f + ((carBounds.size.x + carBounds.size.z) / 3);
            //     var high_Max = 0.1f + carBounds.extents.y * 2;
            //     var high_Min = 0.1f;
            //     carLength = MathF.Max(carBounds.extents.x, carBounds.extents.z) * 2;
            //     
            //     listObstacles = new List<Obstacle>();
            //
            //     foreach (var gameObject in m_ObstacleMap.obstacleObjects) {
            //
            //         var objectCollider = gameObject.GetComponent<Collider>();
            //         
            //         if (gameObject.name.Equals("targetGate"))
            //         {
            //             Vector3 p = new Vector3(3, 0, 0);
            //             Quaternion r = objectCollider.transform.rotation * Quaternion.Euler(0, 180f, 0);
            //             Vector3 center = objectCollider.transform.position + (r * p);
            //             Bounds b = new Bounds(center, new Vector3(1f, 1f, 1f));
            //             Obstacle obstacle = new Obstacle(objectCollider, high_Max, high_Min, gate: true, testBounds: b);
            //             obstacle.ShowObstacle(Color.magenta);
            //             listObstacles.Add(obstacle);
            //         }
            //
            //         else if (gameObject.name.Contains("startOverhead")) {
            //
            //             Obstacle obstacleRight = new Obstacle(objectCollider, true, high_Max);
            //             // obstacleRight.ShowObstacle(Color.magenta);
            //             obstacleRight.AddCarSpace(cSpaceSize);
            //             obstacleRight.ShowObstacle(Color.red);
            //             listObstacles.Add(obstacleRight);
            //
            //             Obstacle obstacleLeft = new Obstacle(objectCollider, false, high_Max);
            //             // obstacleLeft.ShowObstacle(Color.magenta);
            //             obstacleLeft.AddCarSpace(cSpaceSize);
            //             obstacleLeft.ShowObstacle(Color.red);
            //             listObstacles.Add(obstacleLeft);
            //             
            //         } else if (objectCollider != null && !gameObject.name.Contains("goalFlag")) {
            //
            //             Obstacle obstacle = new Obstacle(objectCollider, high_Max, high_Min);
            //             obstacle.ShowObstacle(Color.magenta);
            //             obstacle.AddCarSpace(cSpaceSize);
            //             obstacle.ShowObstacle(Color.red);
            //             listObstacles.Add(obstacle);
            //         }
            //     }
            //     
            //     foreach (var obstacle in listObstacles) {
            //         obstacle.CalculateVisibleEdges(listObstacles);
            //     }
            //
            //     staticAstarInitDone = true;
            // }

            // PDCar.path = new List<int>();

            // List<Vector3> points = new List<Vector3>();
            //
            // for (int i = 0; i < gateGroup.Count ; i++)
            // {
            //     Point start = new Point(pathSet[i, 0]);
            //     Point before_start = new Point(pathSet[i, 0] - m_Car.transform.forward);
            //     Point end = new Point(pathSet[i, 1]);
            //
            //     List<Vector3> moves = new List<Vector3>() {
            //         Vector3.back, Vector3.forward, Vector3.left, Vector3.right
            //     };
            //
            //     int tries = 0;
            //     
            //     while (!Obstacle.GetVisibilityPoints(start, listObstacles)) {
            //         start = new Point(pathSet[i, 0] + ((tries / 4) * moves[tries % 4]));
            //         Debug.Log(start);
            //         tries++;
            //     }
            //     tries = 0;
            //     while (!Obstacle.GetVisibilityPoints(end, listObstacles)) {
            //         end = new Point(pathSet[i, 1] + ((tries / 4) * moves[tries % 4]));
            //         tries++;
            //     }
            //
            //     // List<Point> currVertices = VisibilityGraphManager.allVertices;
            //     // List<Tuple<Point, Point>> currEdges = VisibilityGraphManager.visibleEdges;
            //     
            //     List<Point> vertices = Graphs.VisibilityGraphManager.allVertices;
            //     List<Tuple<Point, Point>> edges = Graphs.VisibilityGraphManager.visibleEdges;
            //
            //     vertices.Insert(0, before_start);
            //     vertices.Insert(1, start);
            //     vertices.Add(end);
            //
            //     AStar AStarAlgorithm  = new AStar();
            //     
            //     PDCar.path = AStarAlgorithm.regularA(1, vertices.Count - 1, vertices, edges, PDCar.HeuristicsAStar, 0);
            //
            //     if (goalIdx == 2)
            //     {
            //         for (int j = 1; j < PDCar.path.Count; j++) {
            //             Debug.DrawLine(vertices[PDCar.path[j-1]].ToVector3(), vertices[PDCar.path[j]].ToVector3(), Color.green, 10000f);
            //         }
            //         
            //     }
            // }

            // PDCar.path = AStarAlgorithm.regularA(1, vertices.Count - 1, vertices, edges, PDCar.HeuristicsAStar, 0);
            
            
            
        }

        // RRT rrt = new RRT();
        // // List<float> temp;
        // List<Vector2> rrtOut;
        // rrtPath = new List<Vector3>();
        // (rrtOut, _) = rrt.FindPath(pathSet[0, 0], pathSet[0, 1], m_Detector);
        // foreach (var pt in rrtOut)
        // {
        //     rrtPath.Add(new Vector3(pt.x, pathSet[0, 0].y, pt.y));
        // }






        // Debug.Log("Finished Car");


        // See AIP1TrafficCar.cs for other examples
    }


    private void FixedUpdate()
    {
        // Debug.Log("TEEEEEEEST");

        // if (nodePath.Count == 0)
        // {
        //     return;
        // }
        
        // AStarNode target = nodePath[currentNodeIdx];
        // AStarNode nextTarget = nodePath[Math.Min(currentNodeIdx + 1, nodePath.Count-1)];
        // Vector3 targetPosition = m_MapManager.grid.LocalToWorld(target.LocalPosition);
        // Vector3 nextTargetPosition = m_MapManager.grid.LocalToWorld(nextTarget.LocalPosition);
        // float targetSpeed = Mathf.Lerp(0f, m_Car.MaxSpeed, ((nextTargetPosition - targetPosition) / m_Car.MaxSpeed).magnitude);
        // targetVelocity = (nextTargetPosition - targetPosition).normalized * targetSpeed;

        if (pathType == PathType.astar)
        {
            // Vector3 targetPosition = pathSet[currPathIdx, 1];
            // targetVelocity = (targetPosition - targetPosition).normalized * 15f;
            //
            // Vector3 targetPosition = path[currPathIdx];
            //
            
            //----------------

            // float newVelocity = velocity;
            //
            // carDistances[goalIdx] = Vector3.Distance(transform.position, gatesList[gateIdx]);
            // if (carDistances.Max() - carDistances[goalIdx] > 1)
            // {
            //     newVelocity /= 5;
            //     isSlowing[goalIdx] = true;
            // }
            // else
            // {
            //     isSlowing[goalIdx] = false;
            // }
            //
            // Vector3 targetPosition = astarPath[currPathIdx];
            // Vector3 padding = new Vector3(0, 0, 0);
            // PdControllSimpleAstar(targetPosition, newVelocity, padding);

        }

        // Update leader position
        else if (pathType == PathType.lineOfSight)
        {
            // if (isLeader)
            // {
            //     leaderPosition = transform.position;
            // }
            //
            // if (update[goalIdx])
            // {
            //     relativePosition = gatesList[gateIdx] - leaderGatesList[gateIdx];
            //     update[goalIdx] = false;
            // }
            //

            float newVelocity = velocity;
            carDistances[goalIdx] = Vector3.Distance(transform.position, gatesList[gateIdx]);
            // if (gateIdxs[goalIdx] > gateIdxs.Min())
            // {
            //     print("HELLLLPPPP " + goalIdx);
            // }
            // print("Car " + goalIdx + " gateIdx = " + gateIdx);
            
            // if (carDistances.Max() - carDistances[goalIdx] > 1 || gateIdxs[goalIdx] > gateIdxs.Min())
            // {
            //     // print("HEYYYYYY");
            //     // newVelocity /= 5;
            //     isSlowing[goalIdx] = true;
            // }
            // else if (gateIdxs[goalIdx] < gateIdxs.Max())
            // {
            //     newVelocity = velocity * 10;
            //     isSlowing[goalIdx] = false;
            // }
            // else
            // {
            //     newVelocity = velocity * 10;
            //     isSlowing[goalIdx] = false;
            // }
                
            Vector3 targetPosition = path[currPathIdx];
            Vector3 padding = new Vector3(0, 0, 0);
            // if (!isLeader)
            // {
            //     // targetPosition = leaderPosition + relativePosition;
            //     // targetPosition = lineIntersection(path[currPathIdx], path[currPathIdx + 1], leaderPosition,
            //     //     leaderPosition + relativePosition);
            //     
            //     Debug.Log(currPathIdx);
            //
            //     Vector3 point = leaderPosition + relativePosition;
            //     Vector3 linePoint;
            //     Vector3 lineDir;
            //     if (gateIdx == 0)
            //     {
            //         linePoint = pathSet[0, 0];
            //         lineDir = gatesList[gateIdx] - linePoint;
            //     }
            //     else
            //     {
            //         linePoint = gatesList[gateIdx - 1];
            //         lineDir = gatesList[gateIdx] - linePoint;
            //     }
            //     // Vector3 linePoint = path[currPathIdx+2];
            //     // Vector3 lineDir = path[currPathIdx + 3] - linePoint;
            //     
            //     lineDir.Normalize();
            //     Vector3 v = point - linePoint;
            //     float d = Vector3.Dot(v, lineDir);
            //     Vector3 closest = linePoint + (lineDir * d);
            //
            //     targetPosition = closest;
            //     
            //     // Debug.DrawLine(path[currPathIdx], path[currPathIdx + 1], Color.red);
            //     // Debug.DrawLine(leaderPosition, leaderPosition + relativePosition, Color.yellow);
            //     // Debug.DrawLine(transform.position + height, targetPosition + height, Color.yellow);
            //     padding = Vector3.Normalize(lineDir) * ghostLeader;
            //     ghostPos = targetPosition + padding;
            // }
            
            tgt = targetPosition;

            // Vector3 targetPosition = path[currPathIdx];

            // Debug.DrawLine(transform.position, targetPosition, Color.cyan);
            
            PdControllSimpleLOS(targetPosition, newVelocity, padding);
            
            // PdControll(targetPosition, targetVelocity);
        }
        
    }
    
    private void PdControllSimpleAstar(Vector3 targetPosition, float velocity, Vector3 padding)
    {
        Vector3 currentPos = transform.position;

        Vector3 paddedPos = targetPosition + padding;

        // Vector3 ghostTarget = targetPosition + (Vector3.Normalize(targetPosition - transform.position) * followThrough);
        // Vector3 ghostTarget = targetPosition;
        
        Vector3 posError = paddedPos - currentPos;
        
        Vector3 tv = Vector3.Normalize(paddedPos - transform.position) * velocity;
        Vector3 velError = tv - my_rigidbody.velocity;
        Vector3 desiredAccel = (k_p * posError + k_d * velError).normalized;

        float steering = Vector3.Dot(desiredAccel, transform.right);
        float acceleration = Vector3.Dot(desiredAccel, transform.forward);

        // if (Vector3.Angle(transform.forward, targetPosition) > 90)
        // {
        //     // acceleration = 0;
        // }
        
        float threshold = 3f;
        float gateThreshold = 1.5f;
        float distance = Vector3.Distance(paddedPos, currentPos);
        float gateDistance = Vector3.Distance(gatesList[gateIdx], currentPos);

        if (distance <= threshold)
        {
            currPathIdx++;
        }

        if (gateDistance <= gateThreshold)
        {
            gateIdx++;
            for (int i = 0; i < update.Length; i++)
            {
                if (i != leaderIdx)
                    update[i] = true;
            }
        }

        m_Car.Move(steering, acceleration, acceleration, 0f);
    }

    private void PdControllSimpleLOS(Vector3 targetPosition, float velocity, Vector3 padding)
    {
        Vector3 currentPos = transform.position;

        Vector3 paddedPos = targetPosition + padding;

        // Vector3 ghostTarget = targetPosition + (Vector3.Normalize(targetPosition - transform.position) * followThrough);
        // Vector3 ghostTarget = targetPosition;
        
        Vector3 posError = paddedPos - currentPos;

        // if (!isLeader)
        // {
        //     if (posError.magnitude > (ghostLeader * 1.5f))
        //     {
        //         velocity *= 6;
        //     }
        // }
        
        Vector3 tv = Vector3.Normalize(paddedPos - transform.position) * velocity;
        Vector3 velError = tv - my_rigidbody.velocity;
        Vector3 desiredAccel = (k_p * posError + k_d * velError).normalized;

        float steering = Vector3.Dot(desiredAccel, transform.right);
        float acceleration = Vector3.Dot(desiredAccel, transform.forward);

        // if (Vector3.Angle(transform.forward, targetPosition) > 90)
        // {
        //     // acceleration = 0;
        // }
        
        float threshold = 6f;
        float gateThreshold = 3f;
        float distance = Vector3.Distance(paddedPos, currentPos);
        float gateDistance = Vector3.Distance(gatesList[gateIdx], currentPos);

        // if (isLeader && distance <= threshold)
        // {
        //     currPathIdx++;
        // }
        
        if (distance <= threshold)
        {
            currPathIdx++;
        }

        if (gateDistance <= gateThreshold)
        {
            gateIdx++;
            gateIdxs[goalIdx]++;
            for (int i = 0; i < update.Length; i++)
            {
                if (i != leaderIdx)
                    update[i] = true;
            }
        }
        
        m_Car.Move(steering, acceleration, acceleration, 0f);
    }


    
    //  private void PdControll(Vector3 targetPosition, Vector3 targetVelocity)
    // {
    //     Vector3 current_position = transform.position;
    //     
    //     // Number nodes to lookahead to see if path is turning
    //     int lookahead = 0;
    //     int lookahead_index = Mathf.Min(currentNodeIdx + lookahead, nodePath.Count - 1);
    //
    //     Vector3 current_direction = transform.forward;
    //     Vector3 lookahead_position = nodePath[lookahead_index].GetGlobalPosition();
    //     Vector3 direction_to_lookahead_index = current_position - lookahead_position;
    //
    //     float angle_between_vectors = Vector3.Angle(current_direction.normalized, direction_to_lookahead_index.normalized);
    //
    //     // A threshold when we consider a checkpoint reached
    //     float reach_threshold = 1f;
    //
    //     reach_threshold *= angle_between_vectors / 180.0f;
    //
    //
    //     Vector3 pos_error = targetPosition - current_position;
    //     Vector3 vel_error = targetVelocity - my_rigidbody.velocity;
    //     Vector3 desired_acceleration = (k_p * pos_error + k_d * vel_error).normalized; // normalize to receive steering and accel values between (-1, 1)
    //
    //     float steering = Vector3.Dot(desired_acceleration, transform.right);
    //     float acceleration = Vector3.Dot(desired_acceleration, transform.forward);
    //
    //     float distance = Vector3.Distance(targetPosition, current_position);
    //     if (distance < reach_threshold || distance < 3f)
    //     {
    //         currentNodeIdx = Mathf.Min(currentNodeIdx + 1, nodePath.Count - 1);
    //         targetPosition = nodePath[currentNodeIdx].GetGlobalPosition();
    //     }
    //
    //     for (int i = currentNodeIdx + 1; i < nodePath.Count; ++i)
    //     {
    //         float current_tracked_distance = Vector3.Distance(targetPosition, current_position);
    //         float potential_tracked_distance = Vector3.Distance(nodePath[i].GetGlobalPosition(), current_position);
    //         
    //         if (current_tracked_distance > potential_tracked_distance)
    //         {
    //             // found a closer point, i, no need to keep looking
    //             currentNodeIdx = i;
    //             break; 
    //         }
    //     }
    //
    //     m_Car.Move(steering, acceleration, acceleration, 0f);
    // }
    
    private Vector3 lineIntersection(Vector3 pt1, Vector3 pt2, Vector3 pt3, Vector3 pt4)
    {
        Debug.Log(pt1);
        Debug.Log(pt2);
        Debug.Log(pt3);
        Debug.Log(pt4);
        Debug.Log("I hate cars");
        float m1 = (pt1.x - pt2.y) / (pt2.x - pt2.z);
        float m2 = (pt3.x - pt3.y) / (pt4.x - pt4.z);
    
        float a1 = m1;
        float b1 = -1;
        float c1 = -1 * (pt1.z - (m1 * pt1.x));
        
        float a2 = m2;
        float b2 = -1;
        float c2 = -1 * (pt3.z - (m2 * pt3.x));
    
        float delta = (a1 * b2) - (a2 * b1);
        
        float x = ((b2 * c1) - (b1 * c2)) / delta;
        float z = ((a1 * c2) - (a2 * c1)) / delta;
    
        return new Vector3(x, pt1.y, z);
    }

    private void OnDrawGizmos()
    {
        // throw new NotImplementedException();
        
        // Draw car paths
        if (pathType == PathType.lineOfSight && pathSet != null)
        {
            for (int i = 0; i < pathSet.GetLength(0); i++)
            {
                // Debug.DrawLine(pathSet[i, 0] + height, pathSet[i, 1] + height, Color.green, 100f);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(pathSet[i, 0] + height, pathSet[i, 1] + height);
            }
        }
        

        if (pathType == PathType.astar && astarPath != null)
        {
            // Draw path
            for (int i = 0; i < astarPath.Count - 1; i++)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(astarPath[i] + height, astarPath[i+1] + height);
            }
            
            // Draw spheres at target points
            for (int i = 0; i < astarPath.Count; i++)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(astarPath[i] + height, radius:0.5f);

                if (isSlowing[goalIdx])
                {
                    Gizmos.color = Color.red;
                }
                else
                {
                    Gizmos.color = Color.cyan;
                }
                Gizmos.DrawLine(transform.position + height, astarPath[currPathIdx] + height);
            }

            if (goalIdx == 1)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawSphere(gatesList[gateIdx] + height, 0.5f);
            }
        }
        
        // Draw blue spheres at target points
        if (pathType == PathType.lineOfSight && path != null)
        {
            foreach (var point in path)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawSphere(point + height, 0.5f);

                if (isSlowing[goalIdx])
                {
                    Gizmos.color = Color.red;
                }
                else
                {
                    Gizmos.color = Color.cyan;
                }
                Gizmos.DrawLine(transform.position + height, tgt + height);
            }
        }
        
        // // Draw orthogonal leader line
        // if (isLeader)
        // {
        //     // Debug.Log("LEADING");
        //     Vector3 orth = Vector3.Cross(transform.forward, new Vector3(0, 1, 0)).normalized * 35;
        //     Vector3 l = transform.position + orth;
        //     Vector3 r = transform.position + (Quaternion.Euler(0, 180f, 0) * orth);
        //     Gizmos.color = Color.black;
        //     Gizmos.DrawLine(l + height, r + height);
        // }
        // if (!isLeader)
        // {
        //     // Gizmos.color = Color.black;
        //     // Gizmos.DrawLine(leaderPosition + height, leaderPosition + relativePosition + height);
        //     Gizmos.color = Color.magenta;
        //     // Gizmos.DrawSphere(leaderPosition + relativePosition + height, 0.5f);
        //     Gizmos.DrawSphere(tgt + height, 0.5f);
        // }
        
        if (pathType == PathType.lineOfSight && !isLeader)
        {
            // Gizmos.color = Color.black;
            // Gizmos.DrawLine(leaderPosition + height, leaderPosition + relativePosition + height);
            Gizmos.color = Color.magenta;
            // Gizmos.DrawSphere(leaderPosition + relativePosition + height, 0.5f);
            Gizmos.DrawSphere(tgt + height, 0.5f);
            
            // Gizmos.color = Color.cyan;
            // Gizmos.DrawLine(transform.position + height, tgt + height);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(ghostPos + height, 0.5f);
            
            // Gizmos.color = Color.cyan;
            // Gizmos.DrawLine(transform.position + height, ghostPos + height);
        }

        if (rrtPath != null)
        {
            for (int i = 1; i < rrtPath.Count; i++)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(rrtPath[i - 1] + height, rrtPath[i] + height);
            }
        }
    }
    
    private Vector2 GetInitialDirection()
    {
        float angle = transform.rotation.eulerAngles.y;
        angle = Mathf.Round(angle / 45f) * 45f;
        angle *= Mathf.Deg2Rad;
        Vector2 ret = new Vector2(Mathf.RoundToInt(Mathf.Sin(angle)), Mathf.RoundToInt(Mathf.Cos(angle)));

        return ret;
    }
}