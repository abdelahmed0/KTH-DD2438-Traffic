using Imported.StandardAssets.Vehicles.Car.Scripts;
using System;
using System.Linq;
using System.Collections.Generic;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

using aStar;
using vo;
using PathPlanning;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : MonoBehaviour
{
    public float colliderResizeFactor = 2f;
    public int numberSteeringAngles = 3;
    public bool allowReversing = true;   
    public bool smoothPath = false;     
    private Vector3 targetVelocity;
    public float k_p = 2f;
    public float k_d = 1f;
          
    private CarController m_Car; // the car controller we want to use
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private LineOfSightGoal m_CurrentGoal;
    private BoxCollider m_Collider;
    Rigidbody my_rigidbody;
    private List<AStarNode> nodePath = new();
    private int currentNodeIdx;
    private HybridAStarGenerator pathFinder = null;
    public bool drawDebug = false;
    private float StuckTime = 1f; // Seconds after which to engage the backing up mechanism when stuck

    private Agent agent;

    private static VOManager voManager;
    private static CollisionDetector m_Detector = null;
    private static bool StaticInitDone = false;

    private void Start()
    {
        m_Car = GetComponent<CarController>();
        m_CurrentGoal = (LineOfSightGoal)FindObjectOfType<GameManagerA2>().vehicleToGoalMapping[gameObject]; // This car's goal
        my_rigidbody = GetComponent<Rigidbody>();

        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;
        m_Collider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();

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

            // Init collision avoidance
            voManager = new()
            {
                maxSpeed = m_Car.MaxSpeed, 
                maxAngle = m_Car.m_MaximumSteerAngle,
                allowReversing = allowReversing,
                maxAccelaration = 6f,
                Detector = m_Detector,
                TimeLookAhead = 3f
            };

            StaticInitDone = true;
            sw.Stop();
        }

        var localStart = m_ObstacleMap.mapGrid.WorldToLocal(transform.position);
        var localGoal = m_ObstacleMap.mapGrid.WorldToLocal(m_CurrentGoal.targetPosition);

        // Generate path with Hybrid A*
        currentNodeIdx = 0;
        pathFinder = new HybridAStarGenerator(m_ObstacleMap.mapGrid, m_ObstacleMap, m_Car.m_MaximumSteerAngle, m_Collider,
                        colliderResizeFactor, false, allowReversing, 2f)
        {
            Detector = m_Detector
        };

        nodePath = pathFinder.GeneratePath(
            new Vector3(localStart.x, 0.05f, localStart.z),
            new Vector3(localGoal.x, 0.05f, localGoal.z),
            transform.eulerAngles.y,
            numberSteeringAngles);

        nodePath = smoothPath ? pathFinder.SmoothPath(nodePath) : nodePath;

        Vector3 old_wp = localStart;
        foreach (var wp in nodePath)
        {
            Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(old_wp), m_ObstacleMap.mapGrid.LocalToWorld(wp.LocalPosition), Color.green, 2f);
            old_wp = wp.LocalPosition;
        }

        // Initialize velocity obstacles for traffic
        agent = new Agent(Vec3To2(transform.position), Vec3To2(my_rigidbody.velocity), Vector3.zero, m_Collider.transform.localScale.z * colliderResizeFactor);
        voManager.AddAgent(agent);
    }


    private void FixedUpdate()
    {
        // TODO: Car collision avoidance
        // TODO: VO RVO HRVO class refactoring
        // TODO: Fix HRVO?

        if (nodePath.Count == 0)
        {
            return;
        }
        if (StuckTime <= 0f)
        {
            StuckTime = 1f;
            currentNodeIdx = Math.Max(0, currentNodeIdx - 20);
        }
        else if (my_rigidbody.velocity.magnitude < 0.1f)
            StuckTime -= Time.fixedDeltaTime;

        CalculateTargets(lookaheadDistance:20f, 
            out Vector3 targetPosition, out Vector3 targetVelocity);

        float avoidanceRadius = colliderResizeFactor * m_Collider.transform.localScale.z;
        agent.Update(new Agent(Vec3To2(transform.position), Vec3To2(my_rigidbody.velocity), Vec3To2(targetVelocity), avoidanceRadius));

        Vector2 newVelocity = voManager.CalculateNewHRVOVelocity(agent, Time.fixedDeltaTime, out bool isColliding);

        if (isColliding)
        {
            // Avoid other agents if collision is detected via VO
            Vector3 avoidanceVelocity = Vec2To3(newVelocity);
            Vector3 avoidancePosition = transform.position + avoidanceVelocity;

            PdControll(avoidancePosition, avoidanceVelocity);
            Debug.DrawLine(transform.position, transform.position + avoidanceVelocity, Color.green);
        }
        else
        {
            PdControll(targetPosition, targetVelocity);
            Debug.DrawLine(transform.position, transform.position + targetVelocity, Color.blue);
        }

        // Debug.DrawLine(transform.position, targetPosition, Color.magenta);
        // Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.black);

    }

    private void PdControll(Vector3 targetPosition, Vector3 targetVelocity)
    {
        Vector3 current_position = transform.position;

        float distance = Vector3.Distance(targetPosition, current_position);
        if (distance < 2f)
        {
            currentNodeIdx = Mathf.Min(currentNodeIdx + 1, nodePath.Count - 1);
            return;
        }

        for (int i = currentNodeIdx + 1; i < nodePath.Count; ++i)
        {
            float current_tracked_distance = Vector3.Distance(targetPosition, current_position);
            float potential_tracked_distance = Vector3.Distance(nodePath[i].GetGlobalPosition(), current_position);
            
            if (current_tracked_distance > potential_tracked_distance)
            {
                // found a closer point
                currentNodeIdx = i;
                return;
            }
        }
        
        // Move via PD controller
        Vector3 pos_error = targetPosition - current_position;
        Vector3 vel_error = targetVelocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = (k_p * pos_error + k_d * vel_error).normalized; // normalize to receive steering and accel values between (-1, 1)

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        m_Car.Move(steering, acceleration, acceleration, 0f);
    }

    private void CalculateTargets(float lookaheadDistance, out Vector3 targetPosition, out Vector3 targetVelocity)
    {
        AStarNode target = nodePath[currentNodeIdx];
        targetPosition = m_MapManager.grid.LocalToWorld(target.LocalPosition);

        Vector3 lookaheadPosition = PurePursuitTargetPosition(lookaheadDistance);
        Vector2 lookaheadPosition2d = new(lookaheadPosition.x, lookaheadPosition.z);

        Vector2 position2d = new(transform.position.x, transform.position.z);

        // Only use lookahead if it is not occluded
        if(!m_Detector.LineCollision(position2d, lookaheadPosition2d))
        {
            Vector2 currentDirection = new(my_rigidbody.velocity.normalized.x, my_rigidbody.velocity.normalized.z);
            Vector2 directionToLookhead = (lookaheadPosition2d - position2d).normalized;

            float lookaheadAngle = Vector2.Angle(directionToLookhead, currentDirection);
            float targetSpeed = Mathf.Lerp(0f, m_Car.MaxSpeed, 1f - Mathf.Clamp01(lookaheadAngle / 60f));
            targetVelocity = (lookaheadPosition - targetPosition).normalized * targetSpeed;
        }
        else 
        { 
            AStarNode nextTarget = nodePath[Math.Min(currentNodeIdx + 1, nodePath.Count-1)];
            Vector3 nextTargetPosition = m_MapManager.grid.LocalToWorld(nextTarget.LocalPosition);

            float targetSpeed = Mathf.Lerp(0f, m_Car.MaxSpeed, ((nextTargetPosition - targetPosition) / m_Car.MaxSpeed).magnitude);
            targetVelocity = (nextTargetPosition - targetPosition).normalized * targetSpeed;
        }
    }

    private Vector3 PurePursuitTargetPosition(float lookaheadDistance)
    {
        for (int i = currentNodeIdx; i < nodePath.Count; i++)
        {
            Vector3 pathNodePosition = nodePath[i].GetGlobalPosition();
            if (Vector3.Distance(transform.position, pathNodePosition) > lookaheadDistance)
            {
                // Debug.DrawLine(transform.position, pathNodePosition, Color.white);
                return pathNodePosition;
            }
        }
        return nodePath[^1].GetGlobalPosition(); // Return last node if none found within lookahead distance
    }

    private void OnDrawGizmos() {
        if (drawDebug)
        {
            voManager?.DebugDraw(agent);
            // m_Detector?.DebugDrawBoundingBoxes();
        }
        // if (drawDebug && pathFinder != null && pathFinder.flowField != null)
        // {
        //     foreach (var posEntity in pathFinder.flowField)
        //     {
        //         var cell = new Vector3Int(posEntity.Key.x, posEntity.Key.y, 1);
        //         var cellGlobal = m_MapManager.grid.CellToWorld(cell);

        //         var gizmoSize = m_MapManager.grid.cellSize;
        //         gizmoSize.y = 0.005f;
        //         gizmoSize.Scale(m_MapManager.grid.transform.localScale * 0.8f);
                
        //         Gizmos.color = Mathf.CorrelatedColorTemperatureToRGB(1000f + 1000f * posEntity.Value);
        //         Gizmos.DrawCube(cellGlobal, gizmoSize);
        //     }
        // }
    }

    private Vector2 Vec3To2(Vector3 vec)
    {
        return new Vector2(vec.x, vec.z);
    }

    private Vector3 Vec2To3(Vector2 vec)
    {
        return new Vector3(vec.x, 0f, vec.y);
    }
}