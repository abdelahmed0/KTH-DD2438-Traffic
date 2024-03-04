using Imported.StandardAssets.Vehicles.Car.Scripts;
using System;
using System.Linq;
using System.Collections.Generic;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

using aStar;
using avoidance;
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
    private float StuckTime = 2f; // Seconds after which to engage the backing up mechanism when stuck
    private float RecoveryTime = 1f; // How many seconds to engage the backing up mechanism
    private bool recoveryOn = false;
    private const float maxSpeed = 25f;
    private Agent agent;

    private static CollisionManager collisionManager;
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
            m_Detector = new CollisionDetector(m_ObstacleMap, margin: m_Collider.transform.localScale.x / 2f + 0.01f);
            Debug.Log($"Detector init: {sw.ElapsedMilliseconds} ms");

            // Init collision avoidance
            CollisionAvoidanceAlgorithm collisionAlgorithm = new RVOAlgorithm(0.5f)
            {
                maxSpeed = maxSpeed, 
                maxAngle = 25f,
                allowReversing = allowReversing,
                maxAccelaration = 25f,
                Detector = m_Detector,
                TimeLookAhead = 3f
            };
            collisionManager = new();
            collisionManager.SetCollisionAvoidanceAlgorithm(collisionAlgorithm);

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
        collisionManager.AddAgent(agent);
    }


    private void FixedUpdate()
    {
        // TODO: Try Drone RVO and add car controller update
        // TODO: HVOs are still too symmetric and contain errors

        if (nodePath.Count == 0)
            return;

        Vector3 targetVelocity = CalculateTargetVelocity();

        float avoidanceRadius = 1f; // FIXME: localscale should give car length, but is somehow way too large
        agent.Update(new Agent(Vec3To2(transform.position), Vec3To2(my_rigidbody.velocity), Vec3To2(targetVelocity), avoidanceRadius));

        Vector2 newVelocity = collisionManager.CalculateNewVelocity(agent, out bool isColliding);

        // Avoid other agents if collision is detected via VO
        Vector3 avoidanceVelocity = Vec2To3(newVelocity);
        Vector3 avoidancePosition = transform.position + avoidanceVelocity;

        PdControll(avoidancePosition, avoidanceVelocity, recoveryOn);

        Debug.DrawLine(transform.position, nodePath[currentNodeIdx].GetGlobalPosition(), Color.magenta);
        // Debug.DrawLine(transform.position, avoidancePosition, Color.blue);
        // Debug.DrawLine(transform.position, transform.position + avoidanceVelocity, Color.yellow);
    }

    private void PdControll(Vector3 targetPosition, Vector3 targetVelocity, bool recoveryOn)
    {
        Vector3 current_position = transform.position;

        if (Vector3.Distance(targetPosition, current_position) < 20f)
        {
            currentNodeIdx = Mathf.Min(currentNodeIdx + 1, nodePath.Count - 1);
        }
        
        // Move via PD controller
        Vector3 pos_error = targetPosition - current_position;
        Vector3 vel_error = targetVelocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = (k_p * pos_error + k_d * vel_error).normalized; // normalize to receive steering and accel values between (-1, 1)

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        m_Car.Move(steering, acceleration, acceleration, 0f);
    }

    private Vector3 CalculateTargetVelocity()
    {
        // Pure pursuit target
        int targetIdx = NextVisibleNode();
        if (targetIdx == -1)
        {
            // Engage recovery mechanism when occluded
            targetIdx = LastVisibleNode();
            currentNodeIdx = targetIdx;

            Vector3 recoveryPosition = nodePath[targetIdx].GetGlobalPosition();

            float targetSpeed = Mathf.Lerp(0.01f, maxSpeed, ((Vec3To2(recoveryPosition) - Vec3To2(transform.position)) / maxSpeed).magnitude);
            targetVelocity = (recoveryPosition - transform.position).normalized * targetSpeed;
        }

        // Use lookahead
        Vector3 targetPosition = nodePath[targetIdx].GetGlobalPosition();
        Vector2 targetPosition2d = Vec3To2(targetPosition);
        Vector2 position2d = Vec3To2(transform.position);

        Vector2 currentDirection = new(my_rigidbody.velocity.normalized.x, my_rigidbody.velocity.normalized.z);
        Vector2 directionToLookhead = (targetPosition2d - position2d).normalized;

        float lookaheadAngle = Vector2.Angle(directionToLookhead, currentDirection);
        float pursuitTargetSpeed = Mathf.Lerp(0.01f, maxSpeed, 1f - Mathf.Clamp01(lookaheadAngle / 180f));
        targetVelocity = (targetPosition - transform.position).normalized * pursuitTargetSpeed;
        
        return targetVelocity;
    }

    private int NextVisibleNode()
    {
        float acceptableAngleDiff = 1f;

        for (int i = currentNodeIdx; i < nodePath.Count; ++i)
        {
            Vector2 currentPosition = Vec3To2(transform.position);
            Vector2 pathNodePosition = Vec3To2(nodePath[i].GetGlobalPosition());
            Vector2 direction = (pathNodePosition - currentPosition).normalized;

            // Disregard sideways nodes since car cannot drive to those
            if (!m_Detector.LineCollision(currentPosition, pathNodePosition)  
                && (Vector2.Angle(transform.forward, direction) > 90 + acceptableAngleDiff 
                    || Vector2.Angle(transform.forward, direction) < 90 - acceptableAngleDiff))
            {
                return i;
            }
        }
        return -1;
    }

    private int LastVisibleNode()
    {
        float acceptableAngleDiff = 10f;

        for (int i = currentNodeIdx; i >= 0; --i)
        {
            Vector2 currentPosition = Vec3To2(transform.position);
            Vector2 pathNodePosition = Vec3To2(nodePath[i].GetGlobalPosition());
            Vector2 direction = (pathNodePosition - currentPosition).normalized;

            // Disregard sideways nodes since car cannot drive to those
            if (!m_Detector.LineCollision(currentPosition, pathNodePosition)  
                && (Vector2.Angle(transform.forward, direction) > 90 + acceptableAngleDiff 
                    || Vector2.Angle(transform.forward, direction) < 90 - acceptableAngleDiff))
            {
                return i;
            }
        }
        return Math.Max(0, currentNodeIdx - 100);
    }

    private void OnDrawGizmos() {
        if (drawDebug)
        {
            collisionManager?.DrawDebug(agent);
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