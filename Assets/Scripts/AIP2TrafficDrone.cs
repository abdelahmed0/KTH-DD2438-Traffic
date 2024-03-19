using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Vehicle;
using System;
using System.Linq;
using System.Collections.Generic;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

using aStar;
using avoidance;
using PathPlanning;

[RequireComponent(typeof(DroneController))]
public class AIP2TrafficDrone : MonoBehaviour
{   
    public float colliderResizeFactor = 2f;
    public int numberSteeringAngles = 5;
    public bool allowReversing = true;   
    public bool smoothPath = true;
    public float k_p = 2f;
    public float k_d = 1f;
    private DroneController m_Drone;
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private LineOfSightGoal m_CurrentGoal;
    private CapsuleCollider m_Collider;
    Rigidbody my_rigidbody;
    private List<AStarNode> nodePath = new();
    private int currentNodeIdx;
    private HybridAStarGenerator pathFinder = null;
    public bool drawDebug = true;

    private Agent agent;
    private Vector3 localGoal;

    private static CollisionManager collisionManager = null;
    private static bool StaticInitDone = false;
    private static CollisionDetector m_Detector = null;

    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        m_CurrentGoal = (LineOfSightGoal)FindObjectOfType<GameManagerA2>().vehicleToGoalMapping[gameObject];
        my_rigidbody = GetComponent<Rigidbody>();
        m_Collider = GetComponent<CapsuleCollider>();

        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;

        if (!StaticInitDone)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            // Rescale grid to have square shaped grid cells with size proportional to drone size
            float gridCellSize = colliderResizeFactor * m_Collider.height;
            Vector3 gridScale = m_ObstacleMap.mapGrid.transform.localScale;

            m_ObstacleMapManager.grid.cellSize = new Vector3(
                gridCellSize / gridScale.x,
                gridCellSize / gridScale.z,
                gridCellSize / gridScale.y);
            m_MapManager.Initialize();
            m_ObstacleMapManager.Initialize();
            m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;
            Debug.Log($"Grid rescaling: {sw.ElapsedMilliseconds} ms");

            sw.Restart();
            m_Detector = new CollisionDetector(m_ObstacleMap, margin: m_Collider.radius / 2f + 0.0001f);
            Debug.Log($"Detector init: {sw.ElapsedMilliseconds} ms");

            // Init collision avoidance
            CollisionAvoidanceAlgorithm collisionAlgorithm = new HRVOAlgorithm()
            {
                maxSpeed = m_Drone.max_speed,
                maxAngle = 40f,
                allowReversing = allowReversing,
                maxAccelaration = m_Drone.max_acceleration,
                Detector = m_Detector,
                TimeLookAhead = 1.5f
            };

            collisionManager = new CollisionManager();
            collisionManager.SetCollisionAvoidanceAlgorithm(collisionAlgorithm);

            StaticInitDone = true;
            sw.Stop();
        }

        var localStart = m_ObstacleMap.mapGrid.WorldToLocal(transform.position);
        localGoal = m_ObstacleMap.mapGrid.WorldToLocal(m_CurrentGoal.targetPosition);

        // Generate path with Hybrid A*
        currentNodeIdx = 0;
        pathFinder = new HybridAStarGenerator(m_ObstacleMap.mapGrid, m_ObstacleMap, 40f, m_Collider,
                        colliderResizeFactor, true, allowReversing, 2f)
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
            Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(old_wp), m_ObstacleMap.mapGrid.LocalToWorld(wp.LocalPosition), Color.green, 1f);
            old_wp = wp.LocalPosition;
        }

        // Initialize velocity obstacles for traffic
        agent = new Agent(Vec3To2(transform.position), Vec3To2(my_rigidbody.velocity), Vector3.zero, m_Collider.radius * colliderResizeFactor);
        collisionManager.AddAgent(agent);
    }

    private void FixedUpdate()
    {
        if (nodePath.Count == 0)
            return;

        Vector3 targetVelocity = CalculateTargetVelocity();

        float avoidanceRadius = m_Collider.radius * 3f;
        agent.Update(new Agent(Vec3To2(transform.position), Vec3To2(my_rigidbody.velocity), Vec3To2(targetVelocity), avoidanceRadius));

        Vector2 newVelocity = collisionManager.CalculateNewVelocity(agent, out bool velColliding);

        // Avoid other agents if collision is detected via VO
        Vector3 avoidanceVelocity = Vec2To3(newVelocity);
        Vector3 avoidancePosition = transform.position + avoidanceVelocity;

        PdControll(avoidancePosition, avoidanceVelocity);

        Debug.DrawLine(transform.position, nodePath[currentNodeIdx].GetGlobalPosition(), Color.magenta);
        // Debug.DrawLine(transform.position, avoidancePosition, Color.blue);
        // Debug.DrawLine(transform.position, transform.position + avoidanceVelocity, Color.yellow);
    }

    private void PdControll(Vector3 targetPosition, Vector3 targetVelocity)
    {
        Vector3 current_position = transform.position;

        if (Vector3.Distance(targetPosition, current_position) < 100f)
        {
            currentNodeIdx = Mathf.Min(currentNodeIdx + 1, nodePath.Count - 1);
        }
        
        // Move via PD controller
        Vector3 pos_error = targetPosition - current_position;
        Vector3 vel_error = targetVelocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = (k_p * pos_error + k_d * vel_error).normalized; // normalize to receive steering and accel values between (-1, 1)

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        m_Drone.Move(steering, acceleration);
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

            float targetSpeed = Mathf.Lerp(0.01f, m_Drone.max_speed, ((Vec3To2(recoveryPosition) - Vec3To2(transform.position)) / m_Drone.max_speed).magnitude);
            return (recoveryPosition - transform.position).normalized * targetSpeed;
        }

        // Use lookahead
        Vector3 targetPosition = nodePath[targetIdx].GetGlobalPosition();
        Vector2 targetPosition2d = Vec3To2(targetPosition);
        Vector2 position2d = Vec3To2(transform.position);

        Vector2 currentDirection = new(my_rigidbody.velocity.normalized.x, my_rigidbody.velocity.normalized.z);
        Vector2 directionToLookhead = (targetPosition2d - position2d).normalized;

        float lookaheadAngle = Vector2.Angle(directionToLookhead, currentDirection);
        float pursuitTargetSpeed = Mathf.Lerp(0.01f, m_Drone.max_speed, 1f - Mathf.Clamp01(lookaheadAngle / 180f));
        Vector3 targetVelocity = (targetPosition - transform.position).normalized * pursuitTargetSpeed;
        
        return targetVelocity;
    }

    private int NextVisibleNode()
    {
        for (int i = currentNodeIdx; i < nodePath.Count; ++i)
        {
            Vector2 currentPosition = Vec3To2(transform.position);
            Vector2 pathNodePosition = Vec3To2(nodePath[i].GetGlobalPosition());

            if (!m_Detector.LineCollision(currentPosition, pathNodePosition))
            {
                return i;
            }
        }
        return -1;
    }

    private int LastVisibleNode()
    {
        for (int i = currentNodeIdx; i >= 0; --i)
        {
            Vector3 pathNodePosition = nodePath[i].GetGlobalPosition();
            if (!m_Detector.LineCollision(Vec3To2(transform.position), Vec3To2(pathNodePosition)))
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