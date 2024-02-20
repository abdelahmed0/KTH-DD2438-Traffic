using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Vehicle;
using System;
using System.Linq;
using System.Collections.Generic;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

using aStar;
using vo;

[RequireComponent(typeof(DroneController))]
public class AIP2TrafficDrone : MonoBehaviour
{   
    public float colliderResizeFactor = 2f;
    public int numberSteeringAngles = 5;
    public bool allowReversing = true;   
    public bool smoothPath = true;   
    private Vector3 targetVelocity;
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
    public bool drawDebug = false;

    private Agent agent;

    private static VOManager voManager = new();
    private static bool mapResized = false;

    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        m_CurrentGoal = (LineOfSightGoal)FindObjectOfType<GameManagerA2>().vehicleToGoalMapping[gameObject];
        my_rigidbody = GetComponent<Rigidbody>();
        m_Collider = GetComponent<CapsuleCollider>();

        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;

        if (!mapResized)
        {
            // Rescale grid to have square shaped grid cells with size proportional to drone size
            float gridCellSize = 3f * m_Collider.height;
            Vector3 gridScale = m_ObstacleMap.mapGrid.transform.localScale;

            m_ObstacleMapManager.grid.cellSize = new Vector3(
                gridCellSize / gridScale.x,
                gridCellSize / gridScale.z,
                gridCellSize / gridScale.y);
            m_MapManager.Initialize();
            m_ObstacleMapManager.Initialize();
            m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;
            mapResized = true;
        }

        var localStart = m_ObstacleMap.mapGrid.WorldToLocal(transform.position);
        var localGoal = m_ObstacleMap.mapGrid.WorldToLocal(m_CurrentGoal.targetPosition);

        // Generate path with Hybrid A*
        currentNodeIdx = 0;
        pathFinder = new HybridAStarGenerator(m_ObstacleMap.mapGrid, m_ObstacleMap, 40f, m_Collider,
                        colliderResizeFactor, false, allowReversing, 2f);
        nodePath = pathFinder.GeneratePath(
            new Vector3(localStart.x, 0.05f, localStart.z),
            new Vector3(localGoal.x, 0.05f, localGoal.z),
            transform.eulerAngles.y,
            numberSteeringAngles);

        nodePath = smoothPath ? pathFinder.SmoothPath(nodePath) : nodePath;

        // Vector3 old_wp = localStart;
        // foreach (var wp in nodePath)
        // {
        //     Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(old_wp), m_ObstacleMap.mapGrid.LocalToWorld(wp.LocalPosition), Color.magenta, 1000f);
        //     old_wp = wp.LocalPosition;
        // }

        // Initialize velocity obstacles for traffic
        agent = new Agent(Vec3To2(transform.position), Vec3To2(my_rigidbody.velocity), Vec3To2(targetVelocity), m_Collider.radius * colliderResizeFactor);
        voManager.AddAgent(agent);
    }

    private void FixedUpdate()
    {
        if (nodePath.Count == 0)
        {
            return;
        }

        AStarNode target = nodePath[currentNodeIdx];
        AStarNode nextTarget = nodePath[Math.Min(currentNodeIdx + 1, nodePath.Count-1)];
        Vector3 targetPosition = m_MapManager.grid.LocalToWorld(target.LocalPosition);
        Vector3 nextTargetPosition = m_MapManager.grid.LocalToWorld(nextTarget.LocalPosition);
        float targetSpeed = Mathf.Lerp(0f, m_Drone.max_speed, ((nextTargetPosition - targetPosition) / m_Drone.max_speed).magnitude);
        targetVelocity = (nextTargetPosition - targetPosition).normalized * targetSpeed;

        // Apply weighted avoidance via velocity obstacles
        float avoidanceRadius = colliderResizeFactor * m_Collider.radius;
        agent.Update(new Agent(Vec3To2(transform.position), Vec3To2(my_rigidbody.velocity), Vec3To2(targetVelocity), avoidanceRadius));

        voManager.CalculateNewVelocity(agent, m_Drone.max_speed, 
            maxAngle:40f, 
            allowReversing:true,
            out bool isColliding, 
            out Vector2 newVelocity);

        float avoidanceWeight = isColliding ? 1f : 0f;
        
        Vector3 avoidanceVelocity = Vec2To3(newVelocity);
        Vector3 avoidancePosition = transform.position + avoidanceVelocity.normalized * pathFinder.globalStepDistance;

        PdControll(targetPosition * (1f - avoidanceWeight) + avoidancePosition * avoidanceWeight,
                   targetVelocity * (1f - avoidanceWeight) + avoidanceVelocity * avoidanceWeight);

        Debug.DrawLine(transform.position, targetPosition, Color.magenta);
        Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.white);
        Debug.DrawLine(transform.position, transform.position + targetVelocity, Color.blue);
        Debug.DrawLine(transform.position, transform.position + avoidanceVelocity, Color.red);
        // Debug.DrawLine(transform.position, avoidancePosition, Color.red);
    }

    private void PdControll(Vector3 targetPosition, Vector3 targetVelocity)
    {
        Vector3 current_position = transform.position;
        
        // Number nodes to lookahead to see if path is turning
        int lookahead = 10;

        // A threshold when we consider a checkpoint reached
        float reach_threshold = 9.5f;

        int lookahead_index = Mathf.Min(currentNodeIdx + lookahead, nodePath.Count - 1);

        Vector3 current_direction = transform.forward;
        Vector3 direction_to_lookahead_index = current_position - nodePath[lookahead_index].GetGlobalPosition();
        float angle_between_vectors = Vector3.Angle(current_direction.normalized, direction_to_lookahead_index.normalized);

        reach_threshold *= angle_between_vectors / 180.0f;

        this.targetVelocity = targetVelocity;

        Vector3 pos_error = targetPosition - current_position;
        Vector3 vel_error = this.targetVelocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = (k_p * pos_error + k_d * vel_error).normalized; // normalize to receive steering and accel values between (-1, 1)

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        float distance = Vector3.Distance(targetPosition, current_position);
        if (distance < reach_threshold)
        {
            currentNodeIdx = Mathf.Min(currentNodeIdx + 1, nodePath.Count - 1);;
            if (currentNodeIdx >= nodePath.Count) return;
            targetPosition = nodePath[currentNodeIdx].GetGlobalPosition();
        }

        for (int i = currentNodeIdx + 1; i < nodePath.Count; ++i)
        {
            float current_tracked_distance = Vector3.Distance(targetPosition, current_position);
            float potential_tracked_distance = Vector3.Distance(nodePath[i].GetGlobalPosition(), current_position);
            
            if (current_tracked_distance > potential_tracked_distance + 0.01f)
            {
                // found a closer point, i, no need to keep looking
                currentNodeIdx = i;
                break; 
            }
        }

        m_Drone.Move(steering, acceleration);
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