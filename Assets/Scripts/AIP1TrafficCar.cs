using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using System.Collections.Generic;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

using aStar;
using util;
using System;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : MonoBehaviour
{
    public float colliderResizeFactor = 2f;
    public int numberSteeringAngles = 3;
    public bool allowReversing = true;   
    public bool smoothPath = false;     
    private Vector3 target_velocity;
    public float k_p = 2f;
    public float k_i = 0.1f;
    public float k_d = 1f;
    public float nodeDistThreshold = 0.2f;
          
    private CarController m_Car; // the car controller we want to use
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private LineOfSightGoal m_CurrentGoal;
    Rigidbody my_rigidbody;

    private float steering;
    private float acceleration;
    private List<AStarNode> nodePath = new();
    private int currentNodeIdx;
    private float integral = 0f;
    private HybridAStarGenerator pathFinder = null;
    public bool drawDebug = false;

    private static bool mapResized = false;
    private void Start()
    {
        m_Car = GetComponent<CarController>();
        m_CurrentGoal = (LineOfSightGoal)FindObjectOfType<GameManagerA2>().vehicleToGoalMapping[gameObject]; //This car's goal.
        my_rigidbody = GetComponent<Rigidbody>();

        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;
        var carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();

        if (!mapResized) // FIXME: Working for all cars except first one
        {
            // Rescale grid to have square shaped grid cells with size proportional to the car length
            float gridCellSize = carCollider.transform.localScale.z;
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


        // m_OtherCars = GameObject.FindGameObjectsWithTag("Player"); // TODO: use

        var localStart = m_ObstacleMap.mapGrid.WorldToLocal(transform.position);
        var localGoal = m_ObstacleMap.mapGrid.WorldToLocal(m_CurrentGoal.targetPosition);
        // Generate path
        currentNodeIdx = 0;
        pathFinder = new HybridAStarGenerator(m_ObstacleMap.mapGrid, m_ObstacleMap, m_Car.m_MaximumSteerAngle, carCollider,
                        colliderResizeFactor, false, allowReversing, 2f);
        nodePath = pathFinder.GeneratePath(
            new Vector3(localStart.x, 0.05f, localStart.z),
            new Vector3(localGoal.x, 0.05f, localGoal.z),
            transform.eulerAngles.y,
            numberSteeringAngles);

        nodePath = smoothPath ? pathFinder.SmoothPath(nodePath) : nodePath;

        Vector3 old_wp = localStart;
        foreach (var wp in nodePath)
        {
            Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(old_wp), m_ObstacleMap.mapGrid.LocalToWorld(wp.LocalPosition), Color.magenta, 1000f);
            old_wp = wp.LocalPosition;
        }
    }


    private void FixedUpdate()
    {
        if (nodePath.Count == 0)
        {
            return;
        }
        // Vector3 localPosition = m_ObstacleMap.mapGrid.WorldToLocal(transform.position);

        // Vector3 localNextNode = nodePath[currentNodeIdx].LocalPosition;
        // int nextNextIndex = Math.Clamp(currentNodeIdx+1, 0, nodePath.Count-1);
        // Vector3 localNextNextNode = nodePath[nextNextIndex].LocalPosition;

        // if (currentNodeIdx < nodePath.Count - 1 
        //     && (Vector3.Distance(localPosition, localNextNode) < nodeDistThreshold 
        //         || Vector3.Distance(localPosition, localNextNextNode) <  Vector3.Distance(localPosition, localNextNode)))
        // {
        //     currentNodeIdx++;
        // }
        // PidControllTowardsPosition();

        PdControll();
    }

     private void PdControll()
    {
        currentNodeIdx = Mathf.Min(currentNodeIdx, nodePath.Count - 1);
        Vector3 current_position = transform.position;

        AStarNode target = nodePath[currentNodeIdx];
        AStarNode nextTarget = nodePath[Math.Min(currentNodeIdx + 1, nodePath.Count-1)];
        Vector3 targetPosition = m_MapManager.grid.LocalToWorld(target.LocalPosition);
        Vector3 nextTargetPosition = m_MapManager.grid.LocalToWorld(nextTarget.LocalPosition);
        
        // Number nodes to lookahead to see if path is turning
        int lookahead = 10;

        // A threshold when we consider a checkpoint reached
        float reach_threshold = 9.5f;

        int lookahead_index = Mathf.Min(currentNodeIdx + lookahead, nodePath.Count - 1);

        Vector3 current_direction = transform.forward;
        Vector3 direction_to_lookahead_index = current_position - nodePath[lookahead_index].GetGlobalPosition();
        float angle_between_vectors = Vector3.Angle(current_direction.normalized, direction_to_lookahead_index.normalized);

        reach_threshold *= angle_between_vectors / 180.0f;

        target_velocity = nextTargetPosition - targetPosition;

        Vector3 pos_error = targetPosition - current_position;
        Vector3 vel_error = target_velocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = (k_p * pos_error + k_d * vel_error).normalized; // normalize to receive steering and accel values between (-1, 1)

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        float distance = Vector3.Distance(targetPosition, current_position);
        if (distance < reach_threshold)
        {
            currentNodeIdx++;
            if (currentNodeIdx >= nodePath.Count) return;
            targetPosition = nodePath[currentNodeIdx].GetGlobalPosition();
        }

        for (int i = currentNodeIdx + 1; i < nodePath.Count; i++)
        {
            float current_tracked_distance = Vector3.Distance(targetPosition, current_position);
            float potential_tracked_distance = Vector3.Distance(nodePath[i].GetGlobalPosition(), current_position);
            
            if (current_tracked_distance > potential_tracked_distance)
            {
                // found a closer point, i, no need to keep looking
                currentNodeIdx = i;
                break; 
            }
        }
        
        // Debugging
        currentNodeIdx = Mathf.Min(currentNodeIdx, nodePath.Count - 1);
        Debug.DrawLine(transform.position, nodePath[currentNodeIdx].GetGlobalPosition(), Color.black);
        Debug.DrawLine(nodePath[currentNodeIdx].GetGlobalPosition() + new Vector3(0, 0.1f, 0), nodePath[lookahead_index].GetGlobalPosition() + new Vector3(0, 0.1f, 0), Color.blue);

        m_Car.Move(steering, acceleration, acceleration, 0f);
    }

    private void PidControllTowardsPosition()
    {
        AStarNode target = nodePath[currentNodeIdx];
        AStarNode nextTarget = nodePath[Math.Clamp(currentNodeIdx+1, 0, nodePath.Count-1)];
        Vector3 targetPosition = m_MapManager.grid.LocalToWorld(target.LocalPosition);
        Vector3 nextTargetPosition = m_MapManager.grid.LocalToWorld(nextTarget.LocalPosition);

        // Make target velocity lower in curves, where points are denser
        target_velocity = nextTargetPosition - targetPosition;

        // a PD-controller to get desired acceleration from errors in position and velocity
        Vector3 positionError = targetPosition - transform.position;
        Vector3 velocityError = target_velocity - my_rigidbody.velocity;
        
        Vector3 proportional = k_p * positionError;
        integral += Time.fixedDeltaTime * positionError.magnitude;
        Vector3 integralTerm = k_i * integral * positionError.normalized;
        Vector3 derivativeTerm = k_d * velocityError;

        Vector3 desired_acceleration = proportional + integralTerm + derivativeTerm;

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);            

        Debug.DrawLine(targetPosition, targetPosition + target_velocity, Color.red, Time.deltaTime * 2);
        Debug.DrawLine(my_rigidbody.position, my_rigidbody.position + my_rigidbody.velocity, Color.blue);
        Debug.DrawLine(targetPosition, targetPosition + desired_acceleration, Color.black);            

        m_Car.Move(steering, acceleration, acceleration, 0f);
    }


    void OnDrawGizmos()
    {
        if (drawDebug && pathFinder != null && pathFinder.flowField != null)
        {
            foreach (var posEntity in pathFinder.flowField)
            {
                var cell = new Vector3Int(posEntity.Key.x, posEntity.Key.y, 1);
                var cellGlobal = m_MapManager.grid.CellToWorld(cell);

                var gizmoSize = m_MapManager.grid.cellSize;
                gizmoSize.y = 0.005f;
                gizmoSize.Scale(m_MapManager.grid.transform.localScale * 0.8f);
                
                Gizmos.color = Mathf.CorrelatedColorTemperatureToRGB(1000f + 1000f * posEntity.Value);
                Gizmos.DrawCube(cellGlobal, gizmoSize);
            }
        }
    }
}