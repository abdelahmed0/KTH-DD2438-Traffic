﻿using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Map;
using Scripts.Vehicle;
using UnityEngine;

using System.Linq;

using System.Collections.Generic;

[RequireComponent(typeof(DroneController))]
public class AIP4FormationDrone : MonoBehaviour
{
    private DroneController m_Drone;
    
    private Vector3 targetVelocity;
    public float k_p = 2f;
    public float k_d = 3f;
    private float gatePaddingMagnitude = 3f;
    public float followThrough = 1.5f;
    
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    
    private BoxCollider m_Collider;

    private int goalIdx = 0;

    private static bool StaticInitDone = false;

    private int currentNodeIdx;

    private Rigidbody my_rigidbody;

    private Vector3[,] pathSet;
    private static int currPathIdx = 1;

    private List<Vector3> path = new List<Vector3>();
    private float pointFrequency = 8;

    private bool isLeader = false;
    private static Vector3 leaderPosition;

    private Vector3 relativePosition;

    private int leaderIdx = 4;

    private List<Vector3> gatesList = new List<Vector3>();
    private static List<Vector3> leaderGatesList = new List<Vector3>();
    private static int gateIdx = 0;

    private static bool[] update;

    private float ghostLeader = 8f;

    private Vector3 tgt = new Vector3(0, 0, 0);
    
    private Vector3 height = new Vector3(0, 1, 0);

    private Vector3 ghostPos = new Vector3(0, 0, 0);


    private void Start()
    {
        Debug.Log("Entered 1");
        m_Drone = GetComponent<DroneController>();
        
        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap; //Is not a MonoBehavior, so cannot fetch in the same fashion!
        
        my_rigidbody = GetComponent<Rigidbody>();
        
        // For finding individual gates
        var gateGroup = m_MapManager.GetTargetObjects();
        // gateGroup = GameObject.FindGameObjectsWithTag("Target").ToList();
        var individualGates = GameObject.FindGameObjectsWithTag("SubTarget").ToList(); // Gives all individual gates on map... but this might not be so useful

        int numGates = 0;
        
        Debug.Log("Entered 2");
        
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
        
        Debug.Log("Entered 3");
        
        update = new bool[numGates];
        for (int i = 0; i < update.Length; i++)
        {
            update[i] = false;
        }
        
        Transform[] gates = new Transform[numGates];

        int ctr = 0;

        foreach (Transform child in gateGroup[0].transform)
        {
            gates[ctr] = child;
            ctr++;
        }
        
        Debug.Log("Entered 4");
        
        // Get target gate for current car based on distance to gate
        int curr = 0;
        float val = Vector3.Distance(transform.position, gates[0].position);
        for (int i = 1; i < gates.Length; i++)
        {
            float temp = Vector3.Distance(transform.position, gates[i].position);
            if (temp < val)
            {
                curr = i;
                val = temp;
            }
        }
        goalIdx = curr;
        
        Debug.Log("Entered 5");
        
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
        
        Debug.Log("Initialized");
        
        // Move target vector to center of gate
        Quaternion rotation = gateGroup[0].transform.rotation * Quaternion.Euler(0, 180f, 0);
        Vector3 padding = new Vector3(gatePaddingMagnitude, 0, 0);
        padding = rotation * padding;
        // Debug.DrawLine(pathSet[0, 1], pathSet[0, 1] + padding, Color.magenta, 100f);
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
            
            // TODO: Temporary Fix
            if (i >= 6)
            {
                pathSet[i, 1] = gateGroup[i].transform.GetChild(Mathf.Abs(goalIdx - 4)).position;
            }
            else
            {
                pathSet[i, 1] = gateGroup[i].transform.GetChild(goalIdx).position;
            }
            
            // Adjust vector to point to center of gate
            rotation = gateGroup[i].transform.rotation * Quaternion.Euler(0, 180f, 0);
            padding = new Vector3(gatePaddingMagnitude, 0, 0);
            padding = rotation * padding;
            pathSet[i, 1] += padding;
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
    }

    private void FixedUpdate()
    {
        // Update leader position
        if (isLeader)
        {
            leaderPosition = transform.position;
        }

        if (update[goalIdx])
        {
            relativePosition = gatesList[gateIdx] - leaderGatesList[gateIdx];
            update[goalIdx] = false;
        }

        Vector3 targetPosition = path[currPathIdx];
        Vector3 padding = new Vector3(0, 0, 0);
        if (!isLeader)
        {

            Vector3 point = leaderPosition + relativePosition;
            Vector3 linePoint;
            Vector3 lineDir;
            if (gateIdx == 0)
            {
                linePoint = pathSet[0, 0];
                lineDir = gatesList[gateIdx] - linePoint;
            }
            else
            {
                linePoint = gatesList[gateIdx - 1];
                lineDir = gatesList[gateIdx] - linePoint;
            }

            lineDir.Normalize();
            Vector3 v = point - linePoint;
            float d = Vector3.Dot(v, lineDir);
            Vector3 closest = linePoint + (lineDir * d);

            targetPosition = closest;

            tgt = targetPosition;
            
            padding = Vector3.Normalize(lineDir) * ghostLeader;

            ghostPos = targetPosition + padding;
        }

        // Vector3 targetPosition = path[currPathIdx];
        
        
        PdControllSimple(targetPosition, 5f, padding);
        
        // PdControll(targetPosition, targetVelocity);
        
    }

    private void PdControllSimple(Vector3 targetPosition, float velocity, Vector3 padding)
    {
        Vector3 currentPos = transform.position;

        Vector3 paddedPos = targetPosition + padding;

        // Vector3 ghostTarget = targetPosition + (Vector3.Normalize(targetPosition - transform.position) * followThrough);
        // Vector3 ghostTarget = targetPosition;

        Vector3 tv = Vector3.Normalize(paddedPos - transform.position) * velocity;
        
        Vector3 posError = paddedPos - currentPos;
        Vector3 velError = tv - my_rigidbody.velocity;
        Vector3 desiredAccel = (k_p * posError + k_d * velError).normalized;

        float steering = Vector3.Dot(desiredAccel, transform.right);
        float acceleration = Vector3.Dot(desiredAccel, transform.forward);

        // if (Vector3.Angle(transform.forward, targetPosition) > 90)
        // {
        //     // acceleration = 0;
        // }
        
        float threshold = 3f;
        float gateThreshold = 1f;
        float distance = Vector3.Distance(paddedPos, currentPos);
        float gateDistance = Vector3.Distance(gatesList[gateIdx], currentPos);

        if (isLeader && distance <= threshold)
        {
            currPathIdx++;
        }

        if (isLeader && gateDistance <= gateThreshold)
        {
            gateIdx++;
            for (int i = 0; i < update.Length; i++)
            {
                if (i != leaderIdx)
                    update[i] = true;
            }
        }

        m_Drone.Move(steering, acceleration);
    }
    
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

        if (pathSet == null)
        {
            // Debug.Log("Not Ready Yet");
            return;
        }
        
        // Draw car paths
        for (int i = 0; i < pathSet.GetLength(0); i++)
        {
            // Debug.DrawLine(pathSet[i, 0] + height, pathSet[i, 1] + height, Color.green, 100f);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(pathSet[i, 0] + height, pathSet[i, 1] + height);
        }
        
        // Draw blue spheres at target points
        if (isLeader)
        {
            foreach (var point in path)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawSphere(point + height, 0.5f);
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
        if (!isLeader)
        {
            // Gizmos.color = Color.black;
            // Gizmos.DrawLine(leaderPosition + height, leaderPosition + relativePosition + height);
            Gizmos.color = Color.magenta;
            // Gizmos.DrawSphere(leaderPosition + relativePosition + height, 0.5f);
            Gizmos.DrawSphere(tgt + height, 0.5f);
            
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position + height, tgt + height);
            
            Gizmos.color = Color.white;
            Gizmos.DrawSphere(ghostPos + height, 0.5f);
            
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position + height, ghostPos + height);
        }
    }
}