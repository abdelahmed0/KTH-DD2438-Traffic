using System;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using Scripts.Map;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
// using UnityStandardAssets.Vehicles.Car.Map;
using static UnityEditor.PlayerSettings;


public class OurCollisionMap : MonoBehaviour
{
    /// <summary>
    /// Our own implementation of the collision map.
    /// Improves the original by allowing custom collision shapes,
    /// height at which the collision is checked and more
    /// drawing features for debugging.
    /// </summary>
    [Header("Collision Box Parameters")]
    [SerializeField]
    private Vector3 collisionBoxSize;
    [SerializeField]
    private Vector3 minCollisionBoxSize;
    [Tooltip("Only do rotations around Y-axis")]
    [SerializeField]
    private Quaternion[] rotations; 
    [SerializeField]
    private float heightOffGround;

    [Header("Collision Map Parameters")]
    [SerializeField]
    private Vector2 cellSize;

    [Header("Draw Map Options")]
    [Tooltip("Draws colored cells")]
    [SerializeField]
    private bool drawCells;
    [Tooltip("Draws colored outlines")]
    [SerializeField]
    private bool drawCellOutline;
    [Tooltip("Draws colored collision boxes at each cell")]
    [SerializeField]
    private bool drawCollisionMesh;
    [Tooltip("Writes each cell's index")]
    [SerializeField]
    private bool drawLabels;

    // Singleton instance of object
    public static OurCollisionMap Instance { get; private set; }

    private Vector3 worldOrigin;                                // Origin of the map in world coodrinates
    public List<GameObject> obstacles { get; private set; }     // All the obstacles in the map
    private Dictionary<Quaternion, Vector3[]> debugMeshCache;   // Cache to make drawing collision meshes faster

    private State[,] collisionMap;                              // The collision map

    private void Awake()
    {
        // If there is an instance, and it's not me, delete myself.

        if (Instance != null && Instance != this)
            Destroy(this);
        else
            Instance = this;
    }

    private void Start()
    {
        CreateMap();  
    }

    public bool LowerColBoxSize()
    {
        collisionBoxSize = new Vector3(collisionBoxSize.x - 0.25f, collisionBoxSize.y, collisionBoxSize.z - 0.25f);

        if (collisionBoxSize.x < minCollisionBoxSize.x || collisionBoxSize.z < minCollisionBoxSize.z)
            return false;

        CreateMap();

        return true;    
    }

    public void ResetMap() // Only used for UI atm
    {
        collisionMap = null;
        debugMeshCache = new Dictionary<Quaternion, Vector3[]>();
        OurCollisionMap.Instance = null;
    }

    public Vector3 CellToWorld(Vector2Int cell, float height = 0)
    {
        /// Returns the center of the cell in world coordinates
        Vector3 ret = new Vector3(cell.x * cellSize.x + cellSize.x / 2 + worldOrigin.x, height, cell.y * cellSize.y + cellSize.y / 2 + worldOrigin.z);

        return ret;
    }

    public Vector2Int WorldToCell(Vector3 pos)
    {
        /// Returns the cell index for world coordinate
        Vector2Int ret = new Vector2Int(Mathf.FloorToInt((pos.x - worldOrigin.x) / cellSize.x), 
                                        Mathf.FloorToInt((pos.z - worldOrigin.z) / cellSize.y)
                                        );

        return ret;
    }

    public State GetCellState(Vector2Int pos)
    {
        /// Returns the state of given cell
        return collisionMap[pos.x, pos.y];
    }

    public Vector2Int GetMapSize()
    {
        return new Vector2Int(collisionMap.GetLength(0), collisionMap.GetLength(1));
    }

    public Vector2 GetCellSize()
    {
        return cellSize;
    }

    public Vector3 GetCollisionBoxSize()
    {
        return collisionBoxSize;
    }

    private void CreateMap()
    {
        /// Creates the collision map
        var mapManager = FindObjectOfType<MapManager>();
        var obstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        var obstacleMap = obstacleMapManager.ObstacleMap;
        print(obstacleMap);
        print(obstacleMap.obstacleObjects);
        obstacles = obstacleMap.obstacleObjects;
        
        worldOrigin = mapManager.grid.LocalToWorld(new Vector3(obstacleMap.cellBounds.xMin, 0, obstacleMap.cellBounds.zMin));
        Vector3 cellScale = mapManager.grid.transform.localScale; // Their grid scale, rescale to use our scale so it is the same for all maps
        Vector2Int mapSize = new Vector2Int(Mathf.FloorToInt((obstacleMap.cellBounds.xMax - obstacleMap.cellBounds.xMin) / cellSize.x * cellScale.x),
                                            Mathf.FloorToInt((obstacleMap.cellBounds.zMax - obstacleMap.cellBounds.zMin) / cellSize.y * cellScale.z)
                            );


        collisionMap = new State[mapSize.x, mapSize.y];
        debugMeshCache = new Dictionary<Quaternion, Vector3[]>();

        FindObsticles();
    }

    private void FindObsticles()
    {
        /// Casts a collision box at each cell and checks if it collided with an obsticle
        for (int i = 0; i < collisionMap.GetLength(0); i++)
        {
            for (int j = 0; j < collisionMap.GetLength(1); j++)
            {
                foreach (var rot in rotations)
                {
                    Vector3 position = CellToWorld(new Vector2Int(i, j), collisionBoxSize.y/2 + heightOffGround); 
                    var collisions = Physics.OverlapBox(position, collisionBoxSize/2, rot);

                    foreach (var collision in collisions)
                    {
                        if (obstacles.Contains(collision.gameObject))
                            collisionMap[i, j] = State.Blocked;
                    }
                }
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (OurCollisionMap.Instance == null)
        {
            Awake();
            CreateMap();
        }

        /// Used to draw the collision map
        if (drawCells || drawCellOutline || drawLabels || drawCollisionMesh)
        {
            if (collisionMap == null)
                CreateMap();

            GUIStyle style = new GUIStyle();
            style.normal.textColor = Color.black;

            for (int x = 0; x < collisionMap.GetLength(0); x++)
            {
                for (int y = 0; y < collisionMap.GetLength(1); y++)
                {
                    var position = CellToWorld(new Vector2Int(x,y), 20f);
                    var gizmoSize = new Vector3(cellSize.x * 0.9f, 0.005f, cellSize.y * 0.9f);

                    switch (collisionMap[x,y])
                    {
                        case State.Blocked:
                            Gizmos.color = Color.red; 
                            break;
                        case State.Free:
                            Gizmos.color = Color.green;
                            break;
                        default:
                            Gizmos.color = Color.blue;
                            break;
                    }

                    if (drawCells)
                        Gizmos.DrawCube(position, gizmoSize);

                    if (drawCellOutline)
                        Gizmos.DrawWireCube(position, gizmoSize);

                    if (drawLabels)
                        Handles.Label(position, x + ", " + y, style);

                    if (drawCollisionMesh)
                    {
                        var center = CellToWorld(new Vector2Int(x, y), collisionBoxSize.y / 2 + heightOffGround);
                        foreach (var rot in rotations)
                        {
                            DrawDebugBox(center, rot, collisionBoxSize, Gizmos.color);
                        }
                    }
                }
            }
        }
    }

    private void DrawDebugBox(Vector3 pos, Quaternion rot, Vector3 scale, Color c)
    {
        /// Used to draw an outline of the collision box used in collision checking (needed to do rotations)
        // Credit: https://gist.github.com/unitycoder/58f4b5d80f423d29e35c814a9556f9d9
        Vector3 point1, point2, point3, point4, point5, point6, point7, point8;
        if (!debugMeshCache.ContainsKey(rot))
        {
            Matrix4x4 m = new Matrix4x4();
            m.SetTRS(Vector3.zero, rot, scale);

            point1 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, 0.5f));
            point2 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, 0.5f));
            point3 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, -0.5f));
            point4 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, -0.5f));

            point5 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, 0.5f));
            point6 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, 0.5f));
            point7 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, -0.5f));
            point8 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, -0.5f));

            Vector3[] temp = { point1, point2, point3, point4, point5, point6, point7, point8 };
            debugMeshCache.Add(rot, temp);
        } 
        else
        {
            point1 = debugMeshCache[rot][0];
            point2 = debugMeshCache[rot][1];
            point3 = debugMeshCache[rot][2];
            point4 = debugMeshCache[rot][3];

            point5 = debugMeshCache[rot][4];
            point6 = debugMeshCache[rot][5];
            point7 = debugMeshCache[rot][6];
            point8 = debugMeshCache[rot][7];
        }

        Gizmos.DrawLine(point1 + pos, point2 + pos);
        Gizmos.DrawLine(point2 + pos, point3 + pos);
        Gizmos.DrawLine(point3 + pos, point4 + pos);
        Gizmos.DrawLine(point4 + pos, point1 + pos);

        Gizmos.DrawLine(point5 + pos, point6 + pos);
        Gizmos.DrawLine(point6 + pos, point7 + pos);
        Gizmos.DrawLine(point7 + pos, point8 + pos);
        Gizmos.DrawLine(point8 + pos, point5 + pos);

        Gizmos.DrawLine(point1 + pos, point5 + pos);
        Gizmos.DrawLine(point2 + pos, point6 + pos);
        Gizmos.DrawLine(point3 + pos, point7 + pos);
        Gizmos.DrawLine(point4 + pos, point8 + pos);
    }

    public enum State
    {
        Free = 0,
        Blocked = 1,
    }
}
