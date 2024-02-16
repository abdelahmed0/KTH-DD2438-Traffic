using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : MonoBehaviour
{
    private CarController m_Car; // the car controller we want to use
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private LineOfSightGoal m_CurrentGoal;

    public float steering;
    public float acceleration;

    private void Start()
    {
        m_Car = GetComponent<CarController>();
        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap; //Is not a MonoBehavior, so cannot fetch in the same fashion!
        m_CurrentGoal = (LineOfSightGoal)FindObjectOfType<GameManagerA2>().vehicleToGoalMapping[gameObject]; //This car's goal.

        m_OtherCars = GameObject.FindGameObjectsWithTag("Player");
        // Note that this array will have "holes" when objects are destroyed
        // Will work for initial planning they should work
        // If you dont like the "holes", you can re-fetch this during fixed update.

        // Where to go?
        var targetPosition = m_CurrentGoal.targetPosition; //World Coordinates.

        // Equivalent ways to find all the targets in the scene
        var targetObjects = m_MapManager.GetTargetObjects();
        targetObjects = GameObject.FindGameObjectsWithTag("Target").ToList();

        // Equivalent ways of finding the start positions
        var startObjects = m_MapManager.GetStartObjects();
        startObjects = GameObject.FindGameObjectsWithTag("Start").ToList();

        // You can also fetch other types of objects using tags, assuming the objects you are looking for HAVE tags :).

        // Plan your path here
        // ...

        // Easy way to find all position vectors is either "Keys" in above dictionary or:
        // Iterates over LOCAL positions in the map. I.e (x, 0, z)
        foreach (var localPosition in m_ObstacleMap.localBounds.allPositionsWithin)
        {
            m_ObstacleMap.IsLocalPointTraversable(localPosition);
            m_ObstacleMap.mapGrid.LocalToCell(localPosition); // Gives cell position
            m_ObstacleMap.mapGrid.LocalToWorld(localPosition); //Gives world position
        }

        // Iterates over CELL positions in the map.  "(x, z, 0)" or (x, y) in 2nd coordinates
        foreach (var cellPosition in m_ObstacleMap.cellBounds.allPositionsWithin)
        {
            m_ObstacleMap.IsCellTraversable(new Vector2Int(cellPosition.x, cellPosition.y));
            m_ObstacleMap.mapGrid.CellToLocal(cellPosition); // Gives cell position
            m_ObstacleMap.mapGrid.CellToWorld(cellPosition); // Gives world position
        }

        m_ObstacleMap.IsGlobalPointTraversable(transform.position);
        // Remember transform.localPosition only gives a map coordinate if car is inside Grid. Normally we dont spawn it that way.
        // Better to use this if you need local coordinates.
        var carLocalPosition = m_ObstacleMap.mapGrid.WorldToLocal(transform.position);
        m_ObstacleMap.IsLocalPointTraversable(carLocalPosition);

        // Feel free to refer to any examples from previous assignments.
    }


    private void FixedUpdate()
    {
        // Execute your path and collision checking here
        // ...

        // Feel free to refer to any examples from previous assignments.

        //Example of cars moving into the centre of the field.
        Vector3 avg_pos = m_OtherCars.Aggregate(Vector3.zero, (sum, car) => sum + car.transform.position) / m_OtherCars.Length;

        //var
        (steering, acceleration) = ControlsTowardsPoint(avg_pos);

        m_Car.Move(steering, acceleration, acceleration, 0f);
        

    }

    private (float steering, float acceleration) ControlsTowardsPoint(Vector3 avg_pos)
    {
        Vector3 direction = (avg_pos - transform.position).normalized;

        bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
        bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

        float steering = 0f;
        float acceleration = 0;

        if (is_to_the_right && is_to_the_front)
        {
            steering = 1f;
            acceleration = 1f;
        }
        else if (is_to_the_right && !is_to_the_front)
        {
            steering = -1f;
            acceleration = -1f;
        }
        else if (!is_to_the_right && is_to_the_front)
        {
            steering = -1f;
            acceleration = 1f;
        }
        else if (!is_to_the_right && !is_to_the_front)
        {
            steering = 1f;
            acceleration = -1f;
        }

        float alpha = Mathf.Asin(Vector3.Dot(direction, transform.right));
        if (is_to_the_front &&  Mathf.Abs(alpha) < 1f)
        {
            steering = alpha;
        }

        return (steering, acceleration);
    }
}