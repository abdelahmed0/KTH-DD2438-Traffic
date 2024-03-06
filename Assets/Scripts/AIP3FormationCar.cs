using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Map;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class AIP3FormationCar : MonoBehaviour
{
    private CarController m_Car; 
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;

    private void Start()
    {
        m_Car = GetComponent<CarController>();
        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap; //Is not a MonoBehavior, so cannot fetch in the same fashion!
        
        
        // For finding individual gates
        var gateGroup = m_MapManager.GetTargetObjects(); // Gives all gate groups on map, fixed order
        var gateGroupTag = GameObject.FindGameObjectsWithTag("Target").ToList(); // Gives all gate groups on map, but order is random.
        var individualGates = GameObject.FindGameObjectsWithTag("SubTarget").ToList(); // Gives all individual gates on map... but this might not be so useful

        foreach (var group in gateGroup)
        {
            foreach (Transform child in group.transform) // I know this looks odd, but Transform implements Enumerable. When you iterate it, you iterate it's children.
            {
                // Should iterate over child gates left to right. 
                // 
            }
        }
        
        
        // See AIP1TrafficCar.cs for other examples
    }


    private void FixedUpdate()
    {
    }
}