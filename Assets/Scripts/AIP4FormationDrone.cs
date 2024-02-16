using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Map;
using Scripts.Vehicle;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class AIP4FormationDrone : MonoBehaviour
{
    private DroneController m_Drone;


    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        
        
        // See AIP2TrafficDrone.cs for more examples
    }


    private void FixedUpdate()
    {
    }
}