using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        public float maxVelocity;
        public float acceleration;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public ConfigurationSpace configurationSpace;

        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            maxVelocity = 3;
            acceleration = 0.2f;

            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            InitializeCSpace();
        }


        private void FixedUpdate()
        {

        }

        //Determines delta for the kinematic motion model
        private float steerInput(Vector3 position, float theta, Vector3 point)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToPoint = point - position;
            return Mathf.Clamp(-direction.x * directionToPoint.z + direction.z * directionToPoint.x, -1, 1);
        }

        //Determines acceleration which is used to determine v for the kinematic motion model
        private float accelerationInput(Vector3 position, float theta, Vector3 point)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToPoint = point - position;
            return Mathf.Clamp(direction.x * directionToPoint.x + direction.z * directionToPoint.z, -1, 1);
        }

        //Get size of car collider to be used with C space
        private void InitializeCSpace()
        {
            Quaternion carRotation = m_Car.transform.rotation;
            m_Car.transform.rotation = Quaternion.identity;
            configurationSpace = new ConfigurationSpace();
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            configurationSpace.BoxSize = carCollider.transform.TransformVector(carCollider.size);
            m_Car.transform.rotation = carRotation;
        }

        //Visualizes the configuration space by adding red boxes where the center point of the car cannot go.
        private void VisualizeCSpace()
        {
            GameObject[] cubes = GameObject.FindGameObjectsWithTag("config_cube");
            foreach (GameObject cube in cubes)
            {
                Destroy(cube);
            }


            int num_cubes = 10;
            float min_x = Mathf.Clamp(m_Car.transform.position.x - num_cubes, terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high);
            float max_x = Mathf.Clamp(m_Car.transform.position.x + num_cubes, terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high);
            float min_z = Mathf.Clamp(m_Car.transform.position.z - num_cubes, terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high);
            float max_z = Mathf.Clamp(m_Car.transform.position.z + num_cubes, terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high);
            int size = 1;

            for (int i = (int)min_x; i <= (int)max_x; i += size)
            {
                for (int j = (int)min_z; j <= (int)max_z; j += size)
                {
                    if (configurationSpace.Collision(i, j, m_Car.transform.eulerAngles.y))
                    {
                        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        cube.transform.position = new Vector3(i, 0.0f, j);
                        cube.transform.localScale = new Vector3(size, 2, size);
                        cube.GetComponent<BoxCollider>().enabled = false;
                        cube.tag = "config_cube";
                        cube.GetComponent<MeshRenderer>().material.color = Color.red;
                    }
                }
            }
        }
    }
}
