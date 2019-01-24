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

        private float timeStep;
        private int numberOfSteps;
        private float time;

        private float steerDirection;
        private float accelerationDirection;

        private float carLength;

        private TreePoint start;
        private List<TreePoint> path;
        private TreePoint nextPoint;
        private int pathIndex;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public ConfigurationSpace configurationSpace;

        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            maxVelocity = 3;
            acceleration = 1f;

            InitializeCSpace();

            timeStep = 0.1f;
            numberOfSteps = 15;
            time = 0;

            steerDirection = 0;
            accelerationDirection = 0;

            carLength = FindCarLength();

            start = new TreePoint(terrain_manager.myInfo.start_pos, m_Car.transform.eulerAngles.y, 0);
            path = new List<TreePoint>();
            pathIndex = 1;

            //<RRT>
            Vector3 randomPoint;
            
            do
            {
                randomPoint = new Vector3(
                    UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high),
                    0,
                    UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high)
                );
            }
            while (configurationSpace.Collision(randomPoint.x, randomPoint.z, 0));

            TreePoint newPoint = SimulateMovement(start, randomPoint);
            //</RRT>

            nextPoint = path[pathIndex];
        }


        private void FixedUpdate()
        {
            time += Time.deltaTime;
            if(time >= timeStep)
            {
                time = 0;
                steerDirection = SteerInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, nextPoint.position);
                accelerationDirection = AccelerationInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, nextPoint.position);
            }
            if (m_Car.CurrentSpeed >= maxVelocity)
            {
                if (accelerationDirection < 0)
                {
                    m_Car.Move(-steerDirection, 0f, 0f, 0f);
                }
                else
                {
                    m_Car.Move(steerDirection, 0f, 0f, 0f);
                }
            }
            else
            {
                if(accelerationDirection < 0)
                {
                    m_Car.Move(-steerDirection, 0f, accelerationDirection * acceleration, 0f);
                }
                else
                {
                    m_Car.Move(steerDirection, accelerationDirection * acceleration, 0f, 0f);
                }
            }
            //Update point if close enough to current one
            if(Vector3.Distance(m_Car.transform.position, nextPoint.position) <= 1)
            {
                pathIndex = Mathf.Min(pathIndex + 1, path.Count-1);
                nextPoint = path[pathIndex];
            }
        }

        //Simulate movement from a point in the tree to a point in the plane.
        public TreePoint SimulateMovement(TreePoint from, Vector3 to)
        {
            Vector3 position = from.position;
            float theta = from.theta;
            float velocity = from.velocity;
            for(int step = 0; step < numberOfSteps; step++)
            {
                //Get steering angle
                float delta = m_Car.m_MaximumSteerAngle * SteerInput(position, theta, to);
                if(velocity < 0)
                {
                    delta = -delta;
                }
                if(Mathf.Abs(delta) <= 5)
                {
                    delta = 0;
                }

                //Calculate motion model values according to kinematic car model
                float xDiff = velocity * Mathf.Sin(Mathf.Deg2Rad * theta);
                float zDiff = velocity * Mathf.Cos(Mathf.Deg2Rad * theta);
                float thetaDiff = velocity / carLength * Mathf.Tan(Mathf.Deg2Rad * delta) * Mathf.Rad2Deg;

                //Get new position and orientation using Euler's method
                position = new Vector3(Euler(position.x, xDiff, timeStep), 0, Euler(position.z, zDiff, timeStep));
                theta = Euler(theta, thetaDiff, timeStep);

                //If collision, this point is not traversable
                if(configurationSpace.Collision(position.x, position.z, theta))
                {
                    return null;
                }

                //If close enough to end position, stop iterating
                if(Vector3.Distance(position, to) <= 1)
                {
                    break;
                }

                velocity += Mathf.Clamp(AccelerationInput(position, theta, to) * acceleration * timeStep, -maxVelocity, maxVelocity);
            }

            TreePoint result = new TreePoint(position, theta, velocity);
            return result;
        }

        //Extrapolates the value in the next position given the derivative of the value
        float Euler(float value, float difference, float step)
        {
            return value + difference * step;
        }

        //Determines delta for the kinematic motion model
        private float SteerInput(Vector3 position, float theta, Vector3 point)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToPoint = point - position;
            return Mathf.Clamp(-direction.x * directionToPoint.z + direction.z * directionToPoint.x, -1, 1);
        }

        //Determines acceleration which is used to determine v for the kinematic motion model
        private float AccelerationInput(Vector3 position, float theta, Vector3 point)
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

        private float FindCarLength()
        {
            WheelCollider[] wheels = FindObjectsOfType<WheelCollider>();
            float maxValue = 0;
            float midValue = 0;
            for (int i = 1; i < wheels.Length; ++i)
            {
                if ((wheels[0].transform.position - wheels[i].transform.position).magnitude > maxValue)
                {
                    midValue = maxValue;
                    maxValue = (wheels[0].transform.position - wheels[i].transform.position).magnitude;
                }
            }
            return midValue;
        }
    }
}
