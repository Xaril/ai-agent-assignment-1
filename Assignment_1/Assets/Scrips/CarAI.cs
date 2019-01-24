﻿using System.Collections;
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

        private List<Vector3> aStarPath;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public bool visualize = false;

        public ConfigurationSpace configurationSpace;

        private void Awake()
        {
            Time.timeScale = 2;
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            maxVelocity = 3;
            acceleration = 0.2f;

            InitializeCSpace();

            timeStep = 0.05f;
            numberOfSteps = 5;
            time = 0;

            steerDirection = 0;
            accelerationDirection = 0;

            carLength = FindCarLength();

            start = new TreePoint(terrain_manager.myInfo.start_pos, m_Car.transform.eulerAngles.y, 0);
            path = new List<TreePoint>();
            pathIndex = 1;

            aStarPath = AStar();

            path = RRT();

            if(path == null)
            {
                Debug.Log("No path found!");
            }
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
            if(visualize)
            {
                VisualizeCSpace();
            }
        }

        public List<TreePoint> RRT()
        {
            bool foundGoal = false;
            TreePoint pathPoint = null;
            for(int i = 0; i < 25000; i++)
            {
                Vector3 randomPoint;
                do
                {
                    randomPoint = GetRandomPoint();
                }
                while (configurationSpace.Collision(randomPoint.x, randomPoint.z, 0));

                TreePoint nearPoint = NearestNeighbour(randomPoint);
                TreePoint newPoint = SimulateMovement(nearPoint, randomPoint);
                if(newPoint != null)
                {
                    newPoint.parent = nearPoint;
                    nearPoint.children.Add(newPoint);

                    if (Vector3.Distance(newPoint.position, terrain_manager.myInfo.goal_pos) <= 5)
                    {
                        foundGoal = true;
                        pathPoint = newPoint;
                        break;
                    }
                }
            }

            if(foundGoal)
            {
                List<TreePoint> goalPath = new List<TreePoint>();
                goalPath.Add(pathPoint);
                while(pathPoint.parent != null)
                {
                    //Visualize the path
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.position = new Vector3(pathPoint.position.x, 0.0f, pathPoint.position.z);
                    cube.transform.localScale = new Vector3(0.5f, 2, 0.5f);
                    cube.transform.eulerAngles = new Vector3(0, pathPoint.theta, 0);
                    cube.GetComponent<BoxCollider>().enabled = false;

                    goalPath.Insert(0, pathPoint.parent);
                    pathPoint = pathPoint.parent;
                }
                return goalPath;
            }
            return null;
        }

        public Vector3 GetRandomPoint()
        {
            float probability = UnityEngine.Random.Range(0, 1);
            if(probability < 0.5f)
            {
                return new Vector3(
                    UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high),
                    0,
                    UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high)
                );
            }
            else
            {
                return new Vector3(
                    UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high),
                    0,
                    UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high)
                );
            }
        }

        //Finds the nearest tree point to a point.
        public TreePoint NearestNeighbour(Vector3 point)
        {
            Queue<TreePoint> queue = new Queue<TreePoint>();
            queue.Enqueue(start);
            float minDistance = Mathf.Infinity;
            TreePoint nearestPoint = null;
            while(queue.Count > 0)
            {
                TreePoint currentPoint = queue.Dequeue();
                foreach(TreePoint child in currentPoint.children)
                {
                    queue.Enqueue(child);
                }
                float distance = MeasureDistance(currentPoint.position, point);
                if(distance < minDistance)
                {
                    minDistance = distance;
                    nearestPoint = currentPoint;
                }
            }
            return nearestPoint;
        }

        private float MeasureDistance(Vector3 a, Vector3 b)
        {
            return Vector3.Distance(a, b);
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
                if(AccelerationInput(position, theta, to) < 0)
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

        //Calculates the shortest discrete path  
        private List<Vector3> AStar()
        {
            Dictionary<Vector3, Vector3> cameFrom = new Dictionary<Vector3, Vector3>();
            float[,] gScore = new float[terrain_manager.myInfo.x_N,terrain_manager.myInfo.z_N];
            float[,] fScore = new float[terrain_manager.myInfo.x_N, terrain_manager.myInfo.z_N];


            List<Vector3> open = new List<Vector3>();
            List<Vector3> closed = new List<Vector3>();
            Vector3 startCell = new Vector3(
                terrain_manager.myInfo.get_x_pos(terrain_manager.myInfo.get_i_index(terrain_manager.myInfo.start_pos.x)),
                0,
                terrain_manager.myInfo.get_z_pos(terrain_manager.myInfo.get_j_index(terrain_manager.myInfo.start_pos.z))
            );
            open.Add(startCell);

            Vector3 goalCell = new Vector3(
                terrain_manager.myInfo.get_x_pos(terrain_manager.myInfo.get_i_index(terrain_manager.myInfo.goal_pos.x)),
                0,
                terrain_manager.myInfo.get_z_pos(terrain_manager.myInfo.get_j_index(terrain_manager.myInfo.goal_pos.z))
            );

            gScore[terrain_manager.myInfo.get_i_index(terrain_manager.myInfo.start_pos.x), 
                   terrain_manager.myInfo.get_j_index(terrain_manager.myInfo.start_pos.z)] = 0;
            fScore[terrain_manager.myInfo.get_i_index(terrain_manager.myInfo.start_pos.x), 
                   terrain_manager.myInfo.get_j_index(terrain_manager.myInfo.start_pos.z)] = Vector3.Distance(startCell, goalCell);

            while (open.Count > 0)
            {
                Vector3 current = Vector3.zero;
                float weight = Mathf.Infinity;
                foreach(Vector3 node in open)
                {
                    int nodeI = terrain_manager.myInfo.get_i_index(node.x);
                    int nodeJ = terrain_manager.myInfo.get_j_index(node.z);
                    if (fScore[nodeI, nodeJ] < weight)
                    {
                        weight = fScore[nodeI, nodeJ];
                        current = node;
                    }
                }

                if (current == goalCell)
                {
                    List<Vector3> aPath = new List<Vector3>();
                    aPath.Add(current);
                    while(true)
                    {
                        bool foundKey = false;
                        foreach (Vector3 key in cameFrom.Keys)
                        {
                            if (current == key)
                            {
                                foundKey = true;
                                current = cameFrom[key];
                                aPath.Insert(0, current);
                            }
                        }
                        if(!foundKey)
                        {
                            break;
                        }
                    }
                    return aPath;
                }

                open.Remove(current);
                closed.Add(current);

                int currentI = terrain_manager.myInfo.get_i_index(current.x);
                int currentJ = terrain_manager.myInfo.get_j_index(current.z);
                for(int i = Mathf.Max(currentI - 1, 0); i <= Mathf.Min(currentI + 1, terrain_manager.myInfo.x_N-1); ++i)
                {
                    for (int j = Mathf.Max(currentJ - 1, 0); j <= Mathf.Min(currentJ + 1, terrain_manager.myInfo.z_N-1); ++j)
                    {
                        Vector3 neighbour = new Vector3(
                            terrain_manager.myInfo.get_x_pos(i),
                            0,
                            terrain_manager.myInfo.get_z_pos(j)
                        );
                        bool alreadyClosed = false;
                        foreach(Vector3 node in closed)
                        {
                            if(node == neighbour)
                            {
                                alreadyClosed = true;
                                break;
                            }
                        }
                        if(alreadyClosed)
                        {

                            continue;
                        }

                        float distance = Vector3.Distance(current, neighbour);
                        if(terrain_manager.myInfo.traversability[i, j] > 0.5f)
                        {
                            distance = Mathf.Infinity;
                        }
                        float tentative_gScore = gScore[currentI, currentJ] + distance;

                        alreadyClosed = false;
                        foreach (Vector3 node in open)
                        {
                            if (node == neighbour)
                            {
                                alreadyClosed = true;
                                break;
                            }
                        }
                        if (!alreadyClosed)
                        {
                            open.Add(neighbour);
                        }
                        else if(tentative_gScore >= gScore[i, j])
                        {
                            continue;
                        }

                        bool foundKey = false;
                        foreach(Vector3 key in cameFrom.Keys)
                        {
                            if(key == neighbour)
                            {
                                cameFrom[key] = current;
                                foundKey = true;
                                break;
                            }
                        }
                        if(!foundKey)
                        {
                            cameFrom[neighbour] = current;
                        }
                        gScore[i, j] = tentative_gScore;
                        fScore[i, j] = gScore[i, j] + Vector3.Distance(neighbour, goalCell);
                    }
                }
            }
            return null;
        }

        //Determines delta for the kinematic motion model
        private float SteerInput(Vector3 position, float theta, Vector3 point)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToPoint = point - position;
            float angle = Vector3.Angle(direction, directionToPoint) * Mathf.Sign(-direction.x * directionToPoint.z + direction.z * directionToPoint.x);
            float steerAngle = Mathf.Clamp(angle, -25, 25) / 25;
            return steerAngle;
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
