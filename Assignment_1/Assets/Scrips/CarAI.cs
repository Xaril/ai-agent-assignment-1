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

        private int RRTIterations;
        private float timeStep;
        private int numberOfSteps;
        private float time;

        private float steerDirection;
        private float accelerationDirection;

        private float carLength;

        private float world_width;
        private float world_height;
        private float gridSize;
        private int gridAmountX;
        private int gridAmountZ;
        private TreePoint start;
        private List<TreePoint>[,] treePoints;

        private float shortestPointToGoalDistance;
        private List<TreePoint> path;
        private TreePoint nextPoint;
        private int pathIndex;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public bool visualizeConfigurationSpace = false;

        public ConfigurationSpace configurationSpace;

        private void Awake()
        {
            Time.timeScale = 2;
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            maxVelocity = 6;
            acceleration = 0.2f;

            InitializeCSpace();

            RRTIterations = 500000;
            timeStep = 0.05f;
            numberOfSteps = 5;
            time = 0;

            steerDirection = 0;
            accelerationDirection = 0;

            carLength = FindCarLength();

            world_width = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            world_height = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            gridSize = Mathf.Min(
                world_width / terrain_manager.myInfo.x_N,
                world_height / terrain_manager.myInfo.z_N
            );
            gridAmountX = (int)(world_width / gridSize);
            gridAmountZ = (int)(world_height / gridSize);
            start = new TreePoint(terrain_manager.myInfo.start_pos, m_Car.transform.eulerAngles.y, 0);
            treePoints = new List<TreePoint>[gridAmountX, gridAmountZ];
            for (int i = 0; i < gridAmountX; ++i)
            {
                for (int j = 0; j < gridAmountZ; ++j)
                {
                    if (NearestNeighbourTraversable(i, j))
                    {
                        treePoints[i, j] = new List<TreePoint>();
                    }
                }
            }
            treePoints[NearestNeighbourGetIndexI(start.position.x), NearestNeighbourGetIndexJ(start.position.z)].Add(start);

            shortestPointToGoalDistance = Vector3.Distance(start.position, terrain_manager.myInfo.goal_pos);
            path = new List<TreePoint>();
            pathIndex = 1;

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
        }

        public List<TreePoint> RRT()
        {
            bool foundGoal = false;
            TreePoint pathPoint = null;
            for(int i = 0; i < RRTIterations; i++)
            {
                Vector3 randomPoint;
                do
                {
                    float probability = UnityEngine.Random.Range(0f, 1f);
                    if(probability < 0.1f)
                    {
                        Vector2 p = UnityEngine.Random.insideUnitCircle * shortestPointToGoalDistance;
                        randomPoint = new Vector3(
                            p.x,
                            0,
                            p.y
                        );
                    }
                    else
                    {
                        randomPoint = new Vector3(
                            UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high),
                            0,
                            UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high)
                        );
                    }

                }
                while (configurationSpace.Collision(randomPoint.x, randomPoint.z, 0));

                TreePoint nearPoint = NearestNeighbour(randomPoint);
                if(nearPoint == null)
                {
                    continue;
                }
                TreePoint newPoint = SimulateMovement(nearPoint, randomPoint);
                if(newPoint != null)
                {
                    newPoint.parent = nearPoint;
                    nearPoint.children.Add(newPoint);
                    int newIIndex = NearestNeighbourGetIndexI(newPoint.position.x);
                    int newJIndex = NearestNeighbourGetIndexJ(newPoint.position.z);
                    if(!NearestNeighbourTraversable(newIIndex, newJIndex))
                    {
                        continue;
                    }
                    treePoints[newIIndex, newJIndex].Add(newPoint);
                    if(Vector3.Distance(newPoint.position, terrain_manager.myInfo.goal_pos) < shortestPointToGoalDistance)
                    {
                        shortestPointToGoalDistance = Vector3.Distance(newPoint.position, terrain_manager.myInfo.goal_pos);
                    }

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
                List<TreePoint> goalPath = new List<TreePoint>{pathPoint};
                while (pathPoint.parent != null)
                {
                    goalPath.Insert(0, pathPoint.parent);
                    pathPoint = pathPoint.parent;
                }
                return goalPath;
            }
            return null;
        }

        //Finds the nearest tree point to a point.
        public TreePoint NearestNeighbour(Vector3 point)
        {
            //Keep track of the closest point
            float minDistance = Mathf.Infinity;
            TreePoint nearestPoint = null;

            int pointIIndex = NearestNeighbourGetIndexI(point.x);
            int pointJIndex = NearestNeighbourGetIndexJ(point.z);

            //Iterate over grid cells using a queue
            Queue<int[]> queue = new Queue<int[]>();
            //Keep track of cells already visited
            bool[,] visited = new bool[gridAmountX, gridAmountZ];

            int[] startCell = { pointIIndex, pointJIndex };

            //Since distance increases in a circle we get more cells every time.
            //To keep track of how many cells to check before trying to return a
            //value, we can use a stop block.
            int[] stopBlock = { -1, -1 };

            queue.Enqueue(startCell);
            queue.Enqueue(stopBlock);

            //This helps keep track of when to put stop blocks in the queue.
            int numberOfBlocks = 1;
            int currentBlock = 0;

            while(queue.Count > 0)
            {
                int[] cell = queue.Dequeue();

                //Stop block
                if(cell[0] == -1 && cell[1] == -1)
                {
                    currentBlock = 0;
                    numberOfBlocks = queue.Count - 1;

                    //If we have found a point on this level, it is the 
                    //nearest point so we return it
                    if(nearestPoint != null)
                    {
                        return nearestPoint;
                    }
                } 
                else if(!visited[cell[0],cell[1]]) //Haven't checked the block yet
                {
                    visited[cell[0], cell[1]] = true;

                    //No need to check a block that cannot be traversed
                    if(!NearestNeighbourTraversable(cell[0], cell[1]))
                    {
                        currentBlock++;
                        if(currentBlock == numberOfBlocks)
                        {
                            queue.Enqueue(stopBlock);
                        }
                    }
                    else
                    {
                        //Look at all points in the cell
                        foreach(TreePoint treePoint in treePoints[cell[0], cell[1]])
                        {
                            float distance = MeasureDistance(treePoint.position, point);
                            if(distance < minDistance)
                            {
                                minDistance = distance;
                                nearestPoint = treePoint;
                            }
                        }

                        //Haven't found a point in this cell, enqueue the enclosing ones
                        if (nearestPoint == null)
                        {
                            currentBlock++;
                            if(cell[0] - 1 >= 0)
                            {
                                queue.Enqueue(new int[] { cell[0] - 1, cell[1] });
                            }
                            if(cell[0] + 1 < gridAmountX)
                            {
                                queue.Enqueue(new int[] { cell[0] + 1, cell[1] });
                            }
                            if(cell[1] - 1 >= 0)
                            {
                                queue.Enqueue(new int[] { cell[0], cell[1] - 1 });
                            }
                            if(cell[1] + 1 < gridAmountZ)
                            {
                                queue.Enqueue(new int[] { cell[0], cell[1] + 1 });
                            }
                            if(currentBlock == numberOfBlocks)
                            {
                                queue.Enqueue(stopBlock);
                            }
                        }
                        else
                        {
                            currentBlock++;
                        }
                    }
                }
                else //Don't have to check already visited blocks
                {
                    currentBlock++;
                    if(currentBlock == numberOfBlocks)
                    {
                        queue.Enqueue(stopBlock);
                    }
                }
            }
            return nearestPoint;
        }

        private float MeasureDistance(Vector3 a, Vector3 b)
        {
            return Vector3.Distance(a, b);
        }

        private bool NearestNeighbourTraversable(int i, int j)
        {
            if(terrain_manager.myInfo.traversability[
                terrain_manager.myInfo.get_i_index(NearestNeighbourGetAxisPositionX(i)),
                terrain_manager.myInfo.get_j_index(NearestNeighbourGetAxisPositionZ(j))
            ] > 0.5f)
            {
                return false;
            }
            return true;
        }

        private float NearestNeighbourGetAxisPositionX(int index)
        {
            return gridSize * index + gridSize / 2 + terrain_manager.myInfo.x_low;
        }

        private float NearestNeighbourGetAxisPositionZ(int index)
        {
            return gridSize * index + gridSize / 2 + terrain_manager.myInfo.z_low;
        }

        private int NearestNeighbourGetIndexI(float x)
        {
            int index = (int)Mathf.Floor(gridAmountX * (x - terrain_manager.myInfo.x_low) / world_width);
            if (index < 0)
            {
                index = 0;
            }
            else if (index > gridAmountX - 1)
            {
                index = gridAmountX - 1;
            }
            return index;
        }

        private int NearestNeighbourGetIndexJ(float z)
        {
            int index = (int)Mathf.Floor(gridAmountZ * (z - terrain_manager.myInfo.z_low) / world_height);
            if (index < 0)
            {
                index = 0;
            }
            else if (index > gridAmountZ - 1)
            {
                index = gridAmountZ - 1;
            }
            return index;
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
                    delta = 0; //Do we need this?
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

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            //Show the tree
            Gizmos.color = Color.blue;
            for(int i = 0; i < gridAmountX; ++i)
            {
                for(int j = 0; j < gridAmountZ; ++j)
                {
                    if(NearestNeighbourTraversable(i, j))
                    {
                        for (int k = 0; k < treePoints[i, j].Count; ++k)
                        {
                            foreach (TreePoint child in treePoints[i, j][k].children)
                            {
                                Gizmos.DrawLine(treePoints[i, j][k].position + Vector3.up * 0.75f, child.position + Vector3.up * 0.75f);
                            }
                        }
                    }
                }
            }

            //Show the path to the goal
            if(path != null)
            {
                Gizmos.color = Color.white;
                for (int i = 0; i < path.Count - 1; ++i)
                {
                    Gizmos.DrawLine(path[i].position + Vector3.up, path[i + 1].position + Vector3.up);
                }
            }

            //Visualizes the configuration space by adding red boxes where the center point of the car cannot go.
            if (visualizeConfigurationSpace)
            {
                Gizmos.color = Color.red;
                int areaWidth = 10;
                float min_x = Mathf.Clamp(m_Car.transform.position.x - areaWidth, terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high);
                float max_x = Mathf.Clamp(m_Car.transform.position.x + areaWidth, terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high);
                float min_z = Mathf.Clamp(m_Car.transform.position.z - areaWidth, terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high);
                float max_z = Mathf.Clamp(m_Car.transform.position.z + areaWidth, terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high);
                float size = 0.5f;

                for (float i = (int)min_x; i <= (int)max_x; i += size)
                {
                    for (float j = (int)min_z; j <= (int)max_z; j += size)
                    {
                        if (configurationSpace.Collision(i, j, m_Car.transform.eulerAngles.y))
                        {
                            Gizmos.DrawCube(new Vector3(i, 0.0f, j), new Vector3(size, 2, size));
                        }
                    }
                }
            }
        }
    }
}
