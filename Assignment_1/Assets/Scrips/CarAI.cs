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
        private int maxDistTarget;
        private int maxTimesFoundGoal;
        private int maxShortcutAngle;
        private int maxShortcutTheta;
        private float timeStep;
        private int numberOfSteps;
        private float time;

        private float steerDirection;
        private float accelerationDirection;
        private float brake;
        private float handBrake;

        private float carLength;

        private float world_width;
        private float world_height;
        private float gridSize;
        private int gridAmountX;
        private int gridAmountZ;
        private TreePoint start;
        private List<TreePoint>[,] treePoints;
        private int treeSize;

        private float shortestPointToGoalDistance;
        private List<TreePoint> path;
        private TreePoint nextPoint;
        private int pathIndex;
        private int goalFoundAmount;

        private bool crashed;
        private float crashTime;
        private float crashCheckTime;
        private float crashDirection;
        private Vector3 previousPosition;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public bool visualizeConfigurationSpace = false;
        public bool visualizeEntireTree = false;

        public ConfigurationSpace configurationSpace;

        private void Awake()
        {
            Time.timeScale = 1;
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            maxVelocity = 150;
            acceleration = 1f;

            InitializeCSpace();

            RRTIterations = 500000;     // Limits the number of iterations allowed in the RRT before running the car.
            maxTimesFoundGoal = 10;    // Limits the times that we find goal before running the car.
            maxShortcutAngle = 5;       // Angle deviance accepted for points that can give a straight line shortcut.
            maxShortcutTheta = 15;
            maxDistTarget = 5;          // Maximum accepted distance from a target point.
            timeStep = 0.05f;
            numberOfSteps = 5;
            time = 0;

            steerDirection = 0;
            accelerationDirection = 0;
            brake = 0;
            handBrake = 0;

            carLength = FindCarLength();

            world_width = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            world_height = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            gridSize = Mathf.Min(
                world_width / terrain_manager.myInfo.x_N,
                world_height / terrain_manager.myInfo.z_N
            );
            gridAmountX = (int)(world_width / gridSize);
            gridAmountZ = (int)(world_height / gridSize);
            start = new TreePoint(terrain_manager.myInfo.start_pos, m_Car.transform.eulerAngles.y, 0, 0);
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
            treeSize = 1;

            shortestPointToGoalDistance = Vector3.Distance(start.position, terrain_manager.myInfo.goal_pos);
            path = new List<TreePoint>();
            pathIndex = 1;
            goalFoundAmount = 0;

            path = RRT();

            if(path == null)
            {
                Debug.Log("No path found!");
            }
            nextPoint = path[pathIndex];

            crashed = false;
            crashTime = 0;
            crashCheckTime = 0.5f;
            crashDirection = 0;
            previousPosition = Vector3.up;
        }

        private void FixedUpdate()
        {
            if(!crashed)
            {
                time += Time.deltaTime;
                if (time >= crashCheckTime && Vector3.Distance(m_Car.transform.position, terrain_manager.myInfo.start_pos) > 5)
                {
                    time = 0;
                    if (Vector3.Distance(previousPosition, m_Car.transform.position) < 0.1f)
                    {
                        crashed = true;
                        if(Physics.BoxCast(
                            m_Car.transform.position, 
                            new Vector3(configurationSpace.BoxSize.x/2, configurationSpace.BoxSize.y/2, 0.5f),
                            m_Car.transform.forward,
                            Quaternion.LookRotation(m_Car.transform.forward),
                            configurationSpace.BoxSize.z/2
                        ))
                        {
                            crashDirection = -1;
                        }
                        else
                        {
                            crashDirection = 1;
                        }

                    }
                    else
                    {
                        previousPosition = m_Car.transform.position;
                    }
                }


                //Improve path by straightening unnecessary curves
                if (pathIndex < path.Count - 1)
                {
                    for (int i = path.Count - 1; i >= pathIndex + 1; --i)
                    {
                        Vector3 position = m_Car.transform.position;
                        Vector3 direction = path[i].position - position;
                        float angle = Vector3.Angle(m_Car.transform.forward, direction);

                        //If somewhat straight to point on path, check if it is possible to go there
                        if (angle <= maxShortcutAngle && Quaternion.Angle(Quaternion.Euler(0, path[i].theta, 0), Quaternion.LookRotation(direction)) <= maxShortcutTheta)
                        {
                            bool collision = false;
                            for(float h = 0; h <= 1; h += 0.005f)
                            {
                                float x = (position + direction * h).x;
                                float z = (position + direction * h).z;
                                if (configurationSpace.Collision(x, z, Quaternion.LookRotation(direction).y)) 
                                {
                                    collision = true;
                                }
                            }

                            //No collision, go to this point instead
                            if(!collision)
                            {
                                nextPoint = path[pathIndex];
                                pathIndex = i;
                                break;
                            }
                        }
                    }
                }

                steerDirection = SteerInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, nextPoint.position);
                if (Mathf.Abs(steerDirection) < 0.2f)
                {
                    steerDirection = 0;
                }
                brake = 0;
                if (Mathf.Abs(steerDirection) > 0.8f && m_Car.CurrentSpeed > maxVelocity / 10)
                {
                    accelerationDirection = 0;
                    if(m_Car.CurrentSpeed > maxVelocity / 5)
                    {
                        handBrake = 1;
                    }
                    else
                    {
                        handBrake = 0;
                    }
                }
                else
                {
                    accelerationDirection = AccelerationInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, nextPoint.position);
                    handBrake = 0;
                }
                if (pathIndex < path.Count - 1)
                {
                    int stepsToCheck = Mathf.Min(3 + (int)(m_Car.CurrentSpeed * 1.6f * 1.6f * m_Car.CurrentSpeed / 500), path.Count - 1 - pathIndex);
                    for (int i = 1; i <= stepsToCheck; ++i)
                    {
                        float steerCheck = SteerInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, path[pathIndex + i].position);
                        if (Mathf.Abs(steerCheck) > 0.8f && (m_Car.CurrentSpeed * 1.6f * 1.6f * m_Car.CurrentSpeed) >= Vector3.Distance(m_Car.transform.position, path[pathIndex + i].position) * 250 * 0.8f)
                        {
                            accelerationDirection = 0;
                            brake = 1;
                            break;
                        }
                    }
                }

                if (m_Car.CurrentSpeed >= maxVelocity)
                {
                    accelerationDirection = 0;
                }

                if (accelerationDirection < 0)
                {
                    m_Car.Move(-steerDirection, brake, accelerationDirection * acceleration, handBrake);
                }
                else
                {
                    m_Car.Move(steerDirection, accelerationDirection * acceleration, -brake, handBrake);
                }
            }
            else
            {
                crashTime += Time.deltaTime;
                if(crashTime <= 1f)
                {
                    steerDirection = SteerInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, nextPoint.position);
                    if (crashDirection > 0)
                    {
                        m_Car.Move(steerDirection, acceleration, 0, 0);
                    }
                    else
                    {
                        m_Car.Move(-steerDirection, 0, -acceleration, 0);
                    }
                }
                else
                {
                    crashTime = 0;
                    crashed = false;
                }
            }

            //Update point if close enough to current one
            if (Vector3.Distance(m_Car.transform.position, nextPoint.position) <= maxDistTarget + m_Car.CurrentSpeed / 40)
            {
                pathIndex = Mathf.Min(pathIndex + 1, path.Count - 1);
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
                    float probability = foundGoal ? 1 : UnityEngine.Random.Range(0f, 1f);
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

                List<TreePoint> nearestPoints = kNearestNeighbours(randomPoint, 1);
                TreePoint nearPoint = nearestPoints.Count > 0 ? nearestPoints[0] : null;
                if(nearPoint == null)
                {
                    continue;
                }
                TreePoint newPoint = SimulateMovement(nearPoint, randomPoint, 1);
                if(newPoint != null)
                {
                    //Check traversability
                    int newIIndex = NearestNeighbourGetIndexI(newPoint.position.x);
                    int newJIndex = NearestNeighbourGetIndexJ(newPoint.position.z);
                    if(!NearestNeighbourTraversable(newIIndex, newJIndex))
                    {
                        continue;
                    }
                    nearestPoints = kNearestNeighbours(newPoint.position, Mathf.Min(10 + treeSize/1000, treeSize));
                    treePoints[newIIndex, newJIndex].Add(newPoint);
                    treeSize++;

                    //RRT*
                    TreePoint minPoint = nearPoint;
                    float minCost = newPoint.cost;

                    foreach(TreePoint point in nearestPoints)
                    {
                        TreePoint p = SimulateMovement(point, newPoint.position, 3);
                        if(p != null)
                        {
                            if (Vector3.Distance(p.position, newPoint.position) <= 0.1f && p.cost < minCost)
                            {
                                minCost = p.cost;
                                minPoint = point;
                            }
                        }
                    }

                    newPoint.parent = minPoint;
                    minPoint.children.Add(newPoint);

                    //Update value for sampling heuristic
                    if (Vector3.Distance(newPoint.position, terrain_manager.myInfo.goal_pos) < shortestPointToGoalDistance)
                    {
                        shortestPointToGoalDistance = Vector3.Distance(newPoint.position, terrain_manager.myInfo.goal_pos);
                    }

                    //Found goal
                    if (Vector3.Distance(newPoint.position, terrain_manager.myInfo.goal_pos) <= 5)
                    {
                        foundGoal = true;
                        if(pathPoint == null)
                        {
                            pathPoint = newPoint;
                        }
                        else if (newPoint.cost < pathPoint.cost)
                        {
                            pathPoint = newPoint;
                        }
                        goalFoundAmount++;
                        if(goalFoundAmount >= maxTimesFoundGoal)
                        {
                            Debug.Log("Found goal " + maxTimesFoundGoal + "  times, stopping.");
                            break;
                        }
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
                Debug.Log("Reached " + RRTIterations + " RRT-iterations with at least one goal, stopping.");
                return goalPath;
            }
            Debug.Log("Reached " + RRTIterations + " RRT-iterations but found no goal, stopping.");
            return null;
        }

        //Finds the nearest tree point to a point.
        public List<TreePoint> kNearestNeighbours(Vector3 point, int k)
        {
            //Keep track of the closest point
            List<float> minDistances = new List<float>();
            List<TreePoint> nearestPoints = new List<TreePoint>();

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
                    if(nearestPoints.Count == k)
                    {
                        return nearestPoints;
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
                            if(nearestPoints.Count < k)
                            {
                                nearestPoints.Add(treePoint);
                                minDistances.Add(distance);
                            }
                            else
                            {
                                int maxIndex = 0;
                                float maxDistance = 0;
                                for (int i = 0; i < k; ++i)
                                {
                                    if(minDistances[i] > maxDistance)
                                    {
                                        maxIndex = i;
                                        maxDistance = minDistances[i];
                                    }
                                }
                                if(distance < minDistances[maxIndex])
                                {
                                    minDistances[maxIndex] = distance;
                                    nearestPoints[maxIndex] = treePoint;
                                }
                            }

                        }

                        //Haven't found enough points in this cell, enqueue the enclosing ones
                        if (nearestPoints.Count < k)
                        {
                            currentBlock++;
                            for(int i = -1; i <= 1; ++i)
                            {
                                for(int j = -1; j <= 1; ++j)
                                {
                                    if(cell[0] + i >= 0 && cell[0] + i < gridAmountX &&
                                       cell[1] + j >= 0 && cell[1] + j < gridAmountZ)
                                    {
                                        queue.Enqueue(new int[] { cell[0] + i, cell[1] + j });
                                    }
                                }
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
            return nearestPoints;
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
        public TreePoint SimulateMovement(TreePoint from, Vector3 to, int timeFactor)
        {
            Vector3 position = from.position;
            float theta = from.theta;
            float velocity = from.velocity;
            float cost = from.cost;
            for(int step = 0; step < timeFactor*numberOfSteps; step++)
            {
                //Get steering angle
                float delta = m_Car.m_MaximumSteerAngle * SteerInput(position, theta, to);
                float braking = 1;
                if(AccelerationInput(position, theta, to) < 0)
                {
                    delta = -delta;
                }
                if(Mathf.Abs(delta) <= 0.4f*m_Car.m_MaximumSteerAngle)
                {
                    delta = 0; //Do we need this?
                } else if(Mathf.Abs(delta) > 0.8f*m_Car.m_MaximumSteerAngle)
                {
                    if(Mathf.Abs(velocity) > maxVelocity / 10)
                    {
                        braking = -1;
                    }
                }
                /*
                //Calculate motion model values according to kinematic car model
                float xDiff = velocity * Mathf.Sin(Mathf.Deg2Rad * theta);
                float zDiff = velocity * Mathf.Cos(Mathf.Deg2Rad * theta);
                float thetaDiff = velocity / carLength * Mathf.Tan(Mathf.Deg2Rad * delta) * Mathf.Rad2Deg;

                //Get new position and orientation using Euler's method
                Vector3 newPosition = new Vector3(Euler(position.x, xDiff, timeStep), 0, Euler(position.z, zDiff, timeStep));
                cost += Vector3.Distance(position, newPosition);
                position = newPosition;
                theta = Euler(theta, thetaDiff, timeStep);
                */

                //Get new position and orientation using RK4
                float[] nextState = RK4Step(position.x, position.z, theta, delta, carLength, velocity, timeStep);
                position.x = nextState[0];
                position.z = nextState[1];
                theta = nextState[2];
                

                //If collision, this point is not traversable
                if(configurationSpace.Collision(position.x, position.z, theta))
                {
                    return null;
                }

                //If close enough to end position, stop iterating
                if(Vector3.Distance(position, to) <= 0.1f)
                {
                    break;
                }

                velocity += Mathf.Clamp(AccelerationInput(position, theta, to) * acceleration * timeStep * braking, -maxVelocity, maxVelocity);
            }

            TreePoint result = new TreePoint(position, theta, velocity, cost);
            return result;
        }

        //Extrapolates the value in the next position given the derivative of the value
        float Euler(float value, float difference, float step)
        {
            return value + difference * step;
        }

        private float[] FPrime(float theta, float delta, float carLenght, float velocity)
        {
            float xDiff = velocity * Mathf.Sin(Mathf.Deg2Rad * theta);
            float zDiff = velocity * Mathf.Cos(Mathf.Deg2Rad * theta);
            float thetaDiff = velocity / carLength * Mathf.Tan(Mathf.Deg2Rad * delta) * Mathf.Rad2Deg;
            float[] diff = new float[] { xDiff, zDiff, thetaDiff};
            return diff;

        }

        private float [] RK4Step(float x, float z, float theta, float delta, float length, float velocity, float h) 
        {
            float[] k1 = FPrime(theta, delta, length, velocity);
            float[] k2 = FPrime(theta + k1[2] * (timeStep / 2), delta, length, velocity);
            float[] k3 = FPrime(theta + k2[2] * (timeStep / 2), delta, length, velocity);
            float[] k4 = FPrime(theta + k3[2] * (timeStep), delta, length, velocity);

            float[] newState = new float[] { 0, 0, 0 };

            float[] oldState = new float[] { x, z, theta };
            for(int i = 0; i < 3; i++){
                newState[i] = oldState[i] + (h / 6) * (k1[i] + k2[i] + k3[i] + k4[i]);
                }
            return newState;
            }

        //Determines delta for the kinematic motion model
        private float SteerInput(Vector3 position, float theta, Vector3 point)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToPoint = point - position;
            float angle = Vector3.Angle(direction, directionToPoint) * Mathf.Sign(-direction.x * directionToPoint.z + direction.z * directionToPoint.x);
            float steerAngle = Mathf.Clamp(angle, -m_Car.m_MaximumSteerAngle, m_Car.m_MaximumSteerAngle) / m_Car.m_MaximumSteerAngle;
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
            if(visualizeEntireTree)
            {
                Gizmos.color = Color.blue;
                for (int i = 0; i < gridAmountX; ++i)
                {
                    for (int j = 0; j < gridAmountZ; ++j)
                    {
                        if (NearestNeighbourTraversable(i, j))
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
