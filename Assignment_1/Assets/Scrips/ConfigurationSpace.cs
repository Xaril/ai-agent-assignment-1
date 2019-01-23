using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class ConfigurationSpace
    {
        private Vector3 boxSize;

        public Vector3 BoxSize
        {
            get
            {
                return boxSize;
            }

            set
            {
                boxSize = value;
            }
        }

        //Checks for a collision at a given point and direction
        public bool Collision(float x, float z, float theta)
        {
            Vector3 position = new Vector3(x, 3, z);
            Quaternion rotation = Quaternion.Euler(0, theta, 0);
            return Physics.CheckBox(position, BoxSize / 2, rotation);
        }
    }
}