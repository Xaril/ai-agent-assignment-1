using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class ConfigurationSpace
    {
        //Checks for a collision at a given point and direction
        public bool Collision(float x, float z, float theta)
        {
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            Vector3 position = new Vector3(x, 3, z);
            Quaternion rotation = Quaternion.Euler(0, theta, 0);
            return Physics.CheckBox(position, carCollider.transform.TransformVector(carCollider.size) / 2, rotation);
        }
    }
}