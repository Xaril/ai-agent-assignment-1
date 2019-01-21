using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class ConfigurationSpace
    {

        public Vector3 CalculateAngleLengths()
        {
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            return carCollider.bounds.extents;
        }
    }
}