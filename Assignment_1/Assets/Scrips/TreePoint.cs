using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TreePoint
{
    public Vector3 position;
    public float theta;
    public float velocity;

    public TreePoint parent = null;
    public List<TreePoint> children;

    public TreePoint(Vector3 position, float theta, float velocity)
    {
        this.position = position;
        this.theta = theta;
        this.velocity = velocity;

        children = new List<TreePoint>();
    }

    public TreePoint(Vector3 position, float theta, float velocity, TreePoint parent)
    {
        this.position = position;
        this.theta = theta;
        this.velocity = velocity;

        this.parent = parent;
        children = new List<TreePoint>();
    }
}
