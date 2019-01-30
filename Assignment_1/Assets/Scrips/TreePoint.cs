using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TreePoint
{
    public Vector3 position;
    public float theta;
    public float velocity;
    public float cost;

    public TreePoint parent = null;
    public List<TreePoint> children;

    public TreePoint(Vector3 position, float theta, float velocity, float cost)
    {
        this.position = position;
        this.theta = theta;
        this.velocity = velocity;
        this.cost = cost;

        children = new List<TreePoint>();
    }

    public TreePoint(Vector3 position, float theta, float velocity, float cost, TreePoint parent)
    {
        this.position = position;
        this.theta = theta;
        this.velocity = velocity;
        this.cost = cost;

        this.parent = parent;
        children = new List<TreePoint>();
    }
}
