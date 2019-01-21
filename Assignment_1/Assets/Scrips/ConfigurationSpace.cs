using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class ConfigurationSpace
    {

        public Vector3 CalculateBoundLengths()
        {
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            return carCollider.bounds.extents;
        }

        public bool Collision(TerrainManager terrainManager, float x, float z)
        {
            //If grid cell is obstacle, return false
            int gridX = terrainManager.myInfo.get_i_index(x);
            int gridZ = terrainManager.myInfo.get_j_index(z);
            if(terrainManager.myInfo.traversability[gridX,gridZ] > 0.5f)
            {
                return true;
            }

            //Get grid cell width and height
            float gridCellWidth = (terrainManager.myInfo.x_high - terrainManager.myInfo.x_low) / terrainManager.myInfo.x_N;
            float gridCellHeight = (terrainManager.myInfo.z_high - terrainManager.myInfo.z_low) / terrainManager.myInfo.z_N;
            Vector3 boundLengths = CalculateBoundLengths();

            for (int i = -1; i <= 1; ++i)
            {
                for(int j = -1; j <= 1; ++j)
                {
                    //Don't check outside the grid
                    if(gridX + i < 0 || gridX + i >= terrainManager.myInfo.x_N || 
                       gridZ + j < 0 || gridZ + j >= terrainManager.myInfo.z_N)
                    {
                        continue;
                    }

                    float gridXPos = terrainManager.myInfo.get_x_pos(gridX + i);
                    float gridZPos = terrainManager.myInfo.get_z_pos(gridZ + j);

                    //Diagonals
                    if (i == j || i == -j)
                    {
                        if(i == -1 && j == -1)
                        {
                            float k = ((gridZPos - gridCellHeight / 2) - (gridZPos - gridCellHeight / 2 - boundLengths.z)) / ((gridXPos + gridCellWidth / 2 + boundLengths.x) - (gridXPos + gridCellWidth / 2));
                            float m = gridZPos - gridCellHeight / 2 - boundLengths.z;
                            float y = k * (x - (gridXPos + gridCellWidth / 2)) + m;
                            if(y <= z)
                            {
                                return true;
                            }
                        } else if(i == -1 && j == 1) 
                        {
                            float k = ((gridZPos + gridCellHeight / 2 + boundLengths.z) - (gridZPos + gridCellHeight / 2)) / ((gridXPos + gridCellWidth / 2 + boundLengths.x) - (gridXPos + gridCellWidth / 2));
                            float m = gridZPos + gridCellHeight / 2;
                            float y = k * (x - (gridXPos + gridCellWidth / 2)) + m;
                            if (y >= z)
                            {
                                return true;
                            }
                        } else if(i == 1 && j == -1)
                        {
                            float k = ((gridZPos - gridCellHeight / 2) - (gridZPos - gridCellHeight / 2 - boundLengths.z)) / ((gridXPos - gridCellWidth / 2 - boundLengths.x) - (gridXPos - gridCellWidth / 2));
                            float m = gridZPos - gridCellHeight / 2;
                            float y = k * (x - (gridXPos - gridCellWidth / 2 - boundLengths.x)) + m;
                            if (y <= z)
                            {
                                return true;
                            }
                        } else if(i == 1 && j == 1)
                        {
                            float k = ((gridZPos + gridCellHeight / 2 + boundLengths.z) - (gridZPos + gridCellHeight / 2)) / ((gridXPos - gridCellWidth / 2 - boundLengths.x) - (gridXPos - gridCellWidth / 2));
                            float m = gridZPos + gridCellHeight / 2;
                            float y = k * (x - (gridXPos - gridCellWidth / 2 - boundLengths.x)) + m;
                            if (y >= z)
                            {
                                return true;
                            }
                        }
                    }
                    else
                    {
                        if(x <= gridXPos + gridCellWidth/2 + boundLengths.x &&
                           x >= gridXPos - gridCellWidth/2 - boundLengths.x &&
                           z <= gridZPos + gridCellHeight/2 + boundLengths.z &&
                           z >= gridZPos - gridCellHeight/2 - boundLengths.z)
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        }
    }
}