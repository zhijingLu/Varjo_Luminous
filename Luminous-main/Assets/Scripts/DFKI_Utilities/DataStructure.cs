using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Collections;
using System;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Text;
using UnityDebug = UnityEngine.Debug;
namespace DFKI_Utilities
{
    public class DataStructure
    {
        
        public struct BBox
        {
            /*
            * this is for model return 
            */
            public int Id;
            public float[] Box; // [x1,y1,w,h]

            public BBox(int id, float[] box)
            {
                Id = id;
                Box = box;
            }
        }

        public struct Corner2DInfo
        {
            public int Id;
            public Vector2[] Points; // 4 corners point, Top-Left, Top-Right, Bottom-Right, Bottom-Left

            public Corner2DInfo(int id, Vector2[] points)
            {
                Id = id;
                Points = points;
            }
            
        }


        public struct Corner3DQuad
        {
            public int Id;
            public Vector3[] Points3D;   // 4 triangulated corners

            // store observed corners so optimizer can compute reprojection error
            public Vector2[] Left2D;     // 4 observed (input 2D)
            public Vector2[] Right2D;    // 4 observed

            public Corner3DQuad(int id, Vector3[] pts3D, Vector2[] l2d, Vector2[] r2d)
            {
                Id = id;
                Points3D = pts3D;
                Left2D = l2d;
                Right2D = r2d;
            }
        }
       



    }


}