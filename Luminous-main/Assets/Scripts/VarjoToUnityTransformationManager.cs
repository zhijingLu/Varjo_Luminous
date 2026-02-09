using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEditor.XR.LegacyInputHelpers;
using UnityEngine;
using Varjo.XR;
using DFKI_Utilities;
using static DFKI_Utilities.VarjoToUnityTransformation;
using System.IO;

public class VarjoToUnityTransformationManager : MonoBehaviour
{
    private class OpenCV
    {
        [DllImport("OpenCV")]
        unsafe public static extern float* DetectArucoMarkers(byte[] img, int width, int height, int numChannels, int bitsPerChannel, int dictionary);

        [DllImport("OpenCV")]
        unsafe public static extern float* Triangulate2DPoints(float[] coordLeft, float[] coordRight, uint size, float[] projMatrixLeft, float[] projMatrixRight);
    }

    public VarjoApiManager varjoApiManager;
    public GameObject cameraOffset;
    public Matrix4x4 fromVarjoToUnityTransformation = Matrix4x4.identity;
    public Matrix4x4 fromUnityToVarjoTransformation = Matrix4x4.identity;
    public bool debugMode = false;

    private VarjoToUnityTransformation calibration;
    private GameObject[] markerObjects; // game objects to visualize for testing out the transformation
    private bool initialized = false;

    // Start is called before the first frame update
    void Start()
    {
        // read transformation from file if it exists
        string filePath = Application.dataPath + "/../Output/VarjoToUnityTransformation.txt";

        if (LoadVarjoUnityTransformations(filePath, out fromVarjoToUnityTransformation, out fromUnityToVarjoTransformation))
        {
            Debug.Log("Loaded transformations from file!");
            // print loaded matrices
            Debug.Log("From Varjo to Unity Transformation:\n" + fromVarjoToUnityTransformation.ToString());
            Debug.Log("From Unity to Varjo Transformation:\n" + fromUnityToVarjoTransformation.ToString());

        }
        else
        {
            Debug.LogWarning("Could not load transformation matrices; using identity.");
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.I))
        {
            Debug.Log("VarjoToUnityTransformationManager: Initialize!");
            calibration = new VarjoToUnityTransformation(ref varjoApiManager.cameraLeft, ref varjoApiManager.cameraRight, debugMode);
            initialized = true;
        }
        else if (Input.GetKeyDown(KeyCode.C))
        {
            if (initialized)
            {
                Debug.Log("VarjoToUnityTransformationManager: Compute Varjo to Unity Transformation!");

                // update texture and Xtrinsics
                varjoApiManager.UpdateTexture2D();
                varjoApiManager.UpdateXtrinsics();

                // detect markers
                bool resultLeft = DetectMarkers2D(ref varjoApiManager.leftTexture, out Vector2[] coordsLeft, out long[] idsLeft);
                bool resultRight = DetectMarkers2D(ref varjoApiManager.rightTexture, out Vector2[] coordsRight, out long[] idsRight);

                calibration.debugMode = debugMode;
                calibration.Calibrate(ref varjoApiManager.leftTexture, ref varjoApiManager.rightTexture, new Vector3(0.0f, 0.0f, 0.3f),
                    ComputeVarjoMarkers3D(), resultLeft, resultRight, coordsLeft, coordsRight, idsLeft, idsRight, true);
                fromVarjoToUnityTransformation = calibration.GetTransformation();
                fromUnityToVarjoTransformation = fromVarjoToUnityTransformation.inverse;

                // write transformation to file
                string filePath = Application.dataPath + "/../Output/VarjoToUnityTransformation.txt";
                using (StreamWriter writer = new StreamWriter(filePath))
                {
                    writer.WriteLine("From Varjo to Unity Transformation:");
                    writer.WriteLine(fromVarjoToUnityTransformation.ToString());
                    writer.WriteLine("From Unity to Varjo Transformation:");
                    writer.WriteLine(fromUnityToVarjoTransformation.ToString());
                }

                SaveImages(varjoApiManager.leftTexture, varjoApiManager.rightTexture, coordsLeft, coordsRight);
            }
        }
        else if (Input.GetKeyDown(KeyCode.V))
        {
            if (initialized)
            {
                Debug.Log("VarjoToUnityTransformationManager: Compute Varjo Markers and Create corresponding 3D Unity Objects!");

                // update texture and Xtrinsics
                varjoApiManager.UpdateTexture2D();
                varjoApiManager.UpdateXtrinsics();

                bool resultLeft = DetectMarkers2D(ref varjoApiManager.leftTexture, out Vector2[] coordsLeft, out long[] idsLeft);
                bool resultRight = DetectMarkers2D(ref varjoApiManager.rightTexture, out Vector2[] coordsRight, out long[] idsRight);

                calibration.debugMode = debugMode;
                calibration.ComputeOurMarkers3D(ref varjoApiManager.leftTexture, ref varjoApiManager.rightTexture, new Vector3(0.0f, 0.0f, 0.3f),
                    true, Optimization.Method.GradientDescent_BLS_Fast, resultLeft, resultRight, coordsLeft, coordsRight, idsLeft, idsRight, out float pixelError);

                // re-initialize 3D blob objects
                if (markerObjects != null)
                {
                    for (var i = 0; i < markerObjects.Length; i++)
                        Destroy(markerObjects[i]);
                }

                var markers = calibration.GetOurMarkers3D();
                markerObjects = new GameObject[markers.Count];

                for (int i = 0; i < markers.Count; i++)
                {
                    Vector3 position = fromVarjoToUnityTransformation.MultiplyPoint3x4(markers[i].position);
                    markerObjects[i] = UnityUtils.CreateNewGameObject("marker_" + i, position, UnityEngine.Color.red, true, cameraOffset, 0.01f);
                }

                Debug.Log("    2D pixel error = " + pixelError);

                SaveImages(varjoApiManager.leftTexture, varjoApiManager.rightTexture, coordsLeft, coordsRight);
            }
        }
    }

    private List<Marker> ComputeVarjoMarkers3D()
    {
        // clean up
        //varjoMarkers.Clear();
        List<Marker> varjoMarkers = new List<VarjoToUnityTransformation.Marker>();

        // detect...
        bool alreadyEnabled = VarjoMarkers.IsVarjoMarkersEnabled();
        if (!alreadyEnabled)
            VarjoMarkers.EnableVarjoMarkers(true);

        // Get a list of markers with up-to-date data.
        List<VarjoMarker> markers = new List<VarjoMarker>();
        VarjoMarkers.GetVarjoMarkers(out markers);

        if (markers.Count <= 0)
            return varjoMarkers; // No marker was found!

        for (int i = 0; i < markers.Count; i++)
            varjoMarkers.Add(new Marker(markers[i].id, markers[i].pose.position));

        if (!alreadyEnabled)
            VarjoMarkers.EnableVarjoMarkers(false);

        return varjoMarkers;
    }

    private bool DetectMarkers2D(ref Texture2D image, out Vector2[] coords, out long[] ids)
    {
        // Detect Aruco Markers
        int nMarkers = 0;
        float[] raw;

        coords = new Vector2[nMarkers];
        ids = new long[nMarkers];

        unsafe
        {
            float* raw_result = OpenCV.DetectArucoMarkers(image.GetRawTextureData(), image.width, image.height, Writer.GetNumChannels(ref image), Writer.GetBitsPerChannel(ref image), -1);

            nMarkers = (int)(raw_result[0]);

            if (nMarkers <= 0)
                return false; // No marker has been detected

            int size = 1 + nMarkers * 9; // nMarkers, [id, cornerA (x,y), cornerB (x,y), cornerC (x,y), cornerD (x,y)]
            raw = new float[size];

            Marshal.Copy((IntPtr)raw_result, raw, 0, size);
        }

        coords = new Vector2[nMarkers];
        ids = new long[nMarkers];

        for (int i = 0; i < nMarkers; ++i)
        {
            ids[i] = (long)raw[1 + i * 9 + 0];

            // average corner center in 2D
            Vector2 c0 = new Vector2(raw[1 + i * 9 + 1], raw[1 + i * 9 + 2]);
            Vector2 c1 = new Vector2(raw[1 + i * 9 + 3], raw[1 + i * 9 + 4]);
            Vector2 c2 = new Vector2(raw[1 + i * 9 + 5], raw[1 + i * 9 + 6]);
            Vector2 c3 = new Vector2(raw[1 + i * 9 + 7], raw[1 + i * 9 + 8]);

            coords[i] = (c0 + c1 + c2 + c3) / 4.0f;
        }

        return true;
    }

    private void SaveImages(Texture2D imageLeft, Texture2D imageRight, Vector2[] coordsLeft, Vector2[] coordsRight)
    {
        if (debugMode)
        {
            // Debug
            Writer.Settings pLeft = new Writer.Settings(coordsLeft.Length);
            pLeft.SetPosition(coordsLeft);
            pLeft.SetColor(Color.red, false);
            pLeft.SetRadius(10);
            Writer.Settings pRight = new Writer.Settings(coordsRight.Length);
            pRight.SetPosition(coordsRight);
            pRight.SetColor(Color.blue, false);
            pRight.SetRadius(5);

            Writer.FlipImage(ref imageLeft);
            Writer.FlipImage(ref imageRight);

            Texture2D left = Writer.PrintTextureCircles(pLeft, imageLeft);
            byte[] bytesLeft = left.EncodeToPNG();
            System.IO.File.WriteAllBytes(Application.dataPath + "/markers_left.png", bytesLeft);

            Texture2D right = Writer.PrintTextureCircles(pRight, imageRight);
            byte[] bytesRight = right.EncodeToPNG();
            System.IO.File.WriteAllBytes(Application.dataPath + "/markers_right.png", bytesRight);
        }
    }

    public static bool LoadVarjoUnityTransformations(string filePath, out Matrix4x4 fromVarjoToUnity, out Matrix4x4 fromUnityToVarjo)
    {
        fromVarjoToUnity = Matrix4x4.identity;
        fromUnityToVarjo = Matrix4x4.identity;

        if (!File.Exists(filePath))
        {
            Debug.LogWarning("Transformation file not found: " + filePath);
            return false;
        }

        string[] lines = File.ReadAllLines(filePath);

        int matCount = 0;
        float[,] mat1 = new float[4,4];
        float[,] mat2 = new float[4,4];
        int row = 0;
        bool readingFirst = false, readingSecond = false;

        foreach (string line in lines)
        {
            if (line.StartsWith("From Varjo to Unity Transformation"))
            {
                readingFirst = true;
                readingSecond = false;
                row = 0;
                continue;
            }
            if (line.StartsWith("From Unity to Varjo Transformation"))
            {
                readingFirst = false;
                readingSecond = true;
                row = 0;
                continue;
            }
            if (string.IsNullOrWhiteSpace(line))
                continue;

            string[] tokens = line.Split(new char[] {'\t', ' '}, StringSplitOptions.RemoveEmptyEntries);
            if (tokens.Length == 4)
            {
                float[] vals = Array.ConvertAll(tokens, float.Parse);
                if (readingFirst)
                {
                    for (int i = 0; i < 4; i++)
                        mat1[row, i] = vals[i];
                    row++;
                    if (row == 4) readingFirst = false;
                }
                else if (readingSecond)
                {
                    for (int i = 0; i < 4; i++)
                        mat2[row, i] = vals[i];
                    row++;
                    if (row == 4) readingSecond = false;
                }
            }
        }

        // Convert arrays to Matrix4x4
        fromVarjoToUnity = ArrayToMatrix(mat1);
        fromUnityToVarjo = ArrayToMatrix(mat2);

        return true;
    }

    private static Matrix4x4 ArrayToMatrix(float[,] mat)
    {
        Matrix4x4 m = new Matrix4x4();
        for (int row = 0; row < 4; row++)
            for (int col = 0; col < 4; col++)
                m[row, col] = mat[row, col];
        return m;
    }
}
