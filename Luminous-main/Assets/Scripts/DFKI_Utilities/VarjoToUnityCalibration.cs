using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Collections;
using System;
using System.Runtime.InteropServices;
using System.Diagnostics;
using static DFKI_Utilities.DataStructure;
using UnityDebug = UnityEngine.Debug;
namespace DFKI_Utilities
{
    public class VarjoToUnityCalibration
    {
        // input settings
        public CameraView cameraLeft;
        public CameraView cameraRight;
        public bool debugMode = false;
        private Optimization.Settings quadSettings;
        public class OpenCV
        {
            [DllImport("OpenCV")]
            unsafe public static extern float* DetectArucoMarkers(byte[] img, int width, int height, int numChannels, int bitsPerChannel, int dictionary);

            [DllImport("OpenCV")]
            unsafe public static extern float* Triangulate2DPoints(float[] coordLeft, float[] coordRight, uint size, float[] projMatrixLeft, float[] projMatrixRight);
        }
        public class Marker
        {
            public long id = -1;
            public Vector3 position = Vector3.zero;

            public Marker(long _id, Vector3 _position)
            {
                id = _id;
                position = _position;
            }

            public Marker(Marker marker)
            {
                id = marker.id;
                position = new Vector3(marker.position.x, marker.position.y, marker.position.z); // full copy!
            }
        }






        // outputs
        private List<Marker> ourMarkers = new List<Marker>();
        private List<Marker> varjoMarkers = new List<Marker>();
        private List<System.Tuple<Vector2, Vector2>> ourMarkerCorrespondences = new List<System.Tuple<Vector2, Vector2>>();
        private Matrix4x4 transformation = Matrix4x4.identity;


        private readonly Vector3[] quadCurrent = new Vector3[4];
        private readonly Vector2[] quadObsLeft = new Vector2[4];
        private readonly Vector2[] quadObsRight = new Vector2[4];

        public VarjoToUnityCalibration(ref CameraView _cameraLeft, ref CameraView _cameraRight, bool _debugMode = true)
        {
            cameraLeft = _cameraLeft;
            cameraRight = _cameraRight;
            debugMode = _debugMode;

        }

        public Matrix4x4 GetTransformation()
        {
            return transformation;
        }


        /// <summary>
        /// Estimates the rigid transformation from the Varjo coordinate system to the Unity coordinate system
        /// using stereo triangulated ArUco marker detections and known Varjo marker positions.
        /// The calibration is translation-only (no rotation), assuming both coordinate systems are already
        /// approximately aligned in orientation.
        /// </summary>
        /// <param name="imageLeft">
        /// Left camera image used for marker detection and reprojection.
        /// </param>
        /// <param name="imageRight">
        /// Right camera image used for marker detection and reprojection.
        /// </param>
        /// <param name="init">
        /// Initial 3D position estimate for all markers, used as the starting point
        /// for gradient-descent optimization during triangulation.
        /// </param>
        /// <param name="_varjoMarkers">
        /// List of 3D marker positions provided by the Varjo SDK, expressed in Unity coordinates.
        /// Marker IDs must correspond to the detected ArUco marker IDs.
        /// </param>
        /// <param name="resultLeft">
        /// True if 2D marker detection on the left image succeeded.
        /// </param>
        /// <param name="resultRight">
        /// True if 2D marker detection on the right image succeeded.
        /// </param>
        /// <param name="coordsLeft">
        /// Array of 2D pixel coordinates of detected markers in the left image.
        /// The order must correspond to <paramref name="idsLeft"/>.
        /// </param>
        /// <param name="coordsRight">
        /// Array of 2D pixel coordinates of detected markers in the right image.
        /// The order must correspond to <paramref name="idsRight"/>.
        /// </param>
        /// <param name="idsLeft">
        /// Array of marker IDs detected in the left image.
        /// </param>
        /// <param name="idsRight">
        /// Array of marker IDs detected in the right image.
        /// </param>
        /// <param name="flipCoordinates">
        /// If true, vertically flips image coordinates (y' = imageHeight - y) when computing reprojection
        /// errors. This should be enabled when Unity and image coordinate systems differ in Y direction.
        /// </param>
        /// <returns>
        /// True if a valid Varjo-to-Unity transformation could be estimated; false otherwise.
        /// On success, the resulting transformation can be retrieved via <see cref="GetTransformation"/>.
        /// </returns>
        ///
        /// <seealso cref="ComputeOurMarkers3D"/>
        /// <seealso cref="GetTransformation"/>
        public bool Calibrate(ref Texture2D imageLeft, ref Texture2D imageRight, Vector3 init, List<Marker> _varjoMarkers,
            bool resultLeft, bool resultRight, Vector2[] coordsLeft, Vector2[] coordsRight, long[] idsLeft, long[] idsRight,
            bool flipCoordinates = true)
        {
            transformation = Matrix4x4.identity;
            Vector3 translation = Vector3.zero;

            bool resultOurMarkers = ComputeOurMarkers3D(ref imageLeft, ref imageRight, init, flipCoordinates,
                Optimization.Method.GradientDescent_BLS, resultLeft, resultRight, coordsLeft, coordsRight, idsLeft, idsRight, out float pixelError);
            //bool resultVarjoMarkers = ComputeVarjoMarkers3D();
            varjoMarkers = _varjoMarkers;
            bool resultVarjoMarkers = _varjoMarkers.Count > 0;

            if (resultOurMarkers && resultVarjoMarkers)
            {
                List<System.Tuple<Marker, Marker>> corresponding = new List<System.Tuple<Marker, Marker>>();

                foreach (Marker ourMarker in ourMarkers)
                {
                    foreach (Marker varjoMarker in varjoMarkers)
                    {
                        if (ourMarker.id == varjoMarker.id)
                        {
                            translation += varjoMarker.position - ourMarker.position;
                            corresponding.Add(new System.Tuple<Marker, Marker>(ourMarker, varjoMarker));
                        }
                    }
                }

                if (corresponding.Count <= 0)
                {
                    UnityDebug.LogError("Error: could not find the Varjo to Unity Transformation");
                    return false;
                }

                translation /= (float)corresponding.Count;
                transformation = Matrix4x4.Translate(translation);

                // Compute the error in 2D and 3D
                float error3D = 0.0f, error2D = 0.0f;
                for (int i = 0; i < corresponding.Count; i++)
                {
                    Vector3 p0 = transformation.MultiplyPoint3x4(corresponding[i].Item1.position);
                    Vector3 p1 = corresponding[i].Item2.position;

                    error3D += (transformation.MultiplyPoint3x4(p0 - p1)).sqrMagnitude;

                    float errorLeft = (cameraLeft.GetPixelCoordinates(p0) - cameraLeft.GetPixelCoordinates(p1)).sqrMagnitude;
                    float errorRight = (cameraRight.GetPixelCoordinates(p0) - cameraRight.GetPixelCoordinates(p1)).sqrMagnitude;

                    error2D += errorLeft + errorRight;
                }
                error3D /= (float)corresponding.Count;
                error3D = Mathf.Sqrt(error3D);

                error2D /= (float)corresponding.Count;
                error2D = Mathf.Sqrt(error2D);

                UnityDebug.Log("Success: Found Varjo to Unity Transformation with errors: 3D=" + error3D + "mm, 2D(reprojection distance)=" + error2D + " pixels, 2D(reprojected 3D Varjo markers)=" + pixelError + " pixels");

                return true;
            }
            else
            {
                if (!resultOurMarkers)
                    UnityDebug.LogError("Error: could not estimate 3D marker position from the Varjo glasses!");
                if (!resultVarjoMarkers)
                    UnityDebug.LogError("Error: could not retrieve 3D marker position from Unity/Varjo SDK!");
            }

            return false;
        }

        public List<Marker> GetOurMarkers3D()
        {
            return ourMarkers;
        }
        /// <summary>
        /// Reconstructs 3D positions of detected ArUco markers from stereo 2D detections
        /// by minimizing reprojection error using gradient-based optimization.
        ///
        /// This method establishes left–right marker correspondences by marker ID,
        /// initializes all marker positions to a common 3D guess, and refines them
        /// jointly using stereo reprojection error minimization.
        /// </summary>
        /// <param name="imageLeft">
        /// Left camera image corresponding to the detected 2D marker coordinates.
        /// Passed by reference to avoid unnecessary copying.
        /// </param>
        /// <param name="imageRight">
        /// Right camera image corresponding to the detected 2D marker coordinates.
        /// Passed by reference to avoid unnecessary copying.
        /// </param>
        /// <param name="init">
        /// Initial 3D position estimate for all markers, used to initialize the optimizer.
        /// This should be reasonably close to the true marker positions to ensure convergence.
        /// </param>
        /// <param name="flipCoordinates">
        /// If true, vertically flips 2D image coordinates before reprojection error computation
        /// to match the internal camera coordinate convention.
        /// </param>
        /// <param name="optimizationMethod">
        /// Optimization strategy used to minimize reprojection error
        /// (e.g., gradient descent with backtracking line search).
        /// </param>
        /// <param name="resultLeft">
        /// True if 2D marker detection on the left image succeeded.
        /// </param>
        /// <param name="resultRight">
        /// True if 2D marker detection on the right image succeeded.
        /// </param>
        /// <param name="coordsLeft">
        /// Array of 2D pixel coordinates of detected markers in the left image.
        /// The order must correspond to <paramref name="idsLeft"/>.
        /// </param>
        /// <param name="coordsRight">
        /// Array of 2D pixel coordinates of detected markers in the right image.
        /// The order must correspond to <paramref name="idsRight"/>.
        /// </param>
        /// <param name="idsLeft">
        /// Array of ArUco marker IDs detected in the left image.
        /// </param>
        /// <param name="idsRight">
        /// Array of ArUco marker IDs detected in the right image.
        /// </param>
        /// <param name="pixelError">
        /// OUTPUT — Root-mean-square (RMS) reprojection error after optimization,
        /// reported in pixels. Lower values indicate better geometric consistency.
        /// </param>
        ///
        /// <returns>
        /// True if 3D marker reconstruction and optimization succeed; false otherwise.
        /// On failure, <paramref name="pixelError"/> is set to <see cref="float.MaxValue"/>.
        /// </returns>
        ///
        /// <seealso cref="Calibrate"/>
        /// <seealso cref="GetEnergy"/>
        /// <seealso cref="Optimization"/>
        public bool ComputeOurMarkers3D(ref Texture2D imageLeft, ref Texture2D imageRight, Vector3 init, bool flipCoordinates, Optimization.Method optimizationMethod,
            bool resultLeft, bool resultRight, Vector2[] coordsLeft, Vector2[] coordsRight, long[] idsLeft, long[] idsRight, out float pixelError)
        {
            //Stopwatch sw = new Stopwatch();
            //sw.Start();
            // clean up
            ourMarkers.Clear();
            ourMarkerCorrespondences.Clear();

            // detect...
            //bool resultLeft = DetectMarkers2D(ref imageLeft, out Vector2[] coordsLeft, out long[] idsLeft);
            //bool resultRight = DetectMarkers2D(ref imageRight, out Vector2[] coordsRight, out long[] idsRight);

            if (resultLeft && resultRight)
            {
                ourMarkerCorrespondences = new List<System.Tuple<Vector2, Vector2>>();
                List<long> idsList = new List<long>();

                for (int i = 0; i < idsLeft.Length; i++)
                    for (int j = 0; j < idsRight.Length; j++)
                        if (idsLeft[i] == idsRight[j])
                        {
                            idsList.Add(idsLeft[i]);

                            if (flipCoordinates)
                                ourMarkerCorrespondences.Add(new System.Tuple<Vector2, Vector2>(new Vector2(coordsLeft[i].x, cameraLeft.height - coordsLeft[i].y), new Vector2(coordsRight[j].x, 1152 - coordsRight[j].y)));
                            else
                                ourMarkerCorrespondences.Add(new System.Tuple<Vector2, Vector2>(coordsLeft[i], coordsRight[j]));

                        }

                if (idsList.Count <= 0)
                {
                    UnityDebug.LogError("Error: No Aruco Marker was found simultaneously on the left and right view!");
                    pixelError = float.MaxValue;
                    return false;
                }

                ourMarkers = new List<Marker>();
                for (int i = 0; i < idsList.Count; ++i)
                    ourMarkers.Add(new Marker(idsList[i], init));

                Optimization.Settings settings = new Optimization.Settings(ourMarkers.Count * 3);
                settings.SetMaxMinIter(5, 2000);
                settings.SetMaxStep(0.01f); // 1 centimeter
                settings.SetInitStep(0.01f); // 1 centimeter, equal to maxSteps
                settings.SetEpsilon(1e-7f);
                settings.debugMode = debugMode;
                Optimization optimize = new Optimization(settings, GetEnergy, SetParameters);

                float[] initParams = new float[ourMarkers.Count * 3];
                for (int i = 0; i < ourMarkers.Count; ++i)
                {
                    initParams[i * 3 + 0] = init.x;
                    initParams[i * 3 + 1] = init.y;
                    initParams[i * 3 + 2] = init.z;
                }

                optimize.SetInitParameters(initParams);
                optimize.Optimize(optimizationMethod);

                // Compute pixel error
                float[] unused = new float[0];
                pixelError = Mathf.Sqrt(-GetEnergy(ref unused, false) / ourMarkers.Count);

                //sw.Stop();
                // UnityDebug.Log($"⏱ ComputeOurMarkers3D Time: {sw.ElapsedMilliseconds} ms");
                return true;
            }
            if (!resultLeft)
                UnityDebug.LogError("Error: could not find Aruco markers on the left view");
            if (!resultRight)
                UnityDebug.LogError("Error: could not find Aruco markers on the right view");

            pixelError = float.MaxValue;
            return false;
        }

        private float GetEnergy(ref float[] gradient, bool compute_gradient = true)
        {
            float energy = 0.0f;

            if (compute_gradient)
                System.Array.Clear(gradient, 0, gradient.Length);

            for (int i = 0; i < ourMarkers.Count; i++)
            {
                Vector2 coordsLeft = UnityUtils.fromHomogeneous(cameraLeft.GetPixelCoordinates(ourMarkers[i].position));
                Vector2 diffLeft = coordsLeft - ourMarkerCorrespondences[i].Item1;
                energy += diffLeft.sqrMagnitude;

                Vector2 coordsRight = UnityUtils.fromHomogeneous(cameraRight.GetPixelCoordinates(ourMarkers[i].position));
                Vector2 diffRight = coordsRight - ourMarkerCorrespondences[i].Item2;
                energy += diffRight.sqrMagnitude;

                if (compute_gradient)
                {
                    Vector3 Px = cameraLeft.P.MultiplyPoint3x4(ourMarkers[i].position);

                    Vector2 derivX = new Vector2((cameraLeft.P[0, 0] - Px.x * cameraLeft.P[2, 0]), // Simplified by removing division by Px.z (considered as fixed multiplication factor)
                                                 (cameraLeft.P[1, 0] - Px.y * cameraLeft.P[2, 0]));
                    Vector2 derivY = new Vector2((cameraLeft.P[0, 1] - Px.x * cameraLeft.P[2, 1]),
                                                 (cameraLeft.P[1, 1] - Px.y * cameraLeft.P[2, 1]));
                    Vector2 derivZ = new Vector2((cameraLeft.P[0, 2] - Px.x * cameraLeft.P[2, 2]),
                                                 (cameraLeft.P[1, 2] - Px.y * cameraLeft.P[2, 2]));

                    gradient[i * 3 + 0] -= Mathf.Clamp(2.0f * (Vector2.Dot(diffLeft, derivX) + Vector2.Dot(diffRight, derivX)), -1.0f, 1.0f);
                    gradient[i * 3 + 1] -= Mathf.Clamp(2.0f * (Vector2.Dot(diffLeft, derivY) + Vector2.Dot(diffRight, derivY)), -1.0f, 1.0f);
                    gradient[i * 3 + 2] -= Mathf.Clamp(2.0f * (Vector2.Dot(diffLeft, derivZ) + Vector2.Dot(diffRight, derivZ)), -1.0f, 1.0f);
                }
            }

            //if (compute_gradient)
            //    GetGradientViaFiniteDifferences(ref gradient);

            return -energy;
        }




        private bool SetParameters(float[] p, bool update_gradient = true)
        {
            for (int i = 0; i < ourMarkers.Count; i++)
            {
                ourMarkers[i].position.x = p[i * 3 + 0];
                ourMarkers[i].position.y = p[i * 3 + 1];
                ourMarkers[i].position.z = p[i * 3 + 2];
            }

            return true;
        }

        /// <summary>
        /// Reconstructs and refines 3D positions of ArUco markers from stereo 2D detections
        /// using a two-stage approach: direct geometric triangulation followed by
        /// gradient-descent reprojection error minimization.
        /// Compared to <see cref="ComputeOurMarkers3D"/>, this method uses explicit stereo
        /// triangulation to obtain an initial 3D estimate for each marker before optimization,
        /// resulting in faster convergence and improved robustness.
        /// </summary>
        /// <param name="resultLeft">
        /// True if 2D marker detection on the left image succeeded.
        /// </param>
        /// <param name="resultRight">
        /// True if 2D marker detection on the right image succeeded.
        /// </param>
        /// <param name="coordsLeft">
        /// Array of 2D pixel coordinates of detected markers in the left image.
        /// The order must correspond to <paramref name="idsLeft"/>.
        /// </param>
        /// <param name="coordsRight">
        /// Array of 2D pixel coordinates of detected markers in the right image.
        /// The order must correspond to <paramref name="idsRight"/>.
        /// </param>
        /// <param name="idsLeft">
        /// Array of ArUco marker IDs detected in the left image.
        /// </param>
        /// <param name="idsRight">
        /// Array of ArUco marker IDs detected in the right image.
        /// </param>
        /// <param name="optimizationMethod">
        /// Optimization strategy used to minimize reprojection error
        /// (e.g., gradient descent with backtracking line search).
        /// </param>
        /// <param name="pixelError">
        /// OUTPUT — Root-mean-square (RMS) reprojection error after optimization,
        /// reported in pixels. Lower values indicate better geometric consistency.
        /// </param>
        /// <returns>
        /// True if at least one marker was successfully triangulated and optimized;
        /// false otherwise. On failure, <paramref name="pixelError"/> is set to
        /// <see cref="float.MaxValue"/>.
        /// </returns>
        //public bool ComputeOurMarkers3D_TriGD(
        //                bool resultLeft, bool resultRight,
        //                Vector2[] coordsLeft, Vector2[] coordsRight,
        //                long[] idsLeft, long[] idsRight,
        //                Optimization.Method optimizationMethod,
        //                out float pixelError
        //                )
        //{

        //    //Stopwatch sw = new Stopwatch();
        //    //sw.Start();

        //    pixelError = float.MaxValue;
        //    ourMarkers.Clear();
        //    ourMarkerCorrespondences.Clear();
        //    bool flipCoordinates = true;
        //    if (!resultLeft || !resultRight ||
        //        coordsLeft == null || coordsRight == null ||
        //        idsLeft == null || idsRight == null)
        //    {
        //        UnityDebug.LogError("Error: No marker detection results available!");
        //        return false;
        //    }

        //    List<long> matchedIds;
        //    List<Vector2> matchedLeft, matchedRight;

        //    // 1️ Triangulation + ID matching
        //    Vector3[] triangulatedPoints = TriangulateFromDetections(
        //        resultLeft, coordsLeft, idsLeft,
        //        resultRight, coordsRight, idsRight,
        //        out matchedIds,
        //        out matchedLeft,
        //        out matchedRight
        //    );

        //    if (triangulatedPoints.Length == 0)
        //    {
        //        UnityDebug.LogError("Error: No matching markers to triangulate!");
        //        return false;
        //    }

        //    // 2️ Build marker + correspondence structure
        //    int N = triangulatedPoints.Length;
        //    for (int i = 0; i < N; i++)
        //    {
        //        ourMarkers.Add(new Marker(matchedIds[i], triangulatedPoints[i]));
        //        if (true)
        //        {
        //            ourMarkerCorrespondences.Add(new Tuple<Vector2, Vector2>(
        //                new Vector2(matchedLeft[i].x, 1152 - matchedLeft[i].y),
        //                new Vector2(matchedRight[i].x, 1152 - matchedRight[i].y)
        //            ));
        //        }
        //        else
        //        {
        //            ourMarkerCorrespondences.Add(new Tuple<Vector2, Vector2>(
        //                matchedLeft[i], matchedRight[i]
        //            ));
        //        }
        //    }

        //    // 3️ Optimization setup
        //    Optimization.Settings settings = new Optimization.Settings(N * 3);
        //    settings.SetMaxMinIter(5, 2000);
        //    settings.SetMaxStep(0.005f); // 1 centimeter
        //    settings.SetInitStep(0.02f);
        //    settings.SetEpsilon(1e-7f);
        //    settings.debugMode = debugMode;

        //    Optimization opt = new Optimization(settings, GetEnergy, SetParameters);

        //    float[] initParams = new float[N * 3];
        //    for (int i = 0; i < N; i++)
        //    {
        //        initParams[i * 3 + 0] = triangulatedPoints[i].x;
        //        initParams[i * 3 + 1] = triangulatedPoints[i].y;
        //        initParams[i * 3 + 2] = triangulatedPoints[i].z;
        //    }
        //    opt.SetInitParameters(initParams);

        //    // 4️ Gradient descent refinement
        //    opt.Optimize(optimizationMethod);

        //    // 5️ Compute reprojection RMS pixel error
        //    float[] unused = new float[0];
        //    pixelError = Mathf.Sqrt(-GetEnergy(ref unused, false) / N);

        //    //sw.Stop();
        //    //UnityDebug.Log($"✔ Tri+Optimize Time: {sw.ElapsedMilliseconds} ms");

        //    return true;
        //}



        unsafe private Vector3[] TriangulateFromDetections(
                            bool resultLeft, Vector2[] coordsLeft, long[] idsLeft,
                            bool resultRight, Vector2[] coordsRight, long[] idsRight,
                            out List<long> matchedIds,
                            out List<Vector2> matchedLeft,
                            out List<Vector2> matchedRight)
        {

            matchedIds = new List<long>();
            matchedLeft = new List<Vector2>();
            matchedRight = new List<Vector2>();

            if (!resultLeft || !resultRight ||
                coordsLeft == null || coordsRight == null ||
                idsLeft == null || idsRight == null)
                return new Vector3[0];

            // Build map from ID to right-eye pixel coordinates
            Dictionary<long, Vector2> rightMap = new Dictionary<long, Vector2>();
            for (int j = 0; j < idsRight.Length; j++)
                rightMap[idsRight[j]] = coordsRight[j];

            // Pair by ID using map (no duplicates, correct matches)
            for (int i = 0; i < idsLeft.Length; i++)
            {
                long id = idsLeft[i];
                if (rightMap.TryGetValue(id, out Vector2 ptR))
                {
                    matchedIds.Add(id);
                    matchedLeft.Add(coordsLeft[i]);
                    matchedRight.Add(ptR);
                }
            }

            if (matchedLeft.Count == 0)
                return new Vector3[0];

            return TriangulatePoints(matchedLeft.ToArray(), matchedRight.ToArray(), cameraLeft.Rt, cameraRight.Rt);
        }


        public unsafe Vector3[] TriangulatePoints(Vector2[] coordsLeft, Vector2[] coordsRight,
                                                    Matrix4x4 RtL_world,
                                                    Matrix4x4 RtR_world)
        {
            int count = Mathf.Min(coordsLeft.Length, coordsRight.Length);

            if (count == 0)
                return new Vector3[0];
            //UnityDebug.Log(
            //    $"[TriangulatePoints] count={count}\n" );
            //PrintVector2Array("coordsLeft", coordsLeft);
            //PrintVector2Array("coordsRight", coordsRight);
            // ---------- FIXED PACKING FORMAT ----------
            float[] ptsL = new float[count * 2];
            float[] ptsR = new float[count * 2];

            for (int i = 0; i < count; i++)
            {
                // First all X then all Y
                ptsL[i] = coordsLeft[i].x;
                ptsL[i + count] = coordsLeft[i].y;

                ptsR[i] = coordsRight[i].x;
                ptsR[i + count] = coordsRight[i].y;
            }


            // Rt = WORLD -> CAMERA transforms from Varjo
            //Matrix4x4 RtL_world = cameraLeft.Rt;
            //Matrix4x4 RtR_world = cameraRight.Rt;

            // Make right relative to left:
            // camR -> world -> camL
            Matrix4x4 RtR_left = RtL_world * RtR_world.inverse;

            // Left camera is stereo origin
            Matrix4x4 P_left = cameraLeft.K * Matrix4x4.identity;
            Matrix4x4 P_right = cameraRight.K * RtR_left;


            float[] P1_f = MatrixToFloatArray(P_left);
            float[] P2_f = MatrixToFloatArray(P_right);


            float* raw = OpenCV.Triangulate2DPoints(
                ptsL, ptsR, (uint)count, P1_f, P2_f
            );

            if (raw == null)
                return new Vector3[0];

            // CONVERT LEFT CAMERA -> VARJO WORLD 
            Matrix4x4 T_left_to_world = RtL_world.inverse;
            Vector3[] worldPoints = new Vector3[count];

            for (int i = 0; i < count; i++)
            {
                // Extract LEFT camera space
                Vector3 X_left = new Vector3(
                    raw[i * 3 + 0],
                    raw[i * 3 + 1],
                    raw[i * 3 + 2]
                );
                Vector3 X_world = T_left_to_world.MultiplyPoint3x4(X_left);

                // ===== Flip Z-axis so positive Z is forward (toward the scene) =====
                X_world.z *= -1f;
                worldPoints[i] = X_world;

            }

            return worldPoints;
        }


        private float[] MatrixToFloatArray(Matrix4x4 mat)
        {
            return new float[]
            {
                mat[0,0], mat[0,1], mat[0,2], mat[0,3],
                mat[1,0], mat[1,1], mat[1,2], mat[1,3],
                mat[2,0], mat[2,1], mat[2,2], mat[2,3]
            };
        }
        private void PrintFloatArray(string name, float[] arr)
        {
            if (arr == null)
            {
                UnityDebug.Log(name + " is null");
                return;
            }

            string s = name + " =\n";
            for (int i = 0; i < 3; i++)
            {
                s += $"{arr[i * 4 + 0],10:F4}  {arr[i * 4 + 1],10:F4}  {arr[i * 4 + 2],10:F4}  {arr[i * 4 + 3],10:F4}\n";
            }
            UnityDebug.Log(s);
        }

        /// <summary>
        /// Reconstructs and refines 3D quad corner positions from stereo 2D corner detections
        /// using batch triangulation followed by gradient-descent reprojection error minimization.
        /// Each quad is triangulated from corresponding left–right 2D corner detections,
        /// then independently optimized in 3D to minimize stereo reprojection error.
        /// </summary>
        /// <param name="pairs">
        /// Collection of stereo corner detections, where each entry
        /// contains an object identifier and corresponding left-eye
        /// and right-eye <see cref="Corner2DInfo"/> instances.
        /// </param>
        /// <param name="leftExtrinsics">
        /// World-to-left-camera extrinsic transformation matrix.
        /// </param>
        /// <param name="rightExtrinsics">
        /// World-to-right-camera extrinsic transformation matrix.
        /// </param>
        /// <param name="optimizationMethod">
        /// Optimization strategy used to minimize reprojection error
        /// (e.g., Optimization.Method.GradientDescent_BLS_Fast).
        /// </param>
        /// <param name="refinedQuads">
        /// OUTPUT — Array of refined 3D quads after triangulation and optimization.
        /// Each quad contains four optimized 3D corner positions and the corresponding 2D observations.
        /// </param>
        /// <param name="pixelError">
        /// OUTPUT — Average root-mean-square (RMS) reprojection error over all optimized quads,
        /// reported in pixels. Lower values indicate better geometric consistency.
        /// </param>
        /// <returns>
        /// True if at least one quad was successfully triangulated and optimized; false otherwise.
        /// </returns>
        public bool Compute3D_TriGD(
                    List<(int id, Corner2DInfo left, Corner2DInfo right)> pairs,
                    Matrix4x4 leftExtrinsics,
                    Matrix4x4 rightExtrinsics,
                    Optimization.Method optimizationMethod,
                    out Corner3DQuad[] refinedQuads,
                    out float pixelError)
        {
            refinedQuads = null;
            pixelError = float.MaxValue;

            if (pairs == null || pairs.Count == 0)
            {
                UnityDebug.LogError("Error: No detection results available!");
                return false;
            }

            // 1 triangulate
            Corner3DQuad[] quads = TriangulateFromCornerDetections(pairs, leftExtrinsics, rightExtrinsics, flipVertical: true);
            if (quads.Length == 0)
            {
                UnityDebug.LogError("Error: No matching markers to triangulate!");
                return false;
            }

            //PrintCorner3DQuads(quads, "[3D]");

            float totalError = 0f;
            //UnityDebug.Log($"Info: Triangulated {quads.Length} quads, starting optimization...");
            // 2 optimize each quad
            for (int i = 0; i < quads.Length; i++)
            {
                float quadError;
                Corner3DQuad resultQuad;

                bool ok = OptimizeQuad3D(
                    quads[i],
                    optimizationMethod,
                    out resultQuad,
                    out quadError);

                if (!ok)
                    UnityDebug.LogWarning($"Warn: Quad {quads[i].Id} failed optimization");

                quads[i] = resultQuad;

                totalError += quadError;
            }

            // 3 average error
            pixelError = totalError / quads.Length;

            refinedQuads = quads;
            return true;
        }
        /// <summary>
        /// Performs batch stereo triangulation of quad corner detections from left and right images
        /// and constructs 3D quad representations with associated 2D observations.
        ///
        /// For each quad detected in the left image, the corresponding quad in the right image
        /// is matched by ID, its four corners are triangulated jointly, and the resulting 3D quad
        /// is returned together with the (optionally flipped) 2D corner coordinates.
        /// </summary>
        /// <param name="left">
        /// List of quad corner detections from the left image.
        /// Each entry must contain four ordered 2D corner points.
        /// </param>
        /// <param name="right">
        /// List of quad corner detections from the right image.
        /// Each entry must contain four ordered 2D corner points.
        /// </param>
        /// <param name="leftExtrinsics">
        /// World-to-left-camera extrinsic transformation matrix.
        /// </param>
        /// <param name="rightExtrinsics">
        /// World-to-right-camera extrinsic transformation matrix.
        /// </param>
        /// <param name="flipVertical">
        /// If true, vertically flips 2D image coordinates (y' = imageHeight − y) when storing
        /// 2D observations for reprojection and optimization.
        /// </param>
        /// <returns>
        /// An array of <see cref="Corner3DQuad"/> objects representing successfully triangulated
        /// quads. The array may be empty if no valid quad correspondences were found.
        /// </returns>
        public Corner3DQuad[] TriangulateFromCornerDetections(
                                List<(int id, Corner2DInfo left, Corner2DInfo right)> pairs,
                                Matrix4x4 leftExtrinsics,
                                Matrix4x4 rightExtrinsics,
                                bool flipVertical = true)
        {

            if (pairs == null || pairs.Count == 0)
                return new Corner3DQuad[0];
            List<Corner3DQuad> results = new List<Corner3DQuad>(pairs.Count);

            foreach ((int id, Corner2DInfo left, Corner2DInfo right) p in pairs)
            {

                if (p.left.Points == null || p.right.Points == null)
                    continue;
                if (p.left.Points.Length != 4 || p.right.Points.Length != 4)
                    continue;

                Vector3[] four3D = TriangulatePoints(
                        p.left.Points,
                        p.right.Points,
                        leftExtrinsics,
                        rightExtrinsics
                    );
                //UnityDebug.Log($"[Triangulate] QuadId={L.Id} four3D={FormatVec3Array(four3D)}");
                if (four3D == null || four3D.Length != 4)
                    continue;

                Vector2[] left2D = new Vector2[4];
                Vector2[] right2D = new Vector2[4];

                for (int k = 0; k < 4; k++)
                {
                    Vector2 pL = p.left.Points[k];
                    Vector2 pR = p.right.Points[k];

                    if (flipVertical)
                    {
                        left2D[k] = new Vector2(pL.x, cameraLeft.height - pL.y);
                        right2D[k] = new Vector2(pR.x, cameraRight.height - pR.y);
                    }
                    else
                    {
                        left2D[k] = pL;
                        right2D[k] = pR;
                    }
                }

                results.Add(new Corner3DQuad(p.id, four3D, left2D, right2D));
            }

            return results.ToArray();
        }

        /// <summary>
        /// Initializes and configures the optimization settings used for per-quad 3D refinement.
        /// This method should init at the beginning once. 
        /// </summary>
        public void InitQuadSettings()
        {
            const int D = 12;  // 4 corners * 3 coordinates
            quadSettings = new Optimization.Settings(D);   // store into the FIELD, not a local variable
            //quadSettings.SetMaxMinIter(5, 2000);
            //quadSettings.SetMaxStep(0.005f);
            //quadSettings.SetInitStep(0.02f);
            //quadSettings.SetEpsilon(1e-7f);
            //best:
            quadSettings.SetMaxMinIter(20, 4000);
            quadSettings.SetMaxStep(0.0015f);
            quadSettings.SetInitStep(0.001f);
            quadSettings.SetEpsilon(1e-9f);

            quadSettings.debugMode = false;
        }






        /// <summary>
        /// Refines the 3D corner positions of a single quad by minimizing stereo reprojection error
        /// using gradient-based optimization.
        /// Starting from an initial 3D estimate for the four quad corners, this method independently
        /// optimizes the quad geometry to achieve better left–right reprojection consistency.
        /// </summary>
        /// <param name="quad">
        /// Input quad containing initial 3D corner positions and corresponding left and right
        /// 2D observations.
        /// </param>
        /// <param name="method">
        /// Optimization strategy used to minimize reprojection error
        /// (e.g., Optimization.Method.GradientDescent_BLS_Fast).
        /// </param>
        /// <param name="resultQuad">
        /// OUTPUT — The refined quad after optimization, containing updated 3D corner positions
        /// and the original 2D observations.
        /// </param>
        /// <param name="pixelError">
        /// OUTPUT — Root-mean-square (RMS) reprojection error over the four quad corners,
        /// reported in pixels. Lower values indicate better reprojection consistency.
        /// </param>
        /// <returns>
        /// True if the quad was successfully optimized; false if validation fails,
        /// optimization settings are not initialized, or an exception occurs.
        /// </returns>
        public bool OptimizeQuad3D(
                Corner3DQuad quad,
                 Optimization.Method method,
                 out Corner3DQuad resultQuad,
                 out float pixelError
                )
        {

            resultQuad = quad;
            pixelError = float.MaxValue;

            try
            {
                // validate input
                if (quad.Points3D == null || quad.Points3D.Length != 4)
                {
                    UnityDebug.LogError($"Quad {quad.Id}: invalid Points3D");
                    return false;
                }
                if (quad.Left2D == null || quad.Left2D.Length != 4 ||
                    quad.Right2D == null || quad.Right2D.Length != 4)
                {
                    UnityDebug.LogError($"Quad {quad.Id}: invalid 2D points");
                    return false;
                }

                // 1) copy into working buffers
                for (int c = 0; c < 4; c++)
                {
                    quadCurrent[c] = quad.Points3D[c];
                    quadObsLeft[c] = quad.Left2D[c];
                    quadObsRight[c] = quad.Right2D[c];
                }

                // 2) initial params
                float[] initParams = new float[12];
                for (int c = 0; c < 4; c++)
                {
                    initParams[c * 3 + 0] = quad.Points3D[c].x;
                    initParams[c * 3 + 1] = quad.Points3D[c].y;
                    initParams[c * 3 + 2] = quad.Points3D[c].z;
                    //initParams[c * 3 + 0] = 0f;
                    //initParams[c * 3 + 1] = 0f;
                    //initParams[c * 3 + 2] = 0.3f;
                }

                // 3) optimizer
                if (quadSettings == null)
                {
                    UnityDebug.LogError("OptimizeQuad3D: quadSettings is not initialized.");
                    return false;
                }

                Optimization opt = new Optimization(
                    quadSettings,
                    GetQuadEnergy,
                    SetQuadParameters);

                opt.SetInitParameters(initParams);

                opt.Optimize(method);

                // 4) compute RMS pixel error
                float[] unused = new float[0];
                pixelError = Mathf.Sqrt(-GetQuadEnergy(ref unused, false) / 4f);

                // 5) build refined quad
                resultQuad = new Corner3DQuad(
                    quad.Id,
                    new Vector3[]
                    {
                            quadCurrent[0],
                            quadCurrent[1],
                            quadCurrent[2],
                            quadCurrent[3]
                    },
                    quad.Left2D,
                    quad.Right2D);

                return true;
            }
            catch (System.Exception ex)
            {
                UnityDebug.LogError($"OptimizeQuad3D failed for marker {quad.Id}. Reason: {ex.Message}");
                return false;
            }

        }

        private float GetQuadEnergy(ref float[] gradient, bool compute_gradient = true)
        {
            float energy = 0f;

            if (compute_gradient)
                System.Array.Clear(gradient, 0, gradient.Length);

            // loop over 4 corners
            for (int c = 0; c < 4; c++)
            {
                // project corner 3D -> left image
                Vector2 coordsLeft = UnityUtils.fromHomogeneous(
                    cameraLeft.GetPixelCoordinates(quadCurrent[c]));
                Vector2 diffLeft = coordsLeft - quadObsLeft[c];
                energy += diffLeft.sqrMagnitude;

                // project corner 3D -> right image
                Vector2 coordsRight = UnityUtils.fromHomogeneous(
                    cameraRight.GetPixelCoordinates(quadCurrent[c]));
                Vector2 diffRight = coordsRight - quadObsRight[c];
                energy += diffRight.sqrMagnitude;

                if (compute_gradient)
                {
                    // derivative for left camera
                    Vector3 PxL = cameraLeft.P.MultiplyPoint3x4(quadCurrent[c]);

                    Vector2 derivXL = new Vector2((cameraLeft.P[0, 0] - PxL.x * cameraLeft.P[2, 0]),
                                                  (cameraLeft.P[1, 0] - PxL.y * cameraLeft.P[2, 0]));
                    Vector2 derivYL = new Vector2((cameraLeft.P[0, 1] - PxL.x * cameraLeft.P[2, 1]),
                                                  (cameraLeft.P[1, 1] - PxL.y * cameraLeft.P[2, 1]));
                    Vector2 derivZL = new Vector2((cameraLeft.P[0, 2] - PxL.x * cameraLeft.P[2, 2]),
                                                  (cameraLeft.P[1, 2] - PxL.y * cameraLeft.P[2, 2]));

                    // derivative for right camera
                    Vector3 PxR = cameraRight.P.MultiplyPoint3x4(quadCurrent[c]);

                    Vector2 derivXR = new Vector2((cameraRight.P[0, 0] - PxR.x * cameraRight.P[2, 0]),
                                                  (cameraRight.P[1, 0] - PxR.y * cameraRight.P[2, 0]));
                    Vector2 derivYR = new Vector2((cameraRight.P[0, 1] - PxR.x * cameraRight.P[2, 1]),
                                                  (cameraRight.P[1, 1] - PxR.y * cameraRight.P[2, 1]));
                    Vector2 derivZR = new Vector2((cameraRight.P[0, 2] - PxR.x * cameraRight.P[2, 2]),
                                                  (cameraRight.P[1, 2] - PxR.y * cameraRight.P[2, 2]));

                    // accumulate gradient contribution for corner c
                    gradient[c * 3 + 0] -= Mathf.Clamp(
                        2.0f * (Vector2.Dot(diffLeft, derivXL) + Vector2.Dot(diffRight, derivXR)),
                        -1f, 1f);
                    gradient[c * 3 + 1] -= Mathf.Clamp(
                        2.0f * (Vector2.Dot(diffLeft, derivYL) + Vector2.Dot(diffRight, derivYR)),
                        -1f, 1f);
                    gradient[c * 3 + 2] -= Mathf.Clamp(
                        2.0f * (Vector2.Dot(diffLeft, derivZL) + Vector2.Dot(diffRight, derivZR)),
                        -1f, 1f);
                }
            }

            return -energy;
        }


        private bool SetQuadParameters(float[] p, bool update_gradient = true)
        {
            // p length = 12
            for (int c = 0; c < 4; c++)
            {
                quadCurrent[c].x = p[c * 3 + 0];
                quadCurrent[c].y = p[c * 3 + 1];
                quadCurrent[c].z = p[c * 3 + 2];
            }
            return true;
        }






    }


}