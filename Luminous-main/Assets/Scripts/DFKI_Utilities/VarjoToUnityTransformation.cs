using UnityEngine;
using System.Collections.Generic;


namespace DFKI_Utilities
{
    public class VarjoToUnityTransformation
    {
        // input settings
        private CameraView cameraLeft;
        private CameraView cameraRight;
        public bool debugMode = true;

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

        public VarjoToUnityTransformation(ref CameraView _cameraLeft, ref CameraView _cameraRight, bool _debugMode = true)
        {
            cameraLeft = _cameraLeft;
            cameraRight = _cameraRight;
            debugMode = _debugMode;
        }

        public Matrix4x4 GetTransformation()
        {
            return transformation;
        }

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
                    Debug.LogError("Error: could not find the Varjo to Unity Transformation");
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

                Debug.Log("Success: Found Varjo to Unity Transformation with errors: 3D=" + error3D + "mm, 2D(reprojection distance)=" + error2D + " pixels, 2D(reprojected 3D Varjo markers)=" + pixelError + " pixels");

                return true;
            }
            else
            {
                if (!resultOurMarkers)
                    Debug.LogError("Error: could not estimate 3D marker position from the Varjo glasses!");
                if (!resultVarjoMarkers)
                    Debug.LogError("Error: could not retrieve 3D marker position from Unity/Varjo SDK!");
            }

            return false;
        }

        public List<Marker> GetOurMarkers3D()
        {
            return ourMarkers;
        }

        public bool ComputeOurMarkers3D(ref Texture2D imageLeft, ref Texture2D imageRight, Vector3 init, bool flipCoordinates, Optimization.Method optimizationMethod,
            bool resultLeft, bool resultRight, Vector2[] coordsLeft, Vector2[] coordsRight, long[] idsLeft, long[] idsRight, out float pixelError)
        {
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
                                ourMarkerCorrespondences.Add(new System.Tuple<Vector2, Vector2>(new Vector2(coordsLeft[i].x, 1152 - coordsLeft[i].y), new Vector2(coordsRight[j].x, 1152 - coordsRight[j].y)));
                            else
                                ourMarkerCorrespondences.Add(new System.Tuple<Vector2, Vector2>(coordsLeft[i], coordsRight[j]));

                        }

                if (idsList.Count <= 0)
                {
                    Debug.LogError("Error: No Aruco Marker was found simultaneously on the left and right view!");
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

                return true;
            }
            if (!resultLeft)
                Debug.LogError("Error: could not find Aruco markers on the left view");
            if (!resultRight)
                Debug.LogError("Error: could not find Aruco markers on the right view");

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
    }

}