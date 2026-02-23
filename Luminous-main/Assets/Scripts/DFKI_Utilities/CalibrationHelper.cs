using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using Varjo.XR;
using DFKI_Utilities;
using static DFKI_Utilities.VarjoToUnityCalibration;
using static DFKI_Utilities.DataStructure;
namespace DFKI_Utilities
{


    public static class CalibrationHelper
    {
        // ---------------------------
        //  OpenCV wrapper
        // ---------------------------
        private static class OpenCV
        {
            [DllImport("OpenCV")]
            unsafe public static extern float* DetectArucoMarkers(byte[] img, int width, int height,
                                                                  int numChannels, int bitsPerChannel, int dictionary);

            [DllImport("OpenCV")]
            unsafe public static extern float* Triangulate2DPoints(float[] coordLeft, float[] coordRight,
                                                                   uint size, float[] projMatrixLeft, float[] projMatrixRight);
        }



        /// <summary>
        /// Waits for a given number of frames before invoking an optional callback.
        /// Typically used to allow camera pipelines or rendering to stabilize
        /// before starting calibration.
        /// </summary>
        public static IEnumerator WarmupFrames(int frames, Action onComplete = null)
        {
            for (int i = 0; i < frames; i++)
                yield return null;

            onComplete?.Invoke();
        }


        /// <summary>
        /// Detects ArUco markers in the specified <see cref="Texture2D"/> by invoking the native
        /// OpenCV-based marker detection function.
        /// </summary>
        /// <remarks>
        /// The method extracts raw pixel data from the texture and forwards it to the native plugin.
        /// The plugin returns the number of detected markers, followed by a structured array containing
        /// marker identifiers and four corner coordinates per marker. The center position of each marker
        /// is computed from the average of its corner coordinates and returned as output.
        ///
        /// Expected native data layout:
        /// • raw[0]                     → Marker count (N)
        /// • raw[1 + i*9]               → Marker ID (float, cast to long)
        /// • raw[2 + i*9..9 + i*9]      → Corner coordinates (x0, y0, x1, y1, x2, y2, x3, y3)
        ///
        /// The method requires that the input texture provides readable pixel data. On failure or if no
        /// markers are detected, both output arrays are set to zero length.
        /// </remarks>
        /// <param name="image">The input texture. Must be CPU-readable.</param>
        /// <param name="coords">
        /// Output array of marker center coordinates in pixel space. Empty if detection fails or yields no markers.
        /// </param>
        /// <param name="ids">
        /// Output array of detected marker IDs. Aligned by index with <paramref name="coords"/>.
        /// </param>
        /// <returns>
        /// <c>true</c> if one or more markers are detected; otherwise, <c>false</c>.
        /// </returns>
        public static bool DetectMarkers2D(Texture2D image, out Vector2[] coords, out long[] ids)
        {

            int nMarkers = 0;
            float[] raw;

            coords = new Vector2[0];
            ids = new long[0];

            unsafe
            {
                float* raw_result = OpenCV.DetectArucoMarkers(
                    image.GetRawTextureData(),
                    image.width,
                    image.height,
                    Writer.GetNumChannels(ref image),
                    Writer.GetBitsPerChannel(ref image),
                    -1);

                nMarkers = (int)raw_result[0];

                if (nMarkers <= 0)
                    return false;

                int size = 1 + nMarkers * 9;
                raw = new float[size];
                Marshal.Copy((IntPtr)raw_result, raw, 0, size);
            }

            coords = new Vector2[nMarkers];
            ids = new long[nMarkers];

            for (int i = 0; i < nMarkers; ++i)
            {
                int baseIdx = 1 + i * 9;
                ids[i] = (long)raw[baseIdx];

                Vector2 c0 = new Vector2(raw[baseIdx + 1], raw[baseIdx + 2]);
                Vector2 c1 = new Vector2(raw[baseIdx + 3], raw[baseIdx + 4]);
                Vector2 c2 = new Vector2(raw[baseIdx + 5], raw[baseIdx + 6]);
                Vector2 c3 = new Vector2(raw[baseIdx + 7], raw[baseIdx + 8]);

                coords[i] = (c0 + c1 + c2 + c3) / 4f;
            }

            return true;
        }

        /// <summary>
        /// Converts a list of 2D bounding boxes into corner-coordinate structures.
        /// </summary>
        /// <remarks>
        /// Each bounding box in the input list must contain at least four float values representing:
        /// <code>
        /// [x, y, width, height]
        /// </code>
        /// For each valid bounding box, four corner coordinates are generated in clockwise order:
        /// <c>(x, y)</c>, <c>(x + width, y)</c>, <c>(x + width, y + height)</c>, and <c>(x, y + height)</c>.
        /// Entries with missing or insufficient box data are skipped. If no valid bounding boxes are found,
        /// the resulting output list remains empty and the function returns <c>false</c>.
        /// </remarks>
        /// <param name="input">
        /// Input collection of bounding box objects. May be null or empty.
        /// </param>
        /// <param name="output">
        /// Output list of <see cref="Corner2DInfo"/> entries created from valid bounding boxes.
        /// Empty if the method returns <c>false</c>.
        /// </param>
        /// <returns>
        /// <c>true</c> if at least one valid bounding box is converted; otherwise, <c>false</c>.
        /// </returns>
        public static bool GetBBoxesList(
                    IList<BBox> input,
                    out List<Corner2DInfo> output)
        {
            output = new List<Corner2DInfo>();

            if (input == null || input.Count == 0)
                return false;

            foreach (var m in input)
            {
                if (m.Box == null || m.Box.Length < 4)
                    continue;

                float x = m.Box[0];
                float y = m.Box[1];
                float w = m.Box[2];
                float h = m.Box[3];

                Vector2 p0 = new Vector2(x, y);
                Vector2 p1 = new Vector2(x + w, y);
                Vector2 p2 = new Vector2(x + w, y + h);
                Vector2 p3 = new Vector2(x, y + h);

                var pts = new[] { p0, p1, p2, p3 };

                output.Add(new Corner2DInfo(m.Id, pts));
            }

            return output.Count > 0;
        }



        /// <summary>
        /// Retrieves the current set of tracked Varjo 3D markers and converts them into the local
        /// <see cref="Marker"/> format.
        /// </summary>
        /// <remarks>
        /// The method ensures that the Varjo marker tracking system is enabled prior to requesting
        /// marker data from the Varjo SDK. If marker tracking was initially disabled, the method
        /// temporarily enables it and restores the original state after retrieval.
        ///
        /// The function queries the Varjo runtime for all currently visible 3D markers, extracts their
        /// identifiers and pose positions, and returns them as a new list of <see cref="Marker"/> objects.
        /// If no markers are detected, an empty list is returned.
        ///
        /// Logging:
        /// • Outputs a diagnostic message when no markers are present.
        /// </remarks>
        /// <returns>
        /// A list of <see cref="Marker"/> instances, each containing the marker ID and 3D position.
        /// The list may be empty if no markers are currently visible.
        /// </returns>
        /// <dependencies>
        /// Requires access to the Varjo SDK with marker tracking support enabled.
        /// </dependencies>
        public static List<Marker> GetVarjoMarkers3D()
        {


            List<Marker> varjoMarkers = new List<Marker>();

            // Make sure Varjo markers system is enabled
            bool alreadyEnabled = VarjoMarkers.IsVarjoMarkersEnabled();

            if (!alreadyEnabled)
                VarjoMarkers.EnableVarjoMarkers(true);


            // Get a list of markers with up-to-date data.
            List<VarjoMarker> markers = new List<VarjoMarker>();
            VarjoMarkers.GetVarjoMarkers(out markers); //[marker1, m2, m3....]
            Debug.Log($"GetVarjoMarkers3D:{markers.Count}");
            if (markers == null || markers.Count <= 0)
            {
                Debug.Log("GetVarjoMarkers3D: No 3D markers were present.");

                // Restore previous state
                if (!alreadyEnabled)
                    VarjoMarkers.EnableVarjoMarkers(false);

                return varjoMarkers; // empty list
            }

            // Convert VarjoMarker → our Marker type
            for (int i = 0; i < markers.Count; i++)
                varjoMarkers.Add(new Marker(markers[i].id, markers[i].pose.position));

            // Restore previous state
            if (!alreadyEnabled)
                VarjoMarkers.EnableVarjoMarkers(false);

            //Debug.Log($"GetVarjoMarkers3D: Detected {varjoMarkers.Count} Varjo 3D markers.\n");

            return varjoMarkers;
        }


        /// <summary>
        /// Saves rigid transformation matrices between Varjo and Unity coordinate systems to a text file.
        /// </summary>
        /// <remarks>
        /// This function writes two 4×4 homogeneous transformation matrices to disk:
        /// <list type="bullet">
        /// <item>
        /// <description>
        /// <paramref name="fromVarjo_Tansform"/> — 4×4 homogeneous transformation matrix mapping points from the Varjo coordinate system
        /// into the Unity coordinate system.
        /// </description>
        /// </item>
        /// <item>
        /// <description>
        /// <paramref name="fromUnity_Transform"/> — Inverse 4×4 homogeneous transformation matrix mapping points from the Unity coordinate system
        /// back into the Varjo coordinate system.
        /// </description>
        /// </item>
        /// </list>
        /// 
        /// The matrices are written in a human-readable row-wise format, intended for debugging,
        /// offline inspection, and reproducibility of calibration results.
        /// </remarks>
        /// <param name="path">
        /// Full file path where the transformation matrices will be saved. If the file already exists,
        /// it will be overwritten.
        /// </param>
        public static void SaveTransformations(string path, Matrix4x4 fromVarjo_Tansform, Matrix4x4 fromUnity_Transform)
        {

            using StreamWriter writer = new StreamWriter(path);
            writer.WriteLine("From Varjo to Unity Transformation:");
            WriteMatrix(writer, fromVarjo_Tansform);

            writer.WriteLine("From Unity to Varjo Transformation:");
            WriteMatrix(writer, fromUnity_Transform);
        }
        /// <summary>
        /// Writes a 4×4 matrix to a text stream in row-wise format.
        /// </summary>
        /// <remarks>
        /// Each row of the matrix is written on a separate line using four
        /// space-separated floating-point values. The output format is intended
        /// for human readability and for later reconstruction of the matrix
        /// during deserialization.
        /// </remarks>
        /// <param name="w">
        /// Output <see cref="StreamWriter"/> used to write the matrix data.
        /// </param>
        /// <param name="m">
        /// 4×4 <see cref="Matrix4x4"/> to be written to the stream.
        /// </param>
        private static void WriteMatrix(StreamWriter w, Matrix4x4 m)
        {
            for (int r = 0; r < 4; r++)
                w.WriteLine($"{m[r, 0]} {m[r, 1]} {m[r, 2]} {m[r, 3]}");
        }



        /// <summary>
        /// Loads rigid transformation matrices between Varjo and Unity coordinate systems from a text file.
        /// </summary>
        /// <remarks>
        /// This function reads two 4×4 homogeneous transformation matrices from a file previously
        /// written by <c>SaveTransformations</c>. 

        ///  If the file cannot be found, the function returns
        /// <c>false</c> and both output matrices are set to <see cref="Matrix4x4.identity"/>.
        /// </remarks>
        /// <param name="filePath">
        /// Full path to the transformation file on disk.
        /// </param>
        /// <param name="fromVarjoToUnity">
        /// OUTPUT — 4×4 homogeneous transformation matrix mapping points from the Varjo coordinate
        /// system into the Unity coordinate system.
        /// </param>
        /// <param name="fromUnityToVarjo">
        /// OUTPUT — 4×4 homogeneous transformation matrix mapping points from the Unity coordinate
        /// system back into the Varjo coordinate system.
        /// </param>
        /// <returns>
        /// <c>true</c> if the transformation file exists and the matrices were successfully parsed;
        /// <c>false</c> if the file does not exist or could not be read.
        /// </returns>
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
            float[,] mat1 = new float[4, 4];
            float[,] mat2 = new float[4, 4];
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

                string[] tokens = line.Split(new char[] { '\t', ' ' }, StringSplitOptions.RemoveEmptyEntries);
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



        /// <summary>
        /// Saves left and right camera images with overlaid 2D marker coordinates to disk.
        /// </summary>
        /// <remarks>
        /// This function overlays detected or predicted 2D marker positions onto the provided
        /// stereo images and exports the annotated results as PNG files.
        /// When <paramref name="ispredicted"/> is set to <c>true</c>, both the images and the input
        /// coordinates are vertically flipped to convert from a bottom-left origin convention
        /// to a top-left image coordinate system prior to visualization. To make the img direction is the same
        /// </remarks>
        /// <param name="imageLeft">
        /// Input texture representing the left camera image.
        /// </param>
        /// <param name="imageRight">
        /// Input texture representing the right camera image.
        /// </param>
        /// <param name="coordsLeft">
        /// 2D pixel coordinates of markers in the left image. The array length determines
        /// the number of markers drawn.
        /// </param>
        /// <param name="coordsRight">
        /// 2D pixel coordinates of markers in the right image. The array length determines
        /// the number of markers drawn.
        /// </param>
        /// <param name="datatime">
        /// Timestamp or unique identifier appended to the output file names
        /// (e.g., acquisition time or frame index).
        /// </param>
        /// <param name="ispredicted">
        /// Indicates whether the provided coordinates are predicted values. If <c>true</c>,
        /// coordinate and image flipping is applied before rendering to match visualization
        /// conventions.
        /// </param>
        /// <exception cref="System.IO.IOException">
        /// Thrown if the output image files cannot be written to disk.
        /// </exception>
        public static void SaveImages(Texture2D imageLeft, Texture2D imageRight,
                        Vector2[] coordsLeft, Vector2[] coordsRight, string datatime,
                        bool ispredicted)
        {

            int heightL = imageLeft.height;
            int heightR = imageRight.height;
            if (ispredicted)
            {
                coordsLeft = FlipCoordsToTopLeft(coordsLeft, heightL);
                coordsRight = FlipCoordsToTopLeft(coordsRight, heightR);
                Writer.FlipImage(ref imageLeft);
                Writer.FlipImage(ref imageRight);
            }

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
            Texture2D right = Writer.PrintTextureCircles(pRight, imageRight);

            byte[] bytesLeft = left.EncodeToPNG();
            byte[] bytesRight = right.EncodeToPNG();

            string predSuffix = ispredicted ? "_predicted" : "";

            string pathLeft = Application.dataPath + $"/../Output/Images/markers_left_{datatime}{predSuffix}.png";
            string pathRight = Application.dataPath + $"/../Output/Images/markers_right_{datatime}{predSuffix}.png";

            File.WriteAllBytes(pathLeft, bytesLeft);
            File.WriteAllBytes(pathRight, bytesRight);

            Debug.Log($"Saved images: {pathLeft}  |  {pathRight}");
        }

        /// <summary>
        /// Converts a set of 2D coordinates from a bottom-left origin
        /// coordinate system to a top-left origin system by vertically
        /// flipping the Y values relative to the given height.
        /// </summary>
        private static Vector2[] FlipCoordsToTopLeft(Vector2[] coords, int height)
        {
            Vector2[] flipped = new Vector2[coords.Length];
            for (int i = 0; i < coords.Length; i++)
                flipped[i] = new Vector2(coords[i].x, height - coords[i].y);
            return flipped;
        }



        /// <summary>
        /// Converts a 4×4 floating-point array into a <see cref="Matrix4x4"/>.
        /// </summary>
        /// <remarks>
        /// The input array is copied element-wise into a new <see cref="Matrix4x4"/> instance
        /// using row–column indexing. The method assumes the array represents a valid
        /// 4×4 homogeneous transformation matrix and performs no bounds or validity checks.
        /// </remarks>
        /// <param name="a">
        /// Two-dimensional float array of size [4,4] containing matrix values in row-major order.
        /// </param>
        /// <returns>
        /// A <see cref="Matrix4x4"/> populated with the values from the input array.
        /// </returns>
        private static Matrix4x4 ArrayToMatrix(float[,] a)
        {
            Matrix4x4 m = new Matrix4x4();
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    m[r, c] = a[r, c];
            return m;
        }


        /// <summary>
        /// Executes a full stereo calibration between the Varjo coordinate system and the Unity world coordinate system.
        /// </summary>
        /// <remarks>
        /// This method performs an end-to-end calibration pipeline 
        /// The resulting transformation matrices can optionally be persisted to disk
        /// for later reuse. On failure, the function returns <c>false</c> and outputs
        /// identity matrices.
        /// </remarks>
        /// <param name="api">
        /// Active <see cref="VarjoApiManager"/> instance used to acquire camera images
        /// and update intrinsic/extrinsic calibration parameters.
        /// </param>
        /// <param name="calibration">
        /// Calibration engine responsible for estimating the Varjo → Unity transformation.
        /// </param>
        /// <param name="debugMode">
        /// Enables verbose logging and diagnostic output during the optimization process.
        /// </param>
        /// <param name="markerOffset">
        /// Initial offset applied to marker positions, used as a starting point
        /// for the optimization.
        /// </param>
        /// <param name="filePath">
        /// Full file path where the resulting transformation matrices should be written
        /// if <paramref name="written2File"/> is set to <c>true</c>.
        /// </param>
        /// <param name="written2File">
        /// If <c>true</c>, the computed transformation matrices are saved to disk.
        /// </param>
        /// <param name="fromVarjoToUnity">
        /// OUTPUT — 4×4 homogeneous transformation matrix mapping points from the Varjo
        /// coordinate system into the Unity world coordinate system.
        /// </param>
        /// <param name="fromUnityToVarjo">
        /// OUTPUT — 4×4 homogeneous transformation matrix mapping points from the Unity
        /// world coordinate system back into the Varjo coordinate system.
        /// </param>
        /// <returns>
        /// <c>true</c> if calibration completes successfully and valid transformations
        /// are computed; <c>false</c> if marker detection, 3D marker retrieval,
        /// optimization, or file output fails.
        /// </returns>
        public static bool RunCalibration(
                        VarjoApiManager api, VarjoToUnityCalibration calibration, bool debugMode,
                        Vector3 markerOffset, string filePath, bool written2File,
                        out Matrix4x4 fromVarjoToUnity, out Matrix4x4 fromUnityToVarjo)
        {

            fromVarjoToUnity = Matrix4x4.identity;
            fromUnityToVarjo = Matrix4x4.identity;


            // Update camera intrinsics/extrinsics
            api.UpdateXtrinsics();


            // 2. Detect 2D markers on left/right camera image
            bool resultLeft = DetectMarkers2D(api.leftTexture, out Vector2[] coordsLeft, out long[] idsLeft);
            bool resultRight = DetectMarkers2D(api.rightTexture, out Vector2[] coordsRight, out long[] idsRight);

            if (!resultLeft || !resultRight)
            {
                Debug.LogWarning("CalibrationModule: Marker detection failed.");
                return false;
            }


            //  Get Varjo 3D markers
            List<Marker> varjoMarkers3D = GetVarjoMarkers3D();
            if (varjoMarkers3D.Count == 0)
            {
                Debug.LogWarning("CalibrationModule: No Varjo 3D markers detected.");
                return false;
            }


            //  Optimization-based calibration using Dradient Descent
            calibration.debugMode = debugMode;
            calibration.Calibrate(ref api.leftTexture, ref api.rightTexture, markerOffset, varjoMarkers3D, true, true, coordsLeft, coordsRight, idsLeft, idsRight, true);


            // get output matrices

            fromVarjoToUnity = calibration.GetTransformation();
            fromUnityToVarjo = fromVarjoToUnity.inverse;


            //Optional: write to file
            if (written2File)
            {
                try
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(filePath));
                    SaveTransformations(filePath, fromVarjoToUnity, fromUnityToVarjo);
                    Debug.Log($"CalibrationModule: Saved calibration to file:\n{filePath}");
                }
                catch (Exception ex)
                {
                    Debug.LogError($"CalibrationModule: Error writing to file:\n{ex}");
                    return false;
                }
            }


            return true;
        }



        /// <summary>
        /// Visualizes a 3D quadrilateral as a square-like wireframe object in Unity space.
        /// </summary>
        /// <remarks>
        /// This function converts a quad defined in the Varjo coordinate system into Unity
        /// world coordinates using the provided transformation matrix 
        /// The visualization is intended for debugging and validation of 3D corner
        /// reconstruction and Varjo-to-Unity calibration accuracy. The resulting GameObject
        /// acts as a parent container for all generated primitives.
        /// </remarks>
        /// <param name="quad">
        /// Input quad containing four 3D corner points defined in the Varjo coordinate system.
        /// The corner order is assumed to be consistent (e.g., Top-Left → Top-Right →
        /// Bottom-Right → Bottom-Left).
        /// </param>
        /// <param name="fromVarjoToUnity">
        /// 4×4 homogeneous transformation matrix mapping points from the Varjo coordinate
        /// system into the Unity world coordinate system.
        /// </param>
        /// <param name="cameraOffset">
        /// Additional positional offset applied after transformation, typically used to
        /// align the visualization with the Unity camera or scene origin.
        /// </param>
        /// <param name="color">
        /// Color applied to both corner spheres and connecting edges.
        /// </param>
        /// <param name="pointSize">
        /// Diameter (in Unity world units) of the rendered corner spheres.
        /// </param>
        /// <returns>
        /// A parent <see cref="GameObject"/> containing all generated corner and edge objects.
        /// </returns>
        public static GameObject DrawQuadAsSquare(
                 Corner3DQuad quad,
                 Matrix4x4 fromVarjoToUnity,
                 Vector3 cameraOffset,
                 Color color,
                 float pointSize = 0.02f,
                 Transform root = null)
        {
            GameObject parent = new GameObject($"Quad_{quad.Id}");

            if (root != null)
                parent.transform.SetParent(root, worldPositionStays: true);

            // Transform corners
            Vector3[] unityCorners = new Vector3[4];
            for (int i = 0; i < 4; i++)
            {
                unityCorners[i] = fromVarjoToUnity.MultiplyPoint3x4(quad.Points3D[i]) + cameraOffset;
            }

            for (int i = 0; i < 4; i++)
            {
                GameObject corner = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                corner.name = $"Corner_{i}";
                corner.transform.SetParent(parent.transform);
                corner.transform.position = unityCorners[i];
                corner.transform.localScale = Vector3.one * pointSize;

                var renderer = corner.GetComponent<Renderer>();
                renderer.material.color = color;
            }

            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                CreateLine(parent.transform, unityCorners[i], unityCorners[j], color);
            }

            return parent;
        }


        /// <summary>
        /// Creates a debug visualization of a 3D quad by instantiating
        /// small sphere primitives at each of its corner positions.
        ///
        /// The quad’s corner coordinates are transformed from Varjo
        /// space into Unity world space using the supplied matrix and
        /// camera offset.
        /// </summary>
        public static GameObject DrawQuad(
                 Corner3DQuad quad,
                 Matrix4x4 fromVarjoToUnity,
                 Vector3 cameraOffset,
                 Color color,
                 float pointSize = 0.02f,
                 Transform root = null)
        {
            GameObject parent = new GameObject($"Quad_{quad.Id}");

            if (root != null)
                parent.transform.SetParent(root, worldPositionStays: true);

            // Transform corners
            Vector3[] unityCorners = new Vector3[4];
            for (int i = 0; i < 4; i++)
            {
                unityCorners[i] = fromVarjoToUnity.MultiplyPoint3x4(quad.Points3D[i]) + cameraOffset;
            }

            for (int i = 0; i < 4; i++)
            {
                GameObject corner = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                corner.name = $"Corner_{i}";
                corner.transform.SetParent(parent.transform);
                corner.transform.position = unityCorners[i];
                corner.transform.localScale = Vector3.one * pointSize;

                var renderer = corner.GetComponent<Renderer>();
                renderer.material.color = color;
            }

            return parent;
        }

        /// <summary>
        /// Creates and renders a colored line segment between two points in world space.
        /// </summary>
        /// <remarks>
        /// This helper method instantiates a Unity <see cref="LineRenderer"/> to visualize
        /// an edge between two 3D positions. The line is attached to the specified parent
        /// transform and rendered using Unity’s default line material. The line width and
        /// color are fixed and intended for lightweight debug visualization.
        /// </remarks>
        /// <param name="parent">
        /// Parent <see cref="Transform"/> under which the line object is created.
        /// </param>
        /// <param name="start">
        /// World-space start position of the line segment.
        /// </param>
        /// <param name="end">
        /// World-space end position of the line segment.
        /// </param>
        /// <param name="color">
        /// Color applied uniformly to the line.
        /// </param>
        private static void CreateLine(Transform parent, Vector3 start, Vector3 end, Color color)
        {
            GameObject lineObj = new GameObject("Edge");
            lineObj.transform.SetParent(parent);

            var lr = lineObj.AddComponent<LineRenderer>();
            lr.positionCount = 2;
            lr.SetPosition(0, start);
            lr.SetPosition(1, end);
            lr.widthMultiplier = 0.005f;
            lr.material = new Material(Shader.Find("Sprites/Default")); // use default unity material
            lr.startColor = color;
            lr.endColor = color;
            lr.useWorldSpace = true;
        }


        /// <summary>
        /// Finds an existing root GameObject used to contain drawn quads,
        /// or creates a new one if none exists.
        ///
        /// This is typically used as a common parent for debug-drawn
        /// quad visualizations in the scene hierarchy.
        /// </summary>
        public static Transform GetOrCreateQuadRoot(string rootName = "QuadRoot")
        {
            var existing = GameObject.Find(rootName);
            if (existing != null) return existing.transform;

            var go = new GameObject(rootName);
            return go.transform;
        }

        /// <summary>
        /// Attempts to extract a numeric quad identifier from a GameObject
        /// name following the convention <c>"Quad_{id}"</c>.
        /// </summary>
        /// <param name="quadName">
        /// Name of the GameObject to parse.
        /// </param>
        /// <param name="id">
        /// When this method returns, contains the parsed identifier
        /// if parsing succeeded; otherwise <c>0</c>.
        /// </param>
        /// <returns>
        /// <c>true</c> if the name matched the expected format and the
        /// identifier could be parsed; otherwise <c>false</c>.
        /// </returns>
        /// <remarks>
        /// This helper is primarily used as a fallback when a
        /// <see cref="QuadTooltipId"/> component is not present.
        /// </remarks>
        private static bool TryParseQuadId(string quadName, out long id)
        {
            id = 0;
            const string prefix = "Quad_";
            if (string.IsNullOrEmpty(quadName) || !quadName.StartsWith(prefix)) return false;
            return long.TryParse(quadName.Substring(prefix.Length), out id);
        }

        /// <summary>
        /// Destroys all quad GameObjects that are children of the
        /// specified quad root transform.
        /// </summary>
        /// <param name="quadRoot">
        /// Root transform containing quad visualization objects.
        /// </param>
        /// <remarks>
        /// Only child objects are destroyed; the root itself is
        /// preserved.
        ///
        /// Safe to call with a null root.
        /// </remarks>
        public static void ClearDrawnQuads(Transform quadRoot)
        {
            if (quadRoot == null) return;

            for (int i = quadRoot.childCount - 1; i >= 0; i--)
            {
                UnityEngine.Object.Destroy(quadRoot.GetChild(i).gameObject);
            }

        }

        /// <summary>
        /// Removes all drawn quad visualizations under the specified root
        /// transform and optionally clears any associated tooltips using
        /// the provided <see cref="TooltipManager"/>.
        ///
        /// Tooltip identifiers are resolved via an attached
        /// <see cref="QuadTooltipId"/> component when present, or by
        /// parsing the GameObject name using the <c>"Quad_{id}"</c>
        /// convention as a fallback.
        /// </summary>
        /// <param name="quadRoot">
        /// Root transform containing quad visualization objects.
        /// </param>
        /// <param name="tooltipManager">
        /// Optional TooltipManager used to remove tooltips associated
        /// with each quad before destruction.
        /// </param>
        /// <remarks>
        /// Destroying the quad GameObjects does not automatically clean
        /// up tooltips unless a manager is provided.
        ///
        /// The root itself is preserved.
        /// </remarks>
        public static void ClearDrawnQuadswithLabel(Transform quadRoot, TooltipManager tooltipManager = null)
        {
            if (quadRoot == null) return;

            for (int i = quadRoot.childCount - 1; i >= 0; i--)
            {
                Transform child = quadRoot.GetChild(i);

                // Remove tooltip first (if we have a manager and an id)
                if (tooltipManager != null)
                {
                    var idTag = child.GetComponent<QuadTooltipId>();
                    if (idTag != null)
                    {
                        tooltipManager.RemoveTooltip(idTag.id);
                    }
                    else
                    {
                        // Fallback: parse id from "Quad_{id}" name if you didn't add QuadTooltipId
                        if (TryParseQuadId(child.name, out long parsedId))
                            tooltipManager.RemoveTooltip(parsedId);
                    }
                }

                UnityEngine.Object.Destroy(child.gameObject);
            }
        }
        /// <summary>
        /// Draws a 3D quad visualization consisting of corner spheres and
        /// edge lines, and optionally attaches a tooltip label near one
        /// corner using a <see cref="TooltipManager"/>.
        ///
        /// Corner coordinates are transformed from Varjo space into Unity
        /// world space before visualization.
        /// </summary>
        /// <param name="quad">
        /// Source quad containing four 3D corner points and an identifier.
        /// </param>
        /// <param name="fromVarjoToUnity">
        /// Transformation matrix converting Varjo tracking coordinates
        /// into Unity world space.
        /// </param>
        /// <param name="cameraOffset">
        /// World-space offset applied after transforming coordinates.
        /// </param>
        /// <param name="color">
        /// Color applied to the corner spheres, edges, and tooltip text.
        /// </param>
        /// <param name="tooltipManager">
        /// TooltipManager used to create and manage the quad label.
        /// If null, no tooltip is created.
        /// </param>
        /// <param name="labelText">
        /// Text displayed in the tooltip label.
        /// If null or empty, a default label containing the quad ID
        /// is used.
        /// </param>
        /// <param name="pointSize">
        /// Diameter of each corner sphere in world units.
        /// Defaults to <c>0.02</c>.
        /// </param>
        /// <param name="labelOffsetMeters">
        /// Distance above the reference corner used to place the
        /// tooltip anchor, measured in meters.
        /// </param>
        /// <param name="root">
        /// Optional parent transform for the created quad GameObject.
        /// </param>
        /// <param name="offsetUsesCameraUp">
        /// If true, the tooltip offset direction is computed using
        /// the XR camera’s up vector rather than world-up.
        /// </param>
        /// <param name="tooltipId">
        /// Optional explicit tooltip identifier.
        /// If not supplied, the quad’s ID is used.
        /// </param>
        /// <returns>
        /// The newly created parent <see cref="GameObject"/> containing
        /// all visualization elements for the quad.
        /// </returns>
        public static GameObject DrawQuadAsSquarewithLabel(
                                                    Corner3DQuad quad,
                                                    Matrix4x4 fromVarjoToUnity,
                                                    Vector3 cameraOffset,
                                                    Color color,
                                                    TooltipManager tooltipManager,
                                                    string labelText,
                                                    float pointSize = 0.02f,
                                                    float labelOffsetMeters = 0.03f,
                                                    Transform root = null,
                                                    bool offsetUsesCameraUp = false,
                                                    long? tooltipId = null)
        {
            GameObject parent = new GameObject($"Quad_{quad.Id}");

            if (root != null)
                parent.transform.SetParent(root, worldPositionStays: true);

            // Transform corners
            Vector3[] unityCorners = new Vector3[4];
            for (int i = 0; i < 4; i++)
                unityCorners[i] = fromVarjoToUnity.MultiplyPoint3x4(quad.Points3D[i]) + cameraOffset;

            Transform corner0Transform = null;

            // Corners
            for (int i = 0; i < 4; i++)
            {
                GameObject corner = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                corner.name = $"Corner_{i}";
                corner.transform.SetParent(parent.transform);
                corner.transform.position = unityCorners[i];
                corner.transform.localScale = Vector3.one * pointSize;

                var renderer = corner.GetComponent<Renderer>();
                renderer.material.color = color;

                if (i == 0) corner0Transform = corner.transform;
            }

            // Edges
            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                CreateLine(parent.transform, unityCorners[i], unityCorners[j], color);
            }

            // Tooltip above Corner_0 (using TooltipManager)
            if (tooltipManager != null && corner0Transform != null)
            {
                var xrCamera = Camera.main;

                // Create an anchor transform above Corner_0 and attach tooltip to it
                var anchor = new GameObject("Corner0_TooltipAnchor");
                anchor.transform.SetParent(corner0Transform, worldPositionStays: false);

                // Offset direction (world-up or camera-up)
                Vector3 up = offsetUsesCameraUp && xrCamera != null ? xrCamera.transform.up : Vector3.up;
                // Since anchor is parented to corner0Transform, store offset in local space.
                // Convert "up" into corner0 local space:
                Vector3 localUp = corner0Transform.InverseTransformDirection(up.normalized);
                anchor.transform.localPosition = localUp * labelOffsetMeters;

                long id = tooltipId ?? quad.Id;
                var tag = parent.AddComponent<QuadTooltipId>();
                tag.id = id;
                // ShowTooltip creates/attaches; ApplyStyle clears text, so update text immediately after.
                tooltipManager.ShowTooltip(id, anchor.transform);
                tooltipManager.UpdateTooltip(
                    id,
                    newText: string.IsNullOrEmpty(labelText) ? $"id: {quad.Id}" : labelText,
                    fontColor: color,
                    bgColor: new Color(0, 0, 0, 0.35f)
                );

                // Optional: fit background once
                var tip = tooltipManager.GetTooltip(id);
                if (tip != null)
                    tooltipManager.FitBackgroundToText(tip);


            }

            return parent;
        }


        /// <summary>
        /// Starts the streaming sender and detection receiver servers using
        /// explicitly provided dependencies.
        /// </summary>
        /// <param name="sender">
        /// Sender responsible for transmitting the stereo stream.
        /// </param>
        /// <param name="receiver">
        /// Receiver responsible for receiving detection packets.
        /// </param>
        /// <remarks>
        /// This variant avoids scene searches and is recommended for SDK usage
        /// and production projects where dependencies should be explicit.
        /// </remarks>
        public static void StartServers(Sender sender, DetectionReceiver receiver)
        {
            Sender resolvedSender = sender != null ? sender : UnityEngine.Object.FindObjectOfType<Sender>();

            DetectionReceiver resolvedReceiver = receiver != null ? receiver : UnityEngine.Object.FindObjectOfType<DetectionReceiver>();

            resolvedSender?.StartStream();
            resolvedReceiver?.StartReceiver();
        }


        /// <summary>
        /// Converts a detection packet into a list of left-eye and right-eye
        /// 2D corner sets scaled to the target image resolution.
        ///
        /// Detection coordinates are assumed to be expressed in a fixed
        /// detector resolution (e.g., 224×224) and are rescaled to the
        /// supplied image width and height.
        /// </summary>
        /// <param name="pkt">
        /// Detection packet containing per-item bounding box or corner
        /// coordinates for left and right views.
        /// </param>
        /// <param name="imgH">
        /// Height of the target image in pixels.
        /// </param>
        /// <param name="imgW">
        /// Width of the target image in pixels.
        /// </param>
        /// <returns>
        /// A list of tuples where each entry contains:
        /// • <c>id</c> – the class or object identifier  
        /// • <c>left</c> – left-eye 2D corner information  
        /// • <c>right</c> – right-eye 2D corner information
        /// </returns>
        /// <remarks>
        /// The detector resolution is currently assumed to be 224×224.
        /// If this changes, the scaling logic should be updated accordingly.
        ///
        /// Corner ordering is:
        /// Top-Left, Top-Right, Bottom-Right, Bottom-Left.
        ///
        /// If the packet or its item list is null, an empty list is returned.
        ///
        /// This method performs allocation and is intended for conversion
        /// or debugging pipelines rather than tight per-frame hot paths.
        /// </remarks>
        public static List<(int id, Corner2DInfo left, Corner2DInfo right)> ToCorner2DList(DetectionReceiver.PairPacket pkt, int imgH, int imgW, int pkt_img_w=640,int pkt_img_h=640)
        {
            int n = (pkt?.items != null) ? pkt.items.Length : 0;
            var result = new List<(int, Corner2DInfo, Corner2DInfo)>(n);
            
            var scaleX = (float)imgW / pkt_img_w;
            var scaleY = (float)imgH / pkt_img_h;

            for (int i = 0; i < n; i++)
            {
                var it = pkt.items[i];

                Vector2[] leftPts = new Vector2[]
                {
                new Vector2(it.lx1*scaleX, it.ly1*scaleY), // TL
                new Vector2(it.lx2*scaleX, it.ly1*scaleY), // TR
                new Vector2(it.lx2*scaleX, it.ly2*scaleY), // BR
                new Vector2(it.lx1*scaleX, it.ly2*scaleY), // BL
                };

                Vector2[] rightPts = new Vector2[]
                {
                new Vector2(it.rx1*scaleX, it.ry1*scaleY), // TL
                new Vector2(it.rx2*scaleX, it.ry1*scaleY), // TR
                new Vector2(it.rx2*scaleX, it.ry2*scaleY), // BR
                new Vector2(it.rx1*scaleX, it.ry2*scaleY), // BL
                };

                result.Add((
                    it.clsid,
                    new Corner2DInfo(it.clsid, leftPts),
                    new Corner2DInfo(it.clsid, rightPts)
                ));
            }

            return result;
        }

        /// <summary>
        /// Logs a formatted list of bounding boxes to the Unity console
        /// using <see cref="Debug.Log(object)"/>.
        /// </summary>
        public static void PrintBBoxList(string label, List<BBox> list)
        {
            Debug.Log($"--- {label}  ({list.Count} bbox) ---");

            for (int i = 0; i < list.Count; i++)
            {
                var box = list[i];
                var arr = box.Box;
                Debug.Log(
                    $"[{i}] ID={box.Id}  " +
                    $"x={arr[0]:F1}, y={arr[1]:F1}, " +
                    $"w={arr[2]:F1}, h={arr[3]:F1}"
                );
            }
        }
        /// <summary>
        /// Logs a formatted list of 2D corner sets to the Unity console
        /// using <see cref="Debug.Log(object)"/>.
        /// </summary>
        public static void PrintCorner2DList(string label, List<Corner2DInfo> list)
        {
            Debug.Log($"--- {label} ({list.Count}) ---");

            for (int i = 0; i < list.Count; i++)
            {
                var c = list[i];
                Debug.Log(
                    $"[{i}] ID={c.Id}  " +
                    $"P0={c.Points[0]}, " +
                    $"P1={c.Points[1]}, " +
                    $"P2={c.Points[2]}, " +
                    $"P3={c.Points[3]}"
                );
            }
        }
        /// <summary>
        /// Logs detailed information about a single <see cref="Corner3DQuad"/>
        /// to the Unity console, including 3D corner points and per-eye 2D
        /// projections.
        /// </summary>
        private static void PrintQuad(Corner3DQuad quad, int index)
        {
            Debug.Log(
                $"Quad {index}  (ID={quad.Id})\n" +
                $" 3D:\n" +
                $"   P0 = {quad.Points3D[0]}\n" +
                $"   P1 = {quad.Points3D[1]}\n" +
                $"   P2 = {quad.Points3D[2]}\n" +
                $"   P3 = {quad.Points3D[3]}\n" +
                $" Left2D:\n" +
                $"   P0 = {quad.Left2D[0]}\n" +
                $"   P1 = {quad.Left2D[1]}\n" +
                $"   P2 = {quad.Left2D[2]}\n" +
                $"   P3 = {quad.Left2D[3]}\n" +
                $" Right2D:\n" +
                $"   P0 = {quad.Right2D[0]}\n" +
                $"   P1 = {quad.Right2D[1]}\n" +
                $"   P2 = {quad.Right2D[2]}\n" +
                $"   P3 = {quad.Right2D[3]}"
            );
        }
        /// <summary>
        /// Logs a summary and detailed listing of all quads in the provided array
        /// to the Unity console.
        /// </summary>
        public static void PrintQuads(Corner3DQuad[] quads)
        {
            Debug.Log($"--- Found {quads.Length} 3D object ---");
            for (int i = 0; i < quads.Length; i++)
                PrintQuad(quads[i], i);
        }



        /// <summary>
        /// Logs a formatted listing of paired left-eye and right-eye
        /// 2D corner detections to the Unity console.
        ///
        /// Each entry prints the pair index, object ID, and the four
        /// ordered corners for both views.
        /// </summary>
        public static void PrintCornerPairs(
            List<(int id, Corner2DInfo left, Corner2DInfo right)> pairs)
        {
            if (pairs == null || pairs.Count == 0)
            {
                Debug.Log("Corner pairs: <empty>");
                return;
            }

            for (int i = 0; i < pairs.Count; i++)
            {
                (int id, Corner2DInfo left, Corner2DInfo right) p = pairs[i];

                Debug.Log(
                    $"Pair {i} | Id={p.id}\n" +
                    $"  Left : {FormatCorner2DInfo(p.left)}\n" +
                    $"  Right: {FormatCorner2DInfo(p.right)}"
                );
            }
        }


        /// <summary>
        /// Formats a <see cref="Corner2DInfo"/> instance into a readable
        /// string containing its four corner coordinates.
        ///
        /// Used internally by debug logging utilities.
        /// </summary>
        /// <param name="info">
        /// Corner data structure to format.
        /// </param>
        /// <returns>
        /// A human-readable string describing the four corners, or
        /// <c>"&lt;invalid&gt;"</c> if the data is missing or malformed.
        /// </returns>
        private static string FormatCorner2DInfo(Corner2DInfo info)
        {
            if (info.Points == null || info.Points.Length != 4)
                return "<invalid>";

            return
                $"TL={FormatVec2(info.Points[0])}, " +
                $"TR={FormatVec2(info.Points[1])}, " +
                $"BR={FormatVec2(info.Points[2])}, " +
                $"BL={FormatVec2(info.Points[3])}";
        }
        /// <summary>
        /// Formats a <see cref="Vector2"/> value using one decimal place
        /// precision for logging and debugging output.
        /// </summary>
        private static string FormatVec2(Vector2 v)
        {
            return $"({v.x:F1}, {v.y:F1})";
        }

        /// <summary>
        /// Logs the contents of a stereo detection packet to the Unity
        /// console, including header information and per-item bounding
        /// box coordinates for both left and right views.
        /// </summary>
        /// <param name="pkt">
        /// Detection packet to inspect.
        /// </param>
        /// <remarks>
        /// Intended for debugging and diagnostics.
        ///
        /// The output includes:
        /// • Timestamp  
        /// • Item count  
        /// • Class IDs  
        /// • Left and right bounding box coordinates
        ///
        /// If the packet or its item list is null or empty, a short
        /// message is logged instead of item details.
        /// </remarks>
        public static void PrintPairPacket(DetectionReceiver.PairPacket pkt)
        {
            if (pkt == null)
            {
                Debug.Log("PairPacket: <null>");
                return;
            }

            Debug.Log(
                $"PairPacket\n" +
                $"  timestamp = {pkt.timestamp}\n" +
                $"  count     = {pkt.count}"
            );

            if (pkt.items == null || pkt.items.Length == 0)
            {
                Debug.Log("  items: <empty>");
                return;
            }

            for (int i = 0; i < pkt.items.Length; i++)
            {
                DetectionReceiver.PairItem it = pkt.items[i];

                Debug.Log(
                    $"  Item[{i}] clsid={it.clsid}\n" +
                    $"    Left : ({it.lx1:F1}, {it.ly1:F1}) → ({it.lx2:F1}, {it.ly2:F1})\n" +
                    $"    Right: ({it.rx1:F1}, {it.ry1:F1}) → ({it.rx2:F1}, {it.ry2:F1})"
                );
            }
        }


    }
    /// <summary>
    /// Store the tooltip id used for this quad so we can clean it up later.
    /// </summary>
    public sealed class QuadTooltipId : MonoBehaviour
    {
        public long id;
    }
}