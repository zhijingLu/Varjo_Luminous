using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEditor.XR.LegacyInputHelpers;
using Varjo.XR;
// Imports for our file
using DFKI_Utilities;
using static DFKI_Utilities.VarjoToUnityCalibration;
using static DFKI_Utilities.CalibrationHelper;
using static DFKI_Utilities.DataStructure;
public class calibration_example : MonoBehaviour
{


    [Header("Varjo / Calibration")]
    public VarjoApiManager varjoApiManager;
    public VarjoToUnityCalibration calibration;
    public Matrix4x4 fromVarjoToUnityTransformation = Matrix4x4.identity;
    public Matrix4x4 fromUnityToVarjoTransformation = Matrix4x4.identity;


    [Header("Tooltips / Visualization")]
    public TooltipManager tooltipManager;

    //Basic Variables
    private bool initialized = false;
    public bool debugMode = true;
    public GameObject cameraOffset;


    [Header("Networking / Detection")]
    public Sender sender; // reference to the stream.cs instance in the scene
    public DetectionReceiver detReceiver;  //get bbox from this receiver


    private Transform quadRoot; // parent object for drawn quads, so they can be cleared easily


    // Start is called before the first frame update
    IEnumerator Start()
    {
        Debug.Log("[Calibration] Start");
        varjoApiManager.SetUndistort(true);
        yield return WarmupFrames(60);
        //Setup calibration object
        calibration = new VarjoToUnityCalibration(ref varjoApiManager.cameraLeft, ref varjoApiManager.cameraRight, debugMode);
        fromVarjoToUnityTransformation = Matrix4x4.identity;
        fromUnityToVarjoTransformation = Matrix4x4.identity;
        Debug.Log("[Calibration] Warm-up complete. Starting calibration...");

        string filePath = Application.dataPath + "/../Output/Calibration.txt";
        bool ok = false;
        //calibration with multi markers
        while (!ok)
        {
            ok = RunCalibration(
                varjoApiManager,
                calibration,
                debugMode,
                new Vector3(0f, 0f, 0.3f),
                filePath,
                true,
                out fromVarjoToUnityTransformation,
                out fromUnityToVarjoTransformation);

            yield return null;   // let Unity breathe
        }
        initialized = true;
        Debug.Log("[Calibration] Calibration complete.");
        calibration.InitQuadSettings();

        // create parent object for drawn quads (to be able to clear them easily)
        quadRoot = GetOrCreateQuadRoot("QuadRoot");

        //Start sender/ receiver 
        StartServers(sender, detReceiver);

        if (tooltipManager == null) tooltipManager = FindObjectOfType<TooltipManager>();


    }


    // Update is called once per frame
    void Update()
    {
        if (detReceiver == null || sender == null || sender.frameBuffer == null) return;
        while (detReceiver.TryDequeue(out var pkt))
        {

            ProcessPairPacket(pkt);

        }

    }


    /// <summary>
    /// Processes a single stereo detection packet by locating the
    /// closest synchronized stereo frame, converting 2D detections
    /// into image-space corner pairs, triangulating 3D quads, and
    /// visualizing the results with optional tooltip labels.
    ///
    /// Previously drawn debug quads and labels are cleared before
    /// rendering the new results.
    /// </summary>
    /// <param name="pkt">
    /// Detection packet containing stereo bounding box or corner
    /// information along with a timestamp.
    /// </param>
    /// <remarks>
    /// The processing pipeline includes:
    /// • Clearing previously drawn quads and tooltips  
    /// • Timestamp synchronization with buffered stereo frames  
    /// • Conversion of detection coordinates into image space  
    /// • Gradient-descent–based 3D triangulation  
    /// • Visualization using quad primitives and tooltips
    ///
    /// If no matching stereo frame is found, the packet is skipped.
    ///
    /// This method is typically called from <see cref="Update"/> while
    /// draining the detection receiver queue.
    /// </remarks>
    private void ProcessPairPacket(DetectionReceiver.PairPacket pkt)
    {
        // Clear previous debug quads + labels
        ClearDrawnQuadswithLabel(quadRoot, tooltipManager);

        Debug.Log($"[Calibration] PairPacket ts={pkt.timestamp}, count={pkt.count}");

        long ts = pkt.timestamp <= (ulong)long.MaxValue ? (long)pkt.timestamp : long.MaxValue;

        // Find closest stereo frame
        var frame = sender.frameBuffer.FindClosest(ts);
        if (frame == null)
        {
            Debug.LogWarning($"[Calibration] No frame found for ts={ts}");
            return;
        }

        Matrix4x4 leftExt = frame.extrinsicsLeft;
        Matrix4x4 rightExt = frame.extrinsicsRight;

        // Convert packet -> corner pairs in image coordinates
        List<(int id, Corner2DInfo left, Corner2DInfo right)> pairs =
            ToCorner2DList(pkt, imgH: varjoApiManager.cameraLeft.width, imgW: varjoApiManager.cameraLeft.height, pkt_img_w: 640, pkt_img_h: 640);

        // Triangulate 3D quads
        bool ok3d = calibration.Compute3D_TriGD(
            pairs,
            leftExt,
            rightExt,
            Optimization.Method.GradientDescent_BLS_Fast,
            out Corner3DQuad[] quads,
            out float pixelError
        );

        if (!ok3d) return;

        //PrintQuads(quads);
        // Debug.Log($"[CalibrationExample] Pixel RMS Error = {pixelError}");

        // Draw quads + labels
        for (int i = 0; i < quads.Length; i++)
        {
            DrawQuadAsSquarewithLabel(
                quad: quads[i],
                fromVarjoToUnity: fromVarjoToUnityTransformation,
                cameraOffset: cameraOffset.transform.position,
                color: Color.red,
                tooltipManager: tooltipManager,
                labelText: $"ID{quads[i].Id}",
                pointSize: 0.01f,
                labelOffsetMeters: 0.05f,
                root: quadRoot,
                offsetUsesCameraUp: false,
                tooltipId: null
            );
        }
    }

}