using System;
using System.Collections.Generic;
using UnityEngine;
using Varjo.XR;


public class VarjoMarkerManager : MonoBehaviour
{
    /*────────────────────────────── Types ───────────────────────────────*/
    [Serializable]
    public struct TrackedObject
    {
        public long       id;               // marker ID from Varjo Marker Designer
        public GameObject gameObject;       // object to move/rotate
        public bool       dynamicTracking;  // true ⇒ enable prediction flag
    }

    /*──────────────────────────── Inspector ─────────────────────────────*/
    [Header("Marker tracking")]
    [Tooltip("Coordinate-space transform helper (Varjo → Unity)")]
    public VarjoToUnityTransformationManager varjoToUnityTransformationManager;

    [Tooltip("Objects driven by Varjo Markers (size MUST match marker IDs)")]
    public TrackedObject[] trackedObjects = Array.Empty<TrackedObject>();

    [Header("Tooltip system")]
    public TooltipManager tooltipManager;            // drag your TooltipSystem GO here

    /*──────────────────────────── Internals ─────────────────────────────*/
    private List<VarjoMarker> markers          = new();   // live markers this frame
    private List<long>        removedMarkerIds = new();   // ids lost this frame
    private readonly HashSet<long>     spawnedTooltips  = new();   // track which marker IDs already got a tooltip

    /*──────────────────────────── Unity hooks ───────────────────────────*/
    void OnEnable()  => VarjoMarkers.EnableVarjoMarkers(true);
    void OnDisable() => VarjoMarkers.EnableVarjoMarkers(false);

    void Update()
    {
        if (!VarjoMarkers.IsVarjoMarkersEnabled()) return;

        // Get up‑to‑date marker set
        VarjoMarkers.GetVarjoMarkers(out markers);

        //  Loop over all visible markers
        foreach (var marker in markers)
        {
            // Try find the matching tracked object
            for (int i = 0; i < trackedObjects.Length; i++)
            {
                if (trackedObjects[i].id != marker.id) continue;

                var obj = trackedObjects[i].gameObject;
                if (!obj) continue;

                // ── Pose update ─────────────────────────────────────────
                obj.SetActive(true);
                obj.transform.localPosition = marker.pose.position;
                obj.transform.localRotation = marker.pose.rotation;

                // if (marker.id == 205)                                     // cup (offset by half depth)
                // {
                //     var rend = obj.GetComponent<Renderer>();
                //     if (rend)
                //     {
                //         float halfDepth = rend.bounds.size.z * 0.5f;
                //         Vector3 localOffset = new(0, -halfDepth, 0);
                //         obj.transform.localPosition += marker.pose.rotation * localOffset;
                //     }
                // }

                // ── Prediction flag toggling ────────────────────────────
                bool wantsPrediction = trackedObjects[i].dynamicTracking;
                bool hasPrediction   = (marker.flags & VarjoMarkerFlags.DoPrediction) != 0;
                if (wantsPrediction && !hasPrediction)
                    VarjoMarkers.AddVarjoMarkerFlags(marker.id, VarjoMarkerFlags.DoPrediction);
                else if (!wantsPrediction && hasPrediction)
                    VarjoMarkers.RemoveVarjoMarkerFlags(marker.id, VarjoMarkerFlags.DoPrediction);

                // Tooltip (once, with default blank style)
                if (tooltipManager && !spawnedTooltips.Contains(marker.id))
                {
                    tooltipManager.ShowTooltip(marker.id, obj.transform);
                    spawnedTooltips.Add(marker.id);
                }
            }
        }

        // Handle removed markers (hide objects + tooltips)
        VarjoMarkers.GetRemovedVarjoMarkerIds(out removedMarkerIds);
        foreach (var id in removedMarkerIds)
        {
            for (int i = 0; i < trackedObjects.Length; i++)
            {
                if (trackedObjects[i].id != id) continue;
                if (trackedObjects[i].gameObject) trackedObjects[i].gameObject.SetActive(false);
            }
            if (tooltipManager) tooltipManager.HideTooltip(id);
            spawnedTooltips.Remove(id);
        }
    }
}
