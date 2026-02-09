using UnityEngine;

/// <summary>
/// Spawns an ObjectTooltip anchored to the HMD camera with a local (headset-relative) offset.
/// Uses the same tooltip component style as TooltipManager (ObjectTooltip.SetText, SetSize, etc.).
/// </summary>
public class HeadsetTooltip : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Assign your HMD/VR camera here (e.g., XR Rig Main Camera).")]
    public Camera hmdCamera;

    [Tooltip("Same prefab you use in TooltipManager: must contain an ObjectTooltip component.")]
    public GameObject tooltipPrefab;

    [Header("Default placement (relative to HMD)")]
    [Tooltip("Local offset from the HMD camera. +Z is forward, +Y up, +X right (in camera local space).")]
    public Vector3 localOffset = new Vector3(0f, -0.10f, 0.60f);

    [Header("Default text")]
    public string initialText = "Hello from HMD tooltip!";

    private ObjectTooltip _tip;
    private Transform _anchor;

    private void Awake()
    {
        if (!hmdCamera) hmdCamera = Camera.main;
        if (!hmdCamera)
        {
            Debug.LogError("[HeadsetTooltip] No HMD camera assigned and Camera.main not found.");
            enabled = false;
            return;
        }

        if (!tooltipPrefab)
        {
            Debug.LogError("[HeadsetTooltip] tooltipPrefab not assigned.");
            enabled = false;
            return;
        }

        // Create an anchor that is a child of the HMD camera so it stays headset-relative.
        var anchorGO = new GameObject("HMD_TooltipAnchor");
        _anchor = anchorGO.transform;
        _anchor.SetParent(hmdCamera.transform, worldPositionStays: false);
        _anchor.localPosition = localOffset;
        _anchor.localRotation = Quaternion.identity;
    }

    private void Start()
    {
        // Show on start (your requested behavior).
        Show(initialText, localOffset);
    }

    /// <summary>
    /// Show (or update) the tooltip text at a headset-relative offset.
    /// </summary>
    public void Show(string text, Vector3 offsetLocalToHmd)
    {
        if (!_anchor) return;

        _anchor.localPosition = offsetLocalToHmd;

        if (_tip == null)
        {
            var go = Instantiate(tooltipPrefab);
            _tip = go.GetComponent<ObjectTooltip>();
            if (_tip == null)
            {
                Debug.LogError("[HeadsetTooltip] tooltipPrefab has no ObjectTooltip component.");
                Destroy(go);
                return;
            }

            // Same pattern as TooltipManager: attach the tooltip to a target transform.
            _tip.AttachTo(_anchor);
        }

        // Same kind of calls you already use (SetText, SetSize, SetArrowSize, etc.).
        _tip.SetText(text);
        _tip.SetSize(new Vector2(200, 40));
        _tip.SetArrowSize(new Vector2(30, 30));
    }

    /// <summary>
    /// Convenience: update only the text (keeps current offset).
    /// </summary>
    public void SetText(string text)
    {
        if (_tip == null) return;
        _tip.SetText(text);
        _tip.SetSize(new Vector2(200, 40));
        _tip.SetArrowSize(new Vector2(30, 30));
    }

    /// <summary>
    /// Hide by clearing text and collapsing size (same approach as TooltipManager.HideTooltip).
    /// </summary>
    public void Hide()
    {
        if (_tip == null) return;
        _tip.SetText("");
        _tip.SetSize(Vector2.zero);
        _tip.SetArrowSize(Vector2.zero);
    }
}
