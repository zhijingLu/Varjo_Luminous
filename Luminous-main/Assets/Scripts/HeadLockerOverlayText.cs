using TMPro;
using UnityEngine;

/// <summary>
/// Head-locked overlay text that stays in front of the HMD camera.
/// Attach this to a World-Space Canvas (recommended) or any GameObject.
/// Put a TextMeshProUGUI (or any TMP_Text) under it.
/// </summary>
public class HeadLockerOverlayText : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Assign your XR HMD camera Transform (Main Camera / CenterEye). If left empty, it will try Camera.main.")]
    public Transform hmdCamera;

    [Tooltip("Assign a TMP text component (TextMeshProUGUI). If left empty, it will auto-find in children.")]
    public TMP_Text text;

    [Header("Placement")]
    [Tooltip("How far in front of the camera (meters).")]
    public float distance = 1.5f;

    [Tooltip("Local offset relative to the camera (meters).")]
    public Vector3 offset = new Vector3(0f, -0.1f, 0f);

    [Header("Startup")]
    public string initialText = "Hello AR Overlay!";
    public bool updateWithTime = false; // demo toggle

    void Awake()
    {
        // Try to find camera if not assigned
        if (!hmdCamera && Camera.main) hmdCamera = Camera.main.transform;

        // Try to find TMP text if not assigned
        if (!text) text = GetComponentInChildren<TMP_Text>(true);

        // Set initial text if possible
        if (text) text.text = initialText;
    }

    void LateUpdate()
    {
        if (!hmdCamera) return;

        // Position in front of the HMD with an optional offset
        transform.position =
            hmdCamera.position +
            hmdCamera.forward * distance +
            hmdCamera.TransformVector(offset);

        // Face the camera (so it's readable)
        transform.rotation = Quaternion.LookRotation(transform.position - hmdCamera.position);

        // Optional demo: update continuously
        if (updateWithTime && text)
            text.text = $"Time: {Time.time:F1}s";
    }

    /// <summary>
    /// Call this from anywhere to change the overlay message at runtime.
    /// </summary>
    public void SetText(string msg)
    {
        if (!text) text = GetComponentInChildren<TMP_Text>(true);
        if (text) text.text = msg;
    }
}
