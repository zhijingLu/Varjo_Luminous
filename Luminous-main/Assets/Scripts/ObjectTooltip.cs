// ObjectTooltip.cs ─ a fully-configurable, camera-facing tooltip
using UnityEngine;
using TMPro;
using UnityEngine.UI;

[RequireComponent(typeof(RectTransform))]
public class ObjectTooltip : MonoBehaviour
{
    [Header("Prefab references")]
    public TextMeshProUGUI label;
    public Image background;
    public Image arrow;

    [Header("Follow")]
    public Vector3 worldOffset = Vector3.up * 0.05f;   // 5 cm above target

    Transform target;
    Camera cam;
    RectTransform rect;

    void Awake()
    {
        rect = GetComponent<RectTransform>();
        cam = Camera.main ?? FindObjectOfType<Camera>();
    }

    // ─────────────────────────────  Public API  ────────────────────────────
    public void AttachTo(Transform targetTransform, float extraHeight = 0.0f)
    {
        target = targetTransform;
        Renderer r = targetTransform.GetComponentInChildren<Renderer>();
        if (r) worldOffset = new Vector3(0, r.bounds.extents.y + 0.05f + extraHeight, 0);
    }

    public void SetText(string txt) => label.text = txt;
    public void SetSize(Vector2 px) => rect.sizeDelta = px;                // width, height in canvas pixels
    public void SetArrowSize(Vector2 px)
    {
        if (arrow) arrow.rectTransform.sizeDelta = px;
    }
    public void SetFont(float size, Color c) { label.fontSize = size; label.color = c; }
    public void SetBackground(Color c)
    {
        if (background) background.color = c;
    }


    /// <summary>
    /// Positions the tooltip arrow below the background panel with
    /// a configurable pixel gap.
    ///
    /// This method assumes both the arrow and background use
    /// <see cref="RectTransform"/> components and are part of the same
    /// tooltip canvas hierarchy.
    /// </summary>
    /// <param name="gapPx">
    /// Vertical spacing in pixels between the bottom edge of the
    /// background panel and the arrow.
    /// </param>
    /// <remarks>
    /// The arrow is anchored and pivoted to the center before positioning.
    /// If either the arrow or background reference is missing,
    /// the method exits without making changes.
    /// </remarks>
    public void PositionArrowBelowBackground(float gapPx)
    {
        if (!arrow || !background) return;

        RectTransform bg = background.rectTransform;
        RectTransform ar = arrow.rectTransform;

        // Make sure arrow is anchored to bottom-center (do this once in prefab if you can)
        ar.anchorMin = new Vector2(0.5f, 0.5f);
        ar.anchorMax = new Vector2(0.5f, 0.5f);
        ar.pivot = new Vector2(0.5f, 0.5f);

        float bgHalfH = bg.rect.height * 0.5f;
        float arrowHalf = ar.rect.height * 0.5f;

        // Put arrow below the background with some gap
        ar.anchoredPosition = new Vector2(0f, -(bgHalfH + gapPx + arrowHalf));
    }

    /// <summary>
    /// Sets whether the tooltip arrow is rendered above or below
    /// the background panel and flips its orientation accordingly.
    ///
    /// If <c>true</c>, place arrow below the background, the arrow points downward.
    /// If <c>false</c>, place arrow above the background, the arrow is rotated 180°
    /// to point upward.
    /// </summary>
    public void SetArrowPlacement(bool above)
    {
        if (!arrow || !background) return;

        RectTransform ar = arrow.rectTransform;
        RectTransform bg = background.rectTransform;

        float bgHalf = bg.rect.height * 0.5f;
        float arHalf = ar.rect.height * 0.5f;
        float gap = 6f;

        if (above)
        {
            // Arrow below background, pointing down
            ar.localRotation = Quaternion.identity;
            ar.anchoredPosition = new Vector2(0f, -(bgHalf + arHalf + gap));
        }
        else
        {
            // Arrow above background, flipped, pointing up
            ar.localRotation = Quaternion.Euler(0, 0, 180f);
            ar.anchoredPosition = new Vector2(0f, bgHalf + arHalf + gap);
        }
    }


    // LateUpdate is called after all Update functions have been called
    void LateUpdate()
    {
        if (!target) return;

        // Follow
        transform.position = target.position + worldOffset;

        // Billboard (keep upright)
        //Vector3 dir = cam.transform.position - transform.position;
        //dir.y = 0;
        //transform.rotation = Quaternion.LookRotation(-dir);
    }

}
