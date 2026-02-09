/*
* TooltipManager.cs
* -----------------
* A simple tooltip system for Unity, allowing you to show and update tooltips
* attached to game objects in the scene. Tooltips can be styled with custom text,
* size, font size, font color, and background color.
*
* Usage:
* 1. Assign a tooltip prefab in the inspector that contains an ObjectTooltip component.
* 2. Call ShowTooltip with a marker ID and target transform to display a tooltip.
* 3. Use UpdateTooltip to change the text or style of an existing tooltip.
* 4. Call HideTooltip to clear the tooltip for a specific marker ID.
*
* Note: The tooltip prefab should have an ObjectTooltip component that handles the
* display logic. The tooltip will automatically attach to the target transform.
* -------------------------------------
* Author: Muhammad Asad Ali | June 2025
*/

using System;
using System.Collections.Generic;
using UnityEngine;

public class TooltipManager : MonoBehaviour
{
    // ───────────────────────── Types ─────────────────────────
    [Serializable]
    public struct ObjectTooltipStyle
    {
        public string text;
        public Vector2 sizePx;
        public float fontSize;
        public Color fontColor;
        public Color bgColor;
    }

    // ────────────────────── Default preset ───────────────────
    public static readonly ObjectTooltipStyle DEFAULT_STYLE = new ObjectTooltipStyle
    {
        text = "",
        sizePx = new Vector2(200, 40),
        fontSize = 14,
        fontColor = Color.black,
        bgColor = new Color(1f, 1f, 1f, 0.3f)
    };
    public class TooltipFitSettings
    {
        public Vector2 padding = new Vector2(30f, 18f);
        public Vector2 minBgSize = new Vector2(120f, 60f);
        public Vector2 maxBgSize = new Vector2(2000f, 800f);
        public bool centerLabel = true;
    }
    // ───────────────────── Inspector refs ─────────────────────
    [Header("Tooltip prefab")]
    public GameObject tooltipPrefab;

    // ───────────────────── Internals ──────────────────────────
    private readonly Dictionary<long, ObjectTooltip> _active = new();


    // ───────────────────── Public API ─────────────────────────
    public void ShowTooltip(long markerId, Transform target) => ShowTooltip(markerId, target, DEFAULT_STYLE);
    /// <summary>
    /// Creates (if necessary) and displays a tooltip attached to the
    /// specified target transform, then applies the provided visual style.
    ///
    /// If a tooltip with the same marker ID already exists, the existing
    /// instance is reused and only its style is updated.
    /// </summary>
    /// <param name="markerId">
    /// Unique identifier used to track and retrieve this tooltip instance.
    /// Reusing an ID will update the existing tooltip instead of creating
    /// a new one.
    /// </param>
    /// <param name="target">
    /// The transform the tooltip should follow in world space, such as
    /// a camera anchor or scene object.
    /// </param>
    /// <param name="style">
    /// Styling data describing the tooltip's appearance, including
    /// colors, font size, background properties, and arrow settings.
    /// </param>
    public void ShowTooltip(long markerId, Transform target, ObjectTooltipStyle style)
    {
        if (!_active.TryGetValue(markerId, out var tip))
        {
            if (!tooltipPrefab)
            {
                Debug.LogError("[TooltipManager] tooltipPrefab not assigned!");
                return;
            }
            GameObject go = Instantiate(tooltipPrefab);
            tip = go.GetComponent<ObjectTooltip>();
            tip.AttachTo(target);
            _active[markerId] = tip;
        }

        ApplyStyle(markerId, tip, style);
    }

    public void UpdateTooltip(long markerId, string newText = null, Color? fontColor = null, Color? bgColor = null)
    {
        if (!_active.TryGetValue(markerId, out var tip)) return;
        if (fontColor.HasValue) tip.SetFont(tip.label.fontSize, fontColor.Value);
        if (bgColor.HasValue) tip.SetBackground(bgColor.Value);
        if (newText != null)
        {
            tip.SetText(newText);
            tip.SetSize(new Vector2(200, 40));
            // set size of arrow image
            tip.SetArrowSize(new Vector2(30, 30));
        }
        // Debug.Log($"[TooltipManager] Updated tooltip for marker {markerId}: {newText}, {newText != null}");
    }

    public void HideTooltip(long markerId)
    {
        if (_active.TryGetValue(markerId, out var tip))
        {
            tip.SetText("");  // clear text if null
            tip.SetSize(new Vector2(0, 0));  // reset size if no text
            tip.SetArrowSize(new Vector2(0, 0));
        }
    }

    // ───────────────────── Helpers ────────────────────────────
    private static void ApplyStyle(long markerId, ObjectTooltip tip, ObjectTooltipStyle s)
    {
        // tip.SetText($"{markerId}");
        // tip.SetSize(s.sizePx);
        tip.SetFont(s.fontSize, s.fontColor);
        tip.SetBackground(s.bgColor);
        tip.SetSize(new Vector2(0, 0));  // reset size first
        tip.SetArrowSize(new Vector2(0, 0)); // reset arrow size first
        tip.SetText("");
    }

    /// <summary>
    /// Retrieves an active tooltip instance by its unique identifier.
    /// </summary>
    /// <param name="id">
    /// The marker ID associated with the tooltip when it was created.
    /// </param>
    /// <returns>
    /// The <see cref="ObjectTooltip"/> instance if found; otherwise
    /// <c>null</c> when no active tooltip with the given ID exists.
    /// </returns>
    public ObjectTooltip GetTooltip(long id)
    {
        return _active.TryGetValue(id, out var tip) ? tip : null;
    }



    /// <summary>
    /// Resizes the tooltip background panel to fit the current text
    /// content using layout constraints defined in the provided
    /// <see cref="TooltipFitSettings"/>.
    /// </summary>
    /// <returns>
    /// The final background size in pixels after padding and clamping
    /// have been applied.
    /// </returns>
    public Vector2 FitBackgroundToText(ObjectTooltip tip, TooltipFitSettings settings = null)
    {
        settings ??= new TooltipFitSettings(); // use default settings if none provided
        if (tip == null || tip.label == null || tip.background == null || settings == null)
            return Vector2.zero;

        tip.label.ForceMeshUpdate();

        Vector2 pref = new Vector2(tip.label.preferredWidth, tip.label.preferredHeight);

        Vector2 target = pref + settings.padding;
        target.x = Mathf.Clamp(target.x, settings.minBgSize.x, settings.maxBgSize.x);
        target.y = Mathf.Clamp(target.y, settings.minBgSize.y, settings.maxBgSize.y);

        RectTransform bgRT = tip.background.rectTransform;
        bgRT.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, target.x);
        bgRT.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, target.y);

        if (settings.centerLabel)
            tip.label.rectTransform.anchoredPosition = Vector2.zero;
        tip.PositionArrowBelowBackground(0f);
        return target;
    }
    /// <summary>
    /// Destroys and removes a tooltip associated with the given marker ID.
    /// </summary>
    /// <param name="markerId">
    /// The unique identifier of the tooltip to remove.
    /// </param>
    public void RemoveTooltip(long markerId)
    {
        if (_active.TryGetValue(markerId, out var tip))
        {
            if (tip != null) Destroy(tip.gameObject);
            _active.Remove(markerId);
        }
    }
    /// <summary>
    /// Removes and destroys all currently active tooltips managed
    /// by this TooltipManager instance.
    /// </summary>
    public void ClearAllTooltips()
    {
        foreach (var kv in _active)
        {
            if (kv.Value != null) Destroy(kv.Value.gameObject);
        }
        _active.Clear();
    }
}
