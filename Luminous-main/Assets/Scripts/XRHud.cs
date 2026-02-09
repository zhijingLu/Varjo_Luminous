// CameraTopBarHUD.cs
//
// ▸ Import TextMesh Pro Essentials once (Window ▸ TextMeshPro ▸ Import…).
// ▸ Add an empty GameObject to your scene (e.g. “XR-UI”) and attach this script.
// ▸ Press Play – a white bar spans the top of the headset view.

using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class XRHud : MonoBehaviour
{
    [Header("Initial text")]
    [SerializeField] string initialText = "Hello XR!";

    [Header("Bar appearance")]
    [SerializeField] int    barHeightPx   = 80;          // fixed pixel height
    [SerializeField] Color  backgroundCol = Color.white; // white bar
    [SerializeField] Color  textCol       = Color.red; // black letters
    [SerializeField] int    fontSize      = 48;

    /* ————————— runtime refs ————————— */
    Camera           cam;
    TextMeshProUGUI  label;

    void Awake()
    {
        cam = Camera.main ?? FindObjectOfType<Camera>();
        if (!cam) { Debug.LogError("CameraTopBarHUD - no Camera found"); return; }

        BuildHUD();
    }

    /* ----------------- public API ------------------------------------- */
    public void SetText(string txt)
    {
        if (label) label.text = txt;
    }

    /* ----------------- internals -------------------------------------- */
    void BuildHUD()
    {
        /* 1 ║ create canvas parented to the camera */
        GameObject canvasGO = new GameObject("HUD_TopBar_Canvas");
        canvasGO.transform.SetParent(cam.transform, false);

        Canvas canvas   = canvasGO.AddComponent<Canvas>();
        canvas.renderMode  = RenderMode.ScreenSpaceCamera;
        canvas.worldCamera = cam;
        canvas.planeDistance = 0.8f;               // 50 cm in front of near-clip

        RectTransform canvasRT            = canvas.GetComponent<RectTransform>();
        canvasRT.sizeDelta = new Vector2(1000, 200); // any baseline size
        canvasRT.localScale = Vector3.one * 0.001f;

        // CanvasScaler scaler = canvasGO.AddComponent<CanvasScaler>();
        // scaler.uiScaleMode            = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        // scaler.referenceResolution    = new Vector2(1000, 200); // any baseline
        // scaler.screenMatchMode        = CanvasScaler.ScreenMatchMode.Expand;

        /* 2 ║ background panel that spans full width */
        GameObject panelGO = new GameObject("TopBar");
        panelGO.transform.SetParent(canvasGO.transform, false);
        Image bg = panelGO.AddComponent<Image>();
        bg.color = backgroundCol;

        RectTransform panelRT = bg.rectTransform;
        // panelRT.anchorMin = new Vector2(0, 1);   // top-left
        // panelRT.anchorMax = new Vector2(1, 1);   // top-right
        panelRT.pivot     = new Vector2(0.5f, 0.5f);
        panelRT.sizeDelta = new Vector2(0, barHeightPx);    // width 0 = stretch
        panelRT.anchoredPosition = Vector2.zero;

        /* 3 ║ text inside the bar */
        GameObject txtGO = new GameObject("Label");
        txtGO.transform.SetParent(panelGO.transform, false);
        label = txtGO.AddComponent<TextMeshProUGUI>();

        label.text      = initialText;
        label.fontSize  = fontSize;
        label.color     = textCol;
        label.alignment = TextAlignmentOptions.MidlineLeft;
        label.enableWordWrapping = false;

        RectTransform txtRT = label.rectTransform;
        txtRT.anchorMin = Vector2.zero;
        txtRT.anchorMax = Vector2.one;
        txtRT.offsetMin = new Vector2(20, 0);    // 20 px left padding
        txtRT.offsetMax = new Vector2(-20, 0);   // 20 px right padding
    }
}
