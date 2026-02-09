using UnityEngine;
using UnityEngine.UI;
using System.Collections;


/// <summary>
/// Manages a segmented UI progress bar with animated step traversal
/// and optional tooltip hints displayed at each point.
///
/// The system supports:
/// • Dynamic point generation based on a sentence list
/// • Normalized progress control (0–1)
/// • Animated traversal between points
/// • XR-compatible tooltip display
/// • Arrow positioning and styling
///
/// Intended for instructional flows, tutorials, or step-based UI
/// guidance systems.
/// </summary>
public class ProgressBarManager : MonoBehaviour
{
   

    [Header("Config")]

    [Range(0f, 1f)] public float progress = 0f;
    private Image[] points;

    [Header("Point colors")]
    public Color reachedColor = Color.blue;
    public Color unreachedColor = new Color(1, 1, 1, 0.35f);

    [Header("Step animation")]
    public bool autoRunSteps = false;         // auto play from point 0 -> last
    public float stepDurationSeconds = 5f;  // time from one point to the next

    private bool runningStep = false;
    private float stepElapsed = 0f;
    private float stepDuration = 2f;
    private int _segFrom, _segTo;

    [Header("Prefab Setup")]
    public GameObject progressBarPrefab;          // drag your ProgressBar.prefab here
    public bool dontDestroyOnLoad = true;
    private string barRectPath = "Canvas/BG";    // change to match your prefab
    private string fillPath = "Canvas/BGFill";
    private string pointsRootPath = "Canvas/points";

    private GameObject progressBarInstance;
    public string progressBarRootName = "ProgressBar";

    [Header("Positioning")]
    private Canvas boundCanvas;
    public bool moveCanvas = true;
    public Vector3 canvasWorldPosition = new Vector3(0f, 0.5f, 5f);
    public Vector3 canvasWorldEuler = Vector3.zero;
    public Vector3 canvasWorldScale = new Vector3(0.01f, 0.01f, 0.01f);

    [Header("Refs")]
    private RectTransform barRect;      // BarBG RectTransform
    private Image fillImage;            // BarFill Image (Type=Filled)
    private RectTransform pointsRoot;   // Points container under Canvas
    public Image pointPrefab;          // Simple UI Image prefab for a point (optional)

    [Header("Refs")]
    public TooltipManager tooltipManager;
    public Camera xrCamera;

    public long markerId = 999;
    public bool showTooltipOnReach = true;
    [Header("Text Style")]
    public float fontSize = 80f;
    public Color textColor = Color.red;
    public Color bgColor = new Color(0, 0, 0, 0.35f);
    public Vector2 arrowSize = new Vector2(100f, 100f);

    [Header("Hint sentences")]
    public string[] sentences = new string[]
    {
        "step 1: Hello",
        "step 2",
        "step 3",
        "step 4",
        "step 5"
    };


    public enum TooltipPlacement { Center, Above, Below }
    [Header("Text(tooltip) Position")]

    public TooltipPlacement tooltipPlacement = TooltipPlacement.Above;
    public float tooltipOffsetY = 1.0f; // UI units (pixels) in Screen Space Canvas

    // Range-run state
    private int _rangeEnd;
    private float _rangeStepDuration;
    private bool _runRange;

    // Tooltip state
    private int _activeTooltipIndex = -1;


    //based on number of sentences
    private int pointCount => (sentences != null) ? sentences.Length : 0;



    void Awake()
    {
       
        EnsurePrefabInstance();
    }
    void Start()
    {
        Initialize();
    }

    void Update()
    {

        TickStep(Time.deltaTime);

        //// 2) Trigger PlayRange only when you want to start it 
        if (Input.GetKeyDown(KeyCode.Space))
        {
            PlayRange(0, pointCount - 1, stepDurationSeconds);
        }
    }
    // =========================
    // Public API (Methods)
    // =========================




    /// <summary>
    /// Replace the content array; point count becomes sentences.Length.
    /// Rebuilds points and refreshes visuals.
    /// </summary>
    public void SetSentences(string[] newSentences, bool rebuild = true)
    {
        sentences = newSentences ?? new string[0];
        if (rebuild) BuildPointsAndLayoutPoints();
        SetProgress(progress);
    }


    /// <summary>
    /// Stops any running animation. Does not change the current progress value.
    /// </summary>
    /// <param name="hideTooltip">
    /// If true, the active tooltip is hidden when stopping.
    /// </param>
    public void Pause(bool hideTooltip = false)
    {
        runningStep = false;
        _runRange = false;

        if (hideTooltip)
            HideActiveTooltip();
    }


    /// <summary>
    /// Animates the progress bar from a starting point index to an
    /// ending point index, advancing one segment at a time.
    ///
    /// Each segment represents the movement between two adjacent
    /// points and runs for the specified duration.
    /// </summary>
    /// <param name="fromIndex">
    /// Starting point index.
    /// </param>
    /// <param name="endIndex">
    /// Final target point index.
    /// </param>
    /// <param name="durationPerSegment">
    /// Duration in seconds to move between two points.
    /// </param>
    public void PlayRange(int fromIndex, int endIndex, float durationPerSegment)
    {
        EnsurePrefabInstance();
        // Clamp to valid point indices
        fromIndex = Mathf.Clamp(fromIndex, 0, pointCount - 1);
        endIndex = Mathf.Clamp(endIndex, 0, pointCount - 1);

        _rangeEnd = endIndex;
        _rangeStepDuration = durationPerSegment;
        _runRange = true;

        // If already at/after end, just snap tooltip to fromIndex and stop
        if (fromIndex >= endIndex)
        {
            _runRange = false;
            SwitchTooltipToPoint(fromIndex);
            return;
        }

        StartSegment(fromIndex, fromIndex + 1, _rangeStepDuration);
    }
    /// <summary>
    /// Sets the normalized progress value for the bar and updates
    /// the fill amount and reached/unreached point colors.
    ///
    /// The value is clamped to the range [0, 1].
    /// </summary>
    /// <param name="t01">
    /// Normalized progress value where 0 represents the start and
    /// 1 represents completion.
    /// </param>
    public void SetProgress(float t01)
    {
        EnsurePrefabInstance();
        t01 = Mathf.Clamp01(t01);
        progress = t01;

        if (fillImage) fillImage.fillAmount = t01;

        if (points == null || points.Length == 0) return;

        // 5 points => thresholds at 0, 0.25, 0.5, 0.75, 1.0
        for (int i = 0; i < points.Length; i++)
        {
            float threshold = (pointCount <= 1) ? 1f : (float)i / (pointCount - 1);
            bool reached = t01 >= threshold;
            points[i].color = reached ? reachedColor : unreachedColor;
        }
    }

    // =========================
    // Prefab Management
    // =========================


    /// <summary>
    /// Ensures that a ProgressBar GameObject instance exists and is ready for use.
    ///
    /// If an instance is already cached, the method exits early.
    /// Otherwise, it attempts to:
    /// - Find an existing ProgressBar GameObject in the scene by name.
    /// - Instantiate the configured prefab if none is found.
    /// - Optionally mark the instance as DontDestroyOnLoad.
    /// - Bind internal component references from the created or reused instance.
    ///
    /// Logs an error and aborts if no prefab is assigned and no existing object
    /// can be found.
    /// </summary>
    private void EnsurePrefabInstance()
    {
        if (progressBarInstance != null)
            return;

        // Reuse existing ProgressBar in scene if present
        var existing = GameObject.Find(progressBarRootName);
        if (existing != null)
        {
            progressBarInstance = existing;
        }
        else
        {
            if (progressBarPrefab == null)
            {
                Debug.LogError("[ProgressBarManager] progressBarPrefab not assigned and no existing ProgressBar found.");
                return;
            }

            progressBarInstance = Instantiate(progressBarPrefab);
            progressBarInstance.name = progressBarRootName;
        }

        if (dontDestroyOnLoad)
            DontDestroyOnLoad(progressBarInstance);

        BindRefsFromInstance();
    }

    /// <summary>
    /// Binds internal component references from the current ProgressBar instance.
    ///
    /// Searches the instantiated GameObject hierarchy for:
    /// - A Canvas component.
    /// - The bar RectTransform.
    /// - The fill Image.
    /// - The points root RectTransform.
    ///
    /// If configured, and if the Canvas is in World Space render mode,
    /// this method will reposition, rotate, and scale the Canvas according
    /// to the configured world-space values.
    ///
    /// Logs descriptive errors when required components or paths
    /// cannot be found.
    /// </summary>
    private void BindRefsFromInstance()
    {
        if (progressBarInstance == null) return;
        boundCanvas = progressBarInstance.GetComponentInChildren<Canvas>(true);
        if (boundCanvas == null)
        {
            Debug.LogError("[ProgressBarManager] No Canvas found in ProgressBar prefab.");
            return;
        }

        //in World Space (otherwise has no effect)
        if (moveCanvas && boundCanvas.renderMode == RenderMode.WorldSpace)
        {
            var t = boundCanvas.transform;
            t.position = canvasWorldPosition;
            t.eulerAngles = canvasWorldEuler;
            t.localScale = canvasWorldScale;
        }
       
        Transform barT = progressBarInstance.transform.Find(barRectPath);
        Transform fillT = progressBarInstance.transform.Find(fillPath);
        Transform pointsT = progressBarInstance.transform.Find(pointsRootPath);

        if (barT != null) barRect = barT.GetComponent<RectTransform>();
        if (fillT != null) fillImage = fillT.GetComponent<Image>();
        if (pointsT != null) pointsRoot = pointsT.GetComponent<RectTransform>();

        if (barRect == null) Debug.LogError($"[ProgressBarManager] barRect not found at '{barRectPath}'.");
        if (fillImage == null) Debug.LogError($"[ProgressBarManager] fillImage not found at '{fillPath}'. (Is it an Image? Type=Filled?)");
        if (pointsRoot == null) Debug.LogError($"[ProgressBarManager] pointsRoot not found at '{pointsRootPath}'.");
    }

    // =========================
    // Internals 
    // =========================


    /// <summary>
    /// Validates refs, sets camera, builds points, applies current progress visuals.
    /// Safe to call multiple times (idempotent).
    /// </summary>
    public void Initialize()
    {
        EnsurePrefabInstance();
        if (!xrCamera) xrCamera = Camera.main;

        if (showTooltipOnReach && !tooltipManager)
        {
            Debug.LogWarning("[ProgressBar] TooltipManager not assigned. Tooltips will be disabled.");
        }

        BuildPointsAndLayoutPoints();
        SetProgress(0);
        SwitchTooltipToPoint(0); //show tooltip for first point
    }


    /// <summary>
    /// Advances the currently active step animation by the
    /// given delta time.
    ///
    /// When the segment completes, optionally continues into
    /// the next segment if a range run is active.
    /// </summary>
    private void TickStep(float deltaTime)
    {
        if (!runningStep) return;

        int fromIndex = _segFrom;
        int toIndex = _segTo;

        float startT = PointThreshold(fromIndex);
        float endT = PointThreshold(toIndex);

        stepElapsed += deltaTime;
        float t = stepElapsed / stepDuration;
        if (t >= 1f) t = 1f;

        float p = Mathf.Lerp(startT, endT, t);
        SetProgress(p);

        if (t >= 1f)
        {
            runningStep = false;

            if (_runRange)
            {
                // reached requested end
                if (toIndex >= _rangeEnd)
                {
                    _runRange = false;

                    // Optional: show tooltip on final reached point
                    SwitchTooltipToPoint(toIndex);
                    return;
                }

                // continue: (toIndex) -> (toIndex + 1)
                StartSegment(toIndex, toIndex + 1, _rangeStepDuration);
            }
        }
    }

    /// <summary>
    /// Starts animating progress from one point index to another
    /// over a specified duration.
    ///
    /// Also switches the tooltip to the starting point.
    /// </summary>
    private void StartSegment(int fromIndex, int toIndex, float durationSeconds)
    {
        _segFrom = fromIndex;
        _segTo = toIndex;

        stepElapsed = 0f;
        stepDuration = durationSeconds;
        runningStep = true;

        SwitchTooltipToPoint(fromIndex);
    }


    /// <summary>
    /// Creates, destroys, and repositions progress point UI
    /// elements based on the current sentence count.
    ///
    /// Points are evenly distributed along the width of the
    /// progress bar.
    /// </summary>
    private void BuildPointsAndLayoutPoints()
    {
        EnsurePrefabInstance();
        // Clean up excess points
        for (int i = pointsRoot.childCount - 1; i >= pointCount; i--)
            Destroy(pointsRoot.GetChild(i).gameObject);

        if (pointsRoot == null || barRect == null) return;

        // 1) Build / cache points array
        int existing = pointsRoot.childCount;

        points = new Image[pointCount];

        // Use existing children up to pointCount
        int useCount = Mathf.Min(existing, pointCount);
        for (int i = 0; i < useCount; i++)
            points[i] = pointsRoot.GetChild(i).GetComponent<Image>();

        // Instantiate missing points if needed
        for (int i = useCount; i < pointCount; i++)
        {
            Image p = Instantiate(pointPrefab, pointsRoot);
            p.name = $"Point_{i + 1}";
            points[i] = p;
        }

        // 2) Layout points
        float width = barRect.rect.width;

        for (int i = 0; i < points.Length; i++)
        {
            float n = (pointCount <= 1) ? 0f : (float)i / (pointCount - 1);
            float x = Mathf.Lerp(-width * 0.5f, width * 0.5f, n);

            RectTransform rt = points[i].rectTransform;
            rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f);
            rt.pivot = new Vector2(0.5f, 0.5f);
            rt.anchoredPosition = new Vector2(x, 0f);
        }
    }


    /// <summary>
    /// Converts a point index into a normalized progress threshold
    /// in the range [0, 1].
    ///
    /// Used to determine when a progress value has reached a
    /// particular step.
    /// </summary>
    /// <param name="index">
    /// Index of the progress point.
    /// </param>
    /// <returns>
    /// Normalized threshold value.
    /// </returns>
    private float PointThreshold(int index)
    {
        if (pointCount <= 1) return 1f;
        index = Mathf.Clamp(index, 0, pointCount - 1);
        return (float)index / (pointCount - 1);
    }


    /// <summary>
    /// Displays and styles a tooltip associated with a specific
    /// progress point.
    ///
    /// Applies text content, sizing, arrow placement, and
    /// world-space offset according to configuration.
    /// </summary>
    /// <param name="index">
    /// Index of the point for which the tooltip should be shown.
    /// </param>
    private void ShowTooltipForPoint(int index)
    {
        if (!showTooltipOnReach || tooltipManager == null) return;
        if (points == null || index < 0 || index >= points.Length) return;

        long id = markerId + index;

        RectTransform anchor = points[index].rectTransform;

        tooltipManager.ShowTooltip(id, anchor);

        var text = sentences[index];
        ObjectTooltip tip = tooltipManager.GetTooltip(id);
        if (tip == null) return;

        tooltipManager.UpdateTooltip(
            id,
            newText: text,
            fontColor: textColor,
            bgColor: bgColor
        );

        // text styling
        tip.label.fontSize = fontSize;
        tip.label.color = textColor;

        // fit background to text
        tooltipManager.FitBackgroundToText(tip);

        float y =
         tooltipPlacement == TooltipPlacement.Above ? tooltipOffsetY :
         tooltipPlacement == TooltipPlacement.Below ? -tooltipOffsetY :
         0f;

        tip.SetArrowSize(arrowSize);
        bool above = tooltipPlacement == TooltipPlacement.Above;
        tip.SetArrowPlacement(above);

        tip.worldOffset = new Vector3(0f, y, 0f);
    }

    /// <summary>
    /// Hides the currently active tooltip (if any) and shows
    /// the tooltip associated with the specified point index.
    /// </summary>
    private void SwitchTooltipToPoint(int index)
    {
        if (!showTooltipOnReach || tooltipManager == null) return;
        if (points == null || index < 0 || index >= points.Length) return;

        if (_activeTooltipIndex == index) return;

        // Hide previous tooltip
        if (_activeTooltipIndex >= 0)
        {
            long prevId = markerId + _activeTooltipIndex;
            tooltipManager.RemoveTooltip(prevId);
        }

        _activeTooltipIndex = index;
        ShowTooltipForPoint(index);
    }
    /// <summary>
    /// Hides the currently active tooltip and clears the
    /// internal active-tooltip state.
    /// </summary>
    private void HideActiveTooltip()
    {
        if (tooltipManager == null) return;
        if (_activeTooltipIndex < 0) return;

        long prevId = markerId + _activeTooltipIndex;
        tooltipManager.RemoveTooltip(prevId);
        _activeTooltipIndex = -1;
    }
}
