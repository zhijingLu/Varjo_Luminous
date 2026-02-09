using UnityEngine;
using UnityEngine.XR;


/// <summary>
/// Displays an example: a head-locked HUD tooltip in front of an XR camera and
/// cycles through sentences over time.
///
/// The tooltip is managed through a <see cref="TooltipManager"/> and
/// is attached to an invisible anchor transform that stays centered
/// in the user's view.
/// </summary>
public class Text_example : MonoBehaviour
{
    [Header("Refs")]
    public TooltipManager tooltipManager; //create and update tooltips
    public Camera xrCamera; //Defaults to Camera.main if not set.

    [Header("HUD placement")]
    public float distance = 1.5f;          // meters in front of camera
    public Vector3 localOffset = Vector3.zero; //Extra offset relative to the camera forward direction.

    [Header("HUD content")]
    private long markerId = 999; //Unique ID for the tooltip instance.

    [Header("Style")]
    public float fontSize = 80f;
    public Color textColor = Color.red;
    public Color bgColor = new Color(0, 0, 0, 0.35f);


    public float secondsPerSentence = 2f; //Time each sentence stays on screen.
    [Header("sentences")]
    public string[] sentences = new string[]
    {
        "step 1",
        "step 2",
        "step 3",
        "step 4",
    };

    public bool loop = true; //  If true, the system loops back to the first sentence
    private int currentIndex = 0; //Current index into the sentences array.
    private float timer = 0f;
    private Transform anchor;  // Anchor point in front of camera
    private bool shown = false;



    void Awake()
    {

        if (!tooltipManager)
        {
            Debug.LogError("[HUD] TooltipManager not assigned.");
            enabled = false;
            return;
        }

        if (!xrCamera)
            xrCamera = Camera.main;

        if (!xrCamera)
        {
            Debug.LogError("[HUD] XR Camera not found.");
            enabled = false;
            return;
        }

        // Create anchor directly in front of the camera
        makeAnchor(distance, localOffset);
    }


    void Update()
    {
        //Debug.Log("LateUpdate called");
        if (!anchor || !tooltipManager) return;

        // Keep the anchor centered and in front
        anchor.localPosition = Vector3.forward * distance + localOffset;
        anchor.localRotation = Quaternion.identity;


        // Show once
        if (!shown)
        {
            tooltipManager.ShowTooltip(markerId, anchor);
            shown = true;
        }
        // Get current text based on time
        string currentText = showSentenceByTime(ref timer, secondsPerSentence, ref currentIndex, sentences, loop);

        // Update content
        tooltipManager.UpdateTooltip(
            markerId,
            newText: currentText,
            fontColor: textColor,
            bgColor: bgColor
        );

        // Adjust these text pro font size and color
        ObjectTooltip tip = tooltipManager.GetTooltip(markerId);
        tip.label.fontSize = fontSize;
        tip.label.color = textColor;
        tooltipManager.FitBackgroundToText(tip);

    }


    /// <summary>
    /// Creates an anchor GameObject parented to the XR camera
    /// which serves as the attachment point for the HUD.
    /// </summary>
    private void makeAnchor(float distance, Vector3 localOffset)
    {
        anchor = new GameObject("HUD_Center_Anchor").transform;
        anchor.SetParent(xrCamera.transform, false);

        anchor.localPosition = Vector3.forward * distance + localOffset;
        anchor.localRotation = Quaternion.identity;
        anchor.localScale = Vector3.one;
    }

    /// <summary>
    /// Advances through an array of sentences based on elapsed time.
    /// </summary>
    /// <returns>
    /// The currently active sentence string.
    /// </returns>
    private string showSentenceByTime(ref float timer, float secondsPerSentence, ref int currentIndex, string[] sentences, bool loop)
    {
        if (sentences == null || sentences.Length == 0)
            return string.Empty;
        if (secondsPerSentence <= 0f)
            return sentences[Mathf.Clamp(currentIndex, 0, sentences.Length - 1)];
        timer += Time.deltaTime;

        while (timer >= secondsPerSentence)
        {
            timer -= secondsPerSentence;

            if (currentIndex < sentences.Length - 1)
                currentIndex++;
            else if (loop)
                currentIndex = 0;
            else
                break; // stay on last sentence
        }

        currentIndex = Mathf.Clamp(currentIndex, 0, sentences.Length - 1);
        return sentences[currentIndex];
    }

    void OnDisable()
    {
        if (tooltipManager == null) return;
        var tip = tooltipManager.GetTooltip(markerId);
        if (tip == null) return; // already destroyed/removed

        tooltipManager.HideTooltip(markerId);
    }


}
