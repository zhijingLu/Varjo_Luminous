/*
 * ActivityTwo.cs
 * ------------------
 * A guided XR activity that walks the user through grabbing a soda from a fridge.
 *
 * Objects involved (names must match the GameObjects / markers in the scene):
 *   ▸ Hand          – implicit, provided by InteractionManager (fingertip "*_tip")
 *   ▸ soda_can      – the drink can sitting inside the fridge
 *   ▸ fridge_door   – the outer door panel (marker always visible ⇢ fridge found)
 *   ▸ fridge_body   – interior body (marker only visible WHEN the door is open)
 *
 * Flow (FSM):
 *   1.  FindFridge      – wait until the fridge_door marker appears.
 *   2.  OpenDoor        – wait until the fridge_body marker appears (door open).
 *   3.  FindCola        – wait until the soda_can marker appears in view.
 *   4.  GrabCola        – user grabs the can and removes it from the fridge (end‑interaction).
 *   5.  CloseDoor       – wait until fridge_body marker disappears (door shut).
 *   6.  Finished        – celebrate and clean‑up.
 *
 * The script relies on:
 *   ▸ InteractionManager – emits trigger enter/exit events between objects.
 *   ▸ TooltipManager     – shows floating labels on tracked objects.
 *   ▸ TextToSpeechPlayer – optional spoken guidance.
 *
 * Author : Muhammad Asad Ali               | June 2025
 * --------------------------------------------------------------------------*/

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActivityTwo : MonoBehaviour
{
    /* ───────────────────────── Inspector refs ───────────────────────── */
    [Header("Runtime references")]
    public InteractionManager interactionManager;   // collision events
    public TooltipManager     tooltipManager;       // floating hints

    [Header("Tracked objects (leave empty ⇢ auto-find by name)")]
    public GameObject fridgeDoor;   // "fridge_door"
    public GameObject fridgeBody;   // "fridge_body"
    public GameObject sodaCan;      // "soda_can"

    [Header("Marker IDs  (optional - for tooltips)")]
    public long fridgeDoorId = 300; 
    public long fridgeBodyId = 243;
    public long sodaCanId    = 242;

    [Header("Behaviour timing")]
    public float finishDelaySeconds = 3f; // how long to wait before finishing
    public float stepDelaySeconds = 2f; // delay before next interaction

    private bool _isRunning = false; // activity state

    /* ──────────────────────────── FSM ──────────────────────────────── */
    private enum Step
    {
        FindFridge = 0,
        OpenDoor,
        FindSoda,
        // GrabSoda,
        TakeSodaOut,
        CloseDoor,
        Finished
    }

    private Step _current = Step.FindFridge;

    private readonly Dictionary<Step,string> _speech = new()
    {
        { Step.FindFridge,      "Find the fridge." },
        { Step.OpenDoor,        "Now open the fridge door." },
        { Step.FindSoda,        "Find the cola can inside." },
        { Step.TakeSodaOut,     "Grab the cola and pull it out." },
        { Step.CloseDoor,       "Close the fridge door." },
        { Step.Finished,        "Awesome, Enjoy your drink!" }
    };

    /* Trigger‑pairs that advance the FSM via InteractionManager events. */
    private readonly Dictionary<Step,(string,string)> _startTriggers = new();
    private readonly Dictionary<Step,(string,string)> _endTriggers   = new();

    /* Coroutine handles */
    private Coroutine _stepDelayCo;   // inter‑step delay
    private Coroutine _finishDelayCo;    // final clean‑up delay
    
    /* ────────────────────────── Unity hooks ────────────────────────── */
    void Awake()
    {
        // Auto‑find references when not assigned.
        fridgeDoor ??= GameObject.Find("fridge_door");
        fridgeBody ??= GameObject.Find("fridge_body");
        sodaCan    ??= GameObject.Find("soda_can");

        interactionManager ??= FindObjectOfType<InteractionManager>();
        tooltipManager     ??= FindObjectOfType<TooltipManager>();

        /* Build trigger dictionaries */
        // _startTriggers[Step.GrabSoda]    = ("hand", "soda_can");
        _endTriggers  [Step.TakeSodaOut] = ("soda_can", "fridge_body");
    }

    public void Begin()
    {
        Debug.Log($"{"[ActivityTwo.cs] Starting the activity…".Colorize(Color.green)}");
        if (!interactionManager)
        {
            Debug.LogError("[ActivityTwo.cs] InteractionManager missing - abort.");
            return;
        }

        TextToSpeechPlayer.speak("I will help you take a drink from fridge. " +
                                 "Please follow the instructions.");

        interactionManager.OnInteractionStart += HandleInteractionStart;
        interactionManager.OnInteractionEnd   += HandleInteractionEnd;

        _isRunning = true;
        AnnounceStep();
        UpdateTooltips();
    }

    /* ────────────────────────── Update loop ────────────────────────── */
    void Update()
    {
        if (!_isRunning) return; // only run if activity is active
        /* Ignore input while we are in a delayed transition */
        if (_stepDelayCo != null) return;

        Debug.Log($"[ActivityTwo.cs] {$"Step.{_current}".Colorize(Color.green)}");

        switch (_current)
        {
            case Step.FindFridge:
                if (IsVisible(fridgeDoor)){
                    AdvanceTo(Step.OpenDoor);
                }
                break;
            case Step.OpenDoor:
                if (IsVisible(fridgeBody)){
                    AdvanceTo(Step.FindSoda);
                }
                break;
            case Step.FindSoda:
                if (IsVisible(sodaCan)){
                    AdvanceTo(Step.TakeSodaOut);
                }
                break;
            case Step.CloseDoor:
                // Door considered closed once the fridge interior (body) disappears
                if (!IsVisible(fridgeBody)){
                    AdvanceTo(Step.Finished);
                }
                break;
        }

        UpdateTooltips();
    }

    /* ───────────────────── Interaction callbacks ───────────────────── */
    private void HandleInteractionStart(GameObject a, GameObject b)
    {
        // if (_current != Step.GrabSoda) return;          // we only care in this state
        // if (!Matches(a,b,_startTriggers[Step.GrabSoda])) return;

        // AdvanceTo(Step.TakeSodaOut);
    }

    private void HandleInteractionEnd(GameObject a, GameObject b)
    {
        if (_current != Step.TakeSodaOut) return;

        // Convert to canonical pair (order‑independent)
        string n1 = Canon(a.name); string n2 = Canon(b.name);
        if (HandObj(n1, n2, "milkpack"))
        { 
            TextToSpeechPlayer.speak("That's a milkpack. Please grab the soda can.", 1f); 
            return; 
        }

        if (!Matches(a, b, _endTriggers[Step.TakeSodaOut])) return;

        AdvanceTo(Step.CloseDoor);
    }

    /* ──────────────────────── Helpers ──────────────────────────────── */
    private static string Canon(string n)=>n.EndsWith("_tip",StringComparison.OrdinalIgnoreCase)?"hand":n.ToLower();

    private static bool Is(string a,string b)=>a.Equals(b,StringComparison.OrdinalIgnoreCase);

    private static bool IsVisible(GameObject go) => go && go.activeInHierarchy;

    // check if one of the objects is "hand" and the other is the specified object    
    private bool HandObj(string n1, string n2, string obj)
        => (Is(n1,"hand") && Is(n2,obj)) || (Is(n1,obj) && Is(n2,"hand"));

    private static bool Matches(GameObject a, GameObject b, (string,string) pair)
    {
        /* Map fingertip proxies to generic "hand" so that
           any finger tip collider counts as the hand.        */
        string nameA = a.name.EndsWith("_tip", StringComparison.OrdinalIgnoreCase) ? "hand" : a.name;
        string nameB = b.name.EndsWith("_tip", StringComparison.OrdinalIgnoreCase) ? "hand" : b.name;

        return (EqualsIgnoreCase(nameA, pair.Item1) && EqualsIgnoreCase(nameB, pair.Item2)) ||
               (EqualsIgnoreCase(nameA, pair.Item2) && EqualsIgnoreCase(nameB, pair.Item1));
    }

    private static bool EqualsIgnoreCase(string a, string b) =>
        string.Equals(a, b, StringComparison.OrdinalIgnoreCase);

    /* ─────────────────────── FSM helpers ─────────────────────────── */
    private void AdvanceTo(Step next)
    {
        /* Cancel any existing pending transition and schedule a new one */
        if (_stepDelayCo != null) StopCoroutine(_stepDelayCo);
        _stepDelayCo = StartCoroutine(AdvanceAfterDelay(next));
    }

    private IEnumerator AdvanceAfterDelay(Step next)
    {
        if (stepDelaySeconds > 0f)
            yield return new WaitForSeconds(stepDelaySeconds);

        _current = next;
        _stepDelayCo = null; // mark transition complete

        if (_current == Step.Finished)
        {
            if (_finishDelayCo != null) StopCoroutine(_finishDelayCo);
            _finishDelayCo = StartCoroutine(FinishAfterDelay());
        }

        AnnounceStep();
        UpdateTooltips();
    }


    private IEnumerator FinishAfterDelay()
    {
        yield return new WaitForSeconds(finishDelaySeconds);
        Cleanup();
    }

    public void Cleanup()
    {
        interactionManager.OnInteractionStart -= HandleInteractionStart;
        interactionManager.OnInteractionEnd   -= HandleInteractionEnd;  
        _isRunning = false;
        _current = Step.FindFridge; // reset to initial state
        // Reset tooltips
        tooltipManager.HideTooltip(fridgeDoorId);
        tooltipManager.HideTooltip(fridgeBodyId);
        tooltipManager.HideTooltip(sodaCanId);

    }

    /* ───────────────────────── UI feedback ─────────────────────────── */
    private void AnnounceStep()
    {
        if (_speech.TryGetValue(_current, out var line))
        {
            Debug.Log($"[ActivityTwo.cs] {$"Step.{_current}".Colorize(Color.green)}: {line}");
            TextToSpeechPlayer.speak(line, 0.5f);
        }
    }

    private void UpdateTooltips()
    {
        if (!tooltipManager) return;
        if (!_isRunning) return; // only update tooltips if activity is active

        // Hide everything by default; only enable what we need.
        tooltipManager.HideTooltip(fridgeDoorId);
        tooltipManager.HideTooltip(fridgeBodyId);
        tooltipManager.HideTooltip(sodaCanId);

        switch (_current)
        {
            case Step.FindFridge:
                tooltipManager.UpdateTooltip(fridgeDoorId, "Fridge here!");
                break;
            case Step.OpenDoor:
                tooltipManager.UpdateTooltip(fridgeDoorId, "Open me");
                break;
            // case Step.FindSoda:
            //     tooltipManager.UpdateTooltip(sodaCanId, "Cola inside");
            //     break;
            case Step.TakeSodaOut:
                tooltipManager.UpdateTooltip(sodaCanId, "Grab me");
                // tooltipManager.HideTooltip(fridgeDoorId);
                break;
            case Step.CloseDoor:
                tooltipManager.UpdateTooltip(fridgeBodyId, "Close the door");
                break;
            case Step.Finished:
                tooltipManager.UpdateTooltip(sodaCanId, "Enjoy the drink!");
                // tooltipManager.HideTooltip(fridgeBodyId);
                break;
        }
    }
}
