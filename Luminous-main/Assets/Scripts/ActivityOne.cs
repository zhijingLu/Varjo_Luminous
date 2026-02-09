/* 
* ActivityOne.cs
* -----------------
* A simple activity script for a coffee-making simulation in Unity.
* It guides the user through a series of steps to make coffee, including
* grabbing a cup, placing it under a coffee tray, selecting coffee from a menu,
* waiting for brewing, taking the cup out, grabbing a milk pack, and pouring milk into the cup.
*
* The script uses an interaction manager to handle user interactions and a tooltip manager
* to provide visual feedback on what the user should do next.
*
* The activity is structured as a finite state machine (FSM) with defined steps and transitions.
* It also includes text-to-speech announcements for each step.
*
* Usage:
* 1. Attach this script to a GameObject in your Unity scene.
* 2. Assign the InteractionManager and TooltipManager references in the inspector.
* 3. Set the marker IDs for the cup, milk pack, and coffee machine in the inspector.
* 4. Call the Begin() method to start the activity.
* 5. Follow the on-screen instructions to complete the coffee-making process.
* -------------------------------------
* Author: Muhammad Asad Ali | June 2025
*/

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class ActivityOne : MonoBehaviour
{
    // ---------- Inspector references -----------------------------------
    [Header("Runtime references")]   
    public InteractionManager interactionManager;        // object interaction manager
    public TooltipManager     tooltipManager;            // centralised tooltips

    [Header("Marker IDs (VarjoMarkers → Inspector)")]
    public long cupId           = 205;
    public long milkPackId      = 225;
    public long coffeeId        = 244;   // adjust to your machine marker ID
    // public long coffeeTrayId    = 200;   // adjust to your tray marker ID
    // public long coffeeMenuId    = 200;   // adjust to your menu marker ID

    [Header("Behaviour timing")]
    public float brewDelaySeconds = 10f;  // how long to wait for brewing
    public float finishDelaySeconds = 3f; // how long to wait before finishing
    public float stepDelaySeconds = 2f; // delay before next interaction

    // ---------- FSM definition -----------------------------------------
    private enum Step
    {
        GrabCup = 0,
        PlaceCupUnderTray,
        SelectCoffeeFromMenu,
        WaitForBrew,
        TakeCupOut,
        GrabMilkPack,
        PourMilkIntoCup,
        Finished
    }

    private Step _current = Step.GrabCup;
    private readonly Dictionary<Step, string> _stepSpeech = new()
    {
        { Step.GrabCup,              "Grab the cup." },
        { Step.PlaceCupUnderTray,    "Place the cup under the coffee tray." },
        { Step.SelectCoffeeFromMenu, "Select your coffee from the menu and press start." },
        { Step.WaitForBrew,          "Brewing… Please wait until it's done." },
        { Step.TakeCupOut,           "Take the cup out of the tray." },
        { Step.GrabMilkPack,         "Pick up the milk pack." },
        { Step.PourMilkIntoCup,      "Pour milk into the cup." },
        { Step.Finished,             "Great job - Coffee is ready!" }
    };

    // Maps each step to the *first* interaction that causes the transition.
    // Format: Tuple<objectA, objectB> where names are as found in the scene.
    private readonly Dictionary<Step, (string, string)> _stepTriggers = new();

    // Timer used during the "WaitForBrew" step.
    private Coroutine _brewTimerCo;
    private Coroutine _finishDelayCo; 
    private Coroutine _stepDelayCo;   // inter‑step delay

    // ------------------- Unity lifecycle --------------------------------
    void Awake()
    {
        // -------- Build the trigger dictionary -------------------------
        _stepTriggers[Step.GrabCup]              = ("hand", "cup");
        _stepTriggers[Step.PlaceCupUnderTray]    = ("cup", "new_coffee_tray");
        _stepTriggers[Step.SelectCoffeeFromMenu] = ("hand", "new_coffee_menu_bar");
        // WaitForBrew handled by timer
        _stepTriggers[Step.TakeCupOut]           = ("cup", "new_coffee_tray");     
        _stepTriggers[Step.GrabMilkPack]         = ("hand", "milkPack");
        _stepTriggers[Step.PourMilkIntoCup]      = ("milkPack", "cup");

        // Fallbacks if not set in inspector
        if (!interactionManager) interactionManager = FindObjectOfType<InteractionManager>();
        if (!tooltipManager)     tooltipManager     = FindObjectOfType<TooltipManager>();
    }

    public void Begin()
    {
        Debug.Log($"{"[ActivityOne.cs] Starting the activity…".Colorize(Color.green)}");
        if (!interactionManager)
        {
            Debug.LogError("[ActivityOne] InteractionManager reference missing - cannot start activity.");
            return;
        }

        TextToSpeechPlayer.speak("I will help you make a coffee. " +
                                 "Please follow the instructions.");

        // Hook to both start & end of interactions
        interactionManager.OnInteractionStart += HandleInteractionStart;
        interactionManager.OnInteractionEnd += HandleInteractionEnd;

        // Kick things off
        AnnounceStep();
        UpdateTooltips();
    }

    // ---------------- Event handlers -----------------------------------
    private void HandleInteractionStart(GameObject a, GameObject b)
    {
        // Ignore if activity already finished
        if (_current == Step.Finished || _current == Step.TakeCupOut) return;

        // Convert to canonical pair (order‑independent)
        string n1 = Canon(a.name); string n2 = Canon(b.name);

        if (FailSafe(n1, n2)) return;

        // Debug.Log($"[ActivityOne] Interaction detected: {n1} + {n2}");

        // Check if this interaction matches the *next* trigger we expect
        if (!_stepTriggers.TryGetValue(_current, out var trigger)) return;

        bool match = (Is(n1, trigger.Item1) && Is(n2, trigger.Item2)) || (Is(n1, trigger.Item2) && Is(n2, trigger.Item1));
        if (!match) return;

        if (_current == Step.SelectCoffeeFromMenu)
        {
            AdvanceTo(Step.WaitForBrew);
            _brewTimerCo = StartCoroutine(BrewTimer());
        }
        else
        {
            AdvanceTo(_current + 1);
        }
    }

    private void HandleInteractionEnd(GameObject a, GameObject b)
    {
        if (_current != Step.TakeCupOut) return; // only care during this step

        // Convert to canonical pair (order‑independent)
        string n1 = Canon(a.name); string n2 = Canon(b.name);

        // Check if this interaction matches the *next* trigger we expect
        if (!_stepTriggers.TryGetValue(_current, out var trigger)) return;

        bool match = (Is(n1, trigger.Item1) && Is(n2, trigger.Item2)) || (Is(n1, trigger.Item2) && Is(n2, trigger.Item1));
        if (!match) return;
        AdvanceTo(_current + 1);  // proceed to next step
        // Debug.Log($"[ActivityOne] Interaction ended: {n1} + {n2}");
    }

    // ---------------- FSM helpers --------------------------------------
    private void AdvanceTo(Step next)
    {
        /* Cancel any existing pending transition and schedule a new one */
        if (_stepDelayCo != null) StopCoroutine(_stepDelayCo);
        _stepDelayCo = StartCoroutine(AdvanceAfterDelay(next));
    }

    private IEnumerator AdvanceAfterDelay(Step next)
    {
        if (stepDelaySeconds > 0f){
            if (next == Step.WaitForBrew)
            {
                yield return new WaitForSeconds(stepDelaySeconds + 2);
            }
            else
            {
                yield return new WaitForSeconds(stepDelaySeconds);
            }
        }

        _current = next;
        _stepDelayCo = null; // mark transition complete

        if (_current == Step.Finished)
        {
            _finishDelayCo = StartCoroutine(FinishAfterDelay());
        }

        AnnounceStep();
        UpdateTooltips();
    }

    private bool FailSafe(string n1, string n2)
    {
        if (_current == Step.GrabCup && HandObj(n1, n2, "milkpack"))
        { TextToSpeechPlayer.speak("That's the milk pack. Please grab the cup.", 1f); return true; }

        if (_current == Step.GrabCup && HandObj(n1, n2, "soda_can"))
        { TextToSpeechPlayer.speak("That's a soda can. Please grab the cup.", 1f); return true; }

        if (_current == Step.WaitForBrew && HandObj(n1, n2, "cup"))
        { TextToSpeechPlayer.speak("Brewing is in progress. Please wait until it's done.", 1f); return true; }

        if (_current == Step.GrabMilkPack && HandObj(n1, n2, "cup"))
        { TextToSpeechPlayer.speak("Please pick up the milk pack now, not the cup.", 1f); return true; }

        if (_current == Step.GrabMilkPack && HandObj(n1, n2, "soda_can"))
        { TextToSpeechPlayer.speak("Place the soda can down and pick up the milk pack.", 1f); return true; }

        // if (_current == Step.PlaceCupUnderTray)
        // {
        //     // check if the cup is touching the tray but not under it
        //     if (ObjObj(n1, n2, "cup", "coffee") && !ObjObj(n1, n2, "cup", "coffee_tray"))
        //     {
        //         TextToSpeechPlayer.speak("Please place the cup under the coffee tray.", 1f);
        //         return true;
        //     }
        // }

        return false;
    }
    
    // check if both objects are same as specified
    private bool ObjObj(string n1, string n2, string obj1, string obj2)
        => (Is(n1,obj1) && Is(n2,obj2)) || (Is(n1,obj2) && Is(n2,obj1));

    // check if one of the objects is "hand" and the other is the specified object    
    private bool HandObj(string n1, string n2, string obj)
        => (Is(n1,"hand") && Is(n2,obj)) || (Is(n1,obj) && Is(n2,"hand"));


    private IEnumerator BrewTimer()
    {
        yield return new WaitForSeconds(brewDelaySeconds);
        AdvanceTo(Step.TakeCupOut);
    }

    private IEnumerator FinishAfterDelay()
    {
        yield return new WaitForSeconds(finishDelaySeconds);
        Cleanup();
    }

    public void Cleanup()
    {
        interactionManager.OnInteractionStart -= HandleInteractionStart;
        interactionManager.OnInteractionEnd -= HandleInteractionEnd;
        if (_brewTimerCo != null) StopCoroutine(_brewTimerCo);
        _current = Step.GrabCup;
        // Hide all tooltips first
        tooltipManager.HideTooltip(cupId);
        tooltipManager.HideTooltip(milkPackId);
        tooltipManager.HideTooltip(coffeeId);
    }

    private void AnnounceStep()
    {
        if (_stepSpeech.TryGetValue(_current, out var line)){
            Debug.Log($"[ActivityOne.cs] {$"Step.{_current}".Colorize(Color.green)}: {line}");
            TextToSpeechPlayer.speak(line, 0.5f);
        }
    }

    private void UpdateTooltips()
    {
        if (!tooltipManager) return;

        // Hide all tooltips first
        tooltipManager.HideTooltip(cupId);
        tooltipManager.HideTooltip(milkPackId);
        tooltipManager.HideTooltip(coffeeId);

        switch (_current)
        {
            case Step.GrabCup:
                tooltipManager.UpdateTooltip(cupId, "Grab me!");
                // Debug.Log($"[ActivityOne] Updating tooltip for cup {cupId}");
                break;
            case Step.PlaceCupUnderTray:
                tooltipManager.UpdateTooltip(cupId, "Put me under the coffee tray");
                tooltipManager.UpdateTooltip(coffeeId, "Put the cup in coffee tray");
                break;
            case Step.SelectCoffeeFromMenu:
                tooltipManager.UpdateTooltip(coffeeId, "Select the coffee from Menu");
                break;
            case Step.WaitForBrew:
                tooltipManager.UpdateTooltip(coffeeId, "Brewing… wait for it");
                // tooltipManager.UpdateTooltip(cupId, "Wait for brew");
                break;
            case Step.TakeCupOut:
                tooltipManager.UpdateTooltip(cupId, "Take me out");
                break;
            case Step.GrabMilkPack:
                tooltipManager.UpdateTooltip(milkPackId, "Pick me up");
                break;
            case Step.PourMilkIntoCup:
                tooltipManager.UpdateTooltip(milkPackId, "Pour into cup");
                tooltipManager.UpdateTooltip(cupId, "Pour milk into me");
                break;
            case Step.Finished:
                tooltipManager.UpdateTooltip(cupId, "Coffee is ready!");
                break;
        }
    }

    /* ───────── Utils ───────── */
    private static string Canon(string n)=>n.EndsWith("_tip",StringComparison.OrdinalIgnoreCase)?"hand":n.ToLower();
    private static bool Is(string a,string b)=>a.Equals(b,StringComparison.OrdinalIgnoreCase);
}