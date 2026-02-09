/*
 *  InteractionManager
 *  ---------------
 *  • Adds trigger-colliders to each "tracked" scene object (FBX, props, etc.).
 *  • Creates a small proxy sphere for every fingertip from the Ultraleap frame.
 *  • Uses trigger‐events to detect:
 *        - object ↔ object   overlap
 *        - fingertip ↔ object overlap
 *  • Logs when an interaction starts, ends, and its duration.
 *  • Each Update also prints an AABB-based overlap-percentage (optional).
 *  • Visual yellow spheres show each fingertip while the hand is visible (optional).
 * -------------------------------------
 * Author: Muhammad Asad Ali | June 2025
 */

using System;
using System.Collections.Generic;
using UnityEngine;
using Leap;
using Leap.Unity;


public class InteractionManager : MonoBehaviour
{
    public event Action<GameObject, GameObject> OnInteractionStart;
    public event Action<GameObject, GameObject> OnInteractionEnd;

    [Header("Debug visuals")]
    [Tooltip("When false, all interactable mesh renderers are hidden but colliders remain active.")]
    public bool show3DObjects = false;
    [Tooltip("When true, fingertip proxy spheres are visible.")]
    public bool showFingertips = false;
    [Tooltip("When true, generated interaction zones are visible (cyan).")]
    public bool showZones = false;  // change only works before running the application

    [Tooltip("All FBX / tracked objects you care about")]
    public GameObject[] trackedObjects;

    [Header("Relative interaction zones (local to a reference transform)")]
    public RelativeZone[] relativeZones;

    [Serializable]
    public class RelativeZone
    {
        public string     name        = "Zone";          // e.g. "MenuBar"
        public Transform  reference;                     // root of the coffee machine
        public Vector3    localCenter = Vector3.zero;    // metres (x‑right, y‑up, z‑forward)
        public Vector3    size        = Vector3.one * .05f; // default cube, metres – width / height / depth
    }

    [Header("Ultraleap provider")]
    public LeapXRServiceProvider leapProvider;

    [Header("Finger-tip sphere radius (metres)")]
    public float fingertipRadius = 0.01f;
    private float fingertipDiameter => fingertipRadius * 2f;

    [Header("Global object interaction exclusion list (exact names, case-insensitive)")]
    public string[] excludedObjectNames;
    private HashSet<string> _excluded;

    // -------------------- Hand-tracking helpers -------------------------------
    private const int HANDS   = 2;   // 0 = left, 1 = right
    private const int FINGERS = 5;   // Thumb, Index, Middle, Ring, Pinky
    // fingertipGOs[handIndex, fingerIndex] → proxy GameObject in the scene
    private readonly GameObject[,] fingertipGOs = new GameObject[HANDS, FINGERS];

    // -------------------- Book-keeping for active interactions ----------------
    private readonly struct PairKey : IEquatable<PairKey>
    {
        public readonly int A, B;           // sorted instance IDs
        public PairKey(GameObject a, GameObject b)
        {
            int idA = a.GetInstanceID();
            int idB = b.GetInstanceID();
            // Debug.Log($"PairKey: {a.name} ({idA}) ↔ {b.name} ({idB})");
            if (idA < idB) { A = idA; B = idB; }
            else           { A = idB; B = idA; }
        }
        public bool Equals(PairKey other) => A == other.A && B == other.B;
        public override int GetHashCode()   => (A * 397) ^ B;
    }

    private class InteractionInfo
    {
        public float startTime;  // Time.time when overlap began
        public GameObject objA;  // first object in the pair
        public GameObject objB;  // second object in the pair
        public override string ToString() => $"{objA.name} ↔ {objB.name} @ {startTime:F2}s";
    }

    private readonly Dictionary<PairKey, InteractionInfo> activePairs =
        new Dictionary<PairKey, InteractionInfo>();  // active object pairs
    
    // generated zones (from RelativeZone definitions)
    private List<GameObject> generatedZones = new List<GameObject>();
    // merged originals + generated zones
    private GameObject[] _allInteractables;
    
    // ───────────────────── Awake() – one-time setup ──────────────────────────────
    void Awake()
    {
         _excluded = excludedObjectNames != null
            ? new HashSet<string>(excludedObjectNames, StringComparer.OrdinalIgnoreCase)
            : new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        //------------------------------------------------------------------
        // Create trigger‑collider volumes for each RelativeZone
        //------------------------------------------------------------------
        if (relativeZones != null)
        {
            foreach (var zone in relativeZones)
            {
                if (zone.reference == null)
                {
                    Debug.LogWarning($"[InteractionManager.cs] RelativeZone '{zone.name}' has no reference Transform - skipped.");
                    continue;
                }

                GameObject go = new(zone.name);
                go.transform.SetParent(zone.reference, false);
                go.transform.localPosition = zone.localCenter;
                // Debug.Log($"[InteractionManager] Creating zone: {go.name} at {zone.localCenter} in {zone.reference.name}");
                go.transform.localRotation = Quaternion.identity;

                // BoxCollider as trigger
                var col = go.AddComponent<BoxCollider>();
                col.size      = zone.size;
                col.isTrigger = true;

                // Kinematic Rigidbody so trigger events fire
                var rb = go.AddComponent<Rigidbody>();
                rb.isKinematic = true; rb.useGravity = false;

                // Reporter forwards trigger events back to this manager
                var rep = go.AddComponent<InteractionReporter>();
                rep.manager = this;

                // Optional visualisation (transparent cyan cube)
                if (showZones)
                {
                    var vis = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    Destroy(vis.GetComponent<Collider>());
                    vis.transform.SetParent(go.transform, false);
                    vis.transform.localScale = zone.size;   // match BoxCollider size exactly
                    var rend = vis.GetComponent<Renderer>();
                    rend.material = new Material(Shader.Find("Standard"));
                    rend.material.color = new Color(0, 1, 1, .2f);
                    // set layer to "Objects"
                    vis.layer = LayerMask.NameToLayer("Objects");
                }

                generatedZones.Add(go);
            }
        }
        
        //------------------------------------------------------------------
        // Merge trackedObjects with the generated zones.
        //------------------------------------------------------------------
        var tmp = new List<GameObject>();
        if (trackedObjects != null)
        {
            foreach (var t in trackedObjects)
            {
                if (t == null) continue;
                if (_excluded.Contains(t.name)) continue;   // ignore globals
                tmp.Add(t);
            }
        }
        tmp.AddRange(generatedZones);
        _allInteractables = tmp.ToArray();

        //----------------------------------------------------------------------
        // Ensure every trackedObject has:
        //    • Collider marked as Trigger
        //    • Kinematic Rigidbody (so trigger events fire)
        //    • InteractionReporter (forwards OnTriggerEnter/Exit to this manager)
        //----------------------------------------------------------------------
        foreach (var go in _allInteractables)
        {
            if (go == null) continue;

            // Get existing collider or add a BoxCollider fallback
            Collider col = go.GetComponent<Collider>()
                       ? go.GetComponent<Collider>()
                       : go.AddComponent<BoxCollider>();        // fallback

            col.isTrigger = true;

            // Rigidbody (kinematic so physics won’t move it)
            Rigidbody rb = go.GetComponent<Rigidbody>()
                        ? go.GetComponent<Rigidbody>()
                        : go.AddComponent<Rigidbody>();

            rb.isKinematic = true;
            rb.useGravity  = false;

            // Reporter forwards trigger events back to OverlapManager
            if (!go.TryGetComponent(out InteractionReporter rep))
            {
                rep = go.AddComponent<InteractionReporter>();
            }
            rep.manager = this;
        }

        //----------------------------------------------------------------------
        // Build fingertip proxy spheres
        //----------------------------------------------------------------------

        // Fallback: find the LeapXRServiceProvider if not assigned.
        if (leapProvider == null){
            leapProvider = FindObjectOfType<LeapXRServiceProvider>();
        }

        string[] fingerLabel = { "thumb", "index", "middle", "ring", "pinky" };
        string[] handLabel = { "left", "right" };

        // Create 10 proxy GameObjects (5 per hand)
        for (int h = 0; h < HANDS; h++)
        {
            for (int f = 0; f < FINGERS; f++)
            {
                // Named "Left_index_tip" etc. for easy debugging
                var tip = new GameObject($"{handLabel[h]}_{fingerLabel[f]}_tip");
                // tip.layer = fingertipLayer;                         // put on its own layer
                tip.transform.parent = transform;                   // keep hierarchy tidy

                // Sphere collider as trigger
                var col = tip.AddComponent<SphereCollider>();
                col.isTrigger = true;
                col.radius    = fingertipRadius;

                // Kinematic rigidbody (needed for trigger events)
                var rb = tip.AddComponent<Rigidbody>();
                rb.isKinematic = true; rb.useGravity = false;

                // Interaction reporter so OverlapManager gets callbacks
                var rep = tip.AddComponent<InteractionReporter>();
                rep.manager = this;

                // Create a visual sphere for fingertip (optional, for debugging)
                GameObject vis = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                Destroy(vis.GetComponent<Collider>());          // remove duplicate collider
                vis.transform.SetParent(tip.transform, false);  // zeroed localPos / Rot / Scale, keet at origin of tip
                vis.transform.localScale = Vector3.one * fingertipDiameter;     // set diameter
                var rend = vis.GetComponent<Renderer>();
                rend.material = new Material(Shader.Find("Standard"));
                rend.material.color = Color.yellow;
                // set layer to "Objects"
                vis.layer = LayerMask.NameToLayer("Objects");

                fingertipGOs[h, f] = tip;
                tip.SetActive(false); // start disabled until hand is seen
            }
        }

        ToggleVisualize(); // ensure visuals are set correctly on start
    }

    private void ToggleVisualize(){
        foreach (var go in trackedObjects){
            // Debug.Log($"Setting visuals for tracked object {go.name} to {(show3DObjects ? "visible" : "hidden")}");
            if (go) SetVisuals(go, show3DObjects);
        }

        if (fingertipGOs == null) return;
        for (int h = 0; h < HANDS; h++)
        {
            for (int f = 0; f < FINGERS; f++)
            {
                var go = fingertipGOs[h, f];
                if (go){
                    // Debug.Log($"Setting fingertip visuals for {go.name} to {(showFingertips ? "visible" : "hidden")}");                
                    SetVisuals(go, showFingertips);
                }
            }
        }
    }
    

    // ───────────────────── Editor toggle support ───────────────────────
    // Unity calls when the script is loaded or a value changes in the Inspector.
    private void OnValidate()
    {
        ToggleVisualize();
    }

    // ─────────────────────── set renderer visibility ───────────────────────
    private static void SetVisuals(GameObject root, bool visible)
    {
        // Debug.Log($"Setting visuals for {root.name} to {(visible ? "visible" : "hidden")}");
        foreach (var rend in root.GetComponentsInChildren<Renderer>(true))
            rend.enabled = visible;
    }

    // ───────────────────── Update() – runs every frame ────────────────────
    void Update()
    {
        UpdateFingertips();
        UpdateOverlapPercentages();
    }

    private void UpdateFingertips()
    {
        if (leapProvider == null) return;

        var frame = leapProvider.CurrentFrame;
        bool[,] seen = new bool[HANDS, FINGERS];

        foreach (var hand in frame.Hands)
        {
            int h = hand.IsLeft ? 0 : 1;
            for (int f = 0; f < hand.Fingers.Count && f < FINGERS; f++)
            {
                var tipGO = fingertipGOs[h, f];
                tipGO.transform.position = hand.Fingers[f].TipPosition;
                tipGO.transform.GetChild(0).localScale = Vector3.one * fingertipDiameter;
                if (!tipGO.activeSelf) tipGO.SetActive(true);
                seen[h, f] = true;
            }
        }
        for (int h = 0; h < HANDS; h++)
            for (int f = 0; f < FINGERS; f++)
                if (!seen[h, f] && fingertipGOs[h, f].activeSelf)
                    fingertipGOs[h, f].SetActive(false);
    }

    private void UpdateOverlapPercentages()
    {
        foreach (var pair in activePairs.Values)
        {
            var colA = pair.objA.GetComponent<Collider>();
            var colB = pair.objB.GetComponent<Collider>();
            if (colA == null || colB == null) continue;

            if (!colA.bounds.Intersects(colB.bounds)) continue;
            if (!ComputeIntersection(colA.bounds, colB.bounds, out var inter)) continue;

            float volA = Volume(colA.bounds);
            float volB = Volume(colB.bounds);
            float volInter = Volume(inter);

            // percentage of A contained in B
            float pctA = (volInter / volA) * 100f;
            // percentage of B contained in A
            float pctB = (volInter / volB) * 100f;
            // iou (Intersection over Union)
            float iou = (volInter / (volA + volB - volInter)) * 100f;

            // Debug.Log($"{"[Interaction]".Colorize(Color.green)}: "+
            //           $"{pair.objA.name.Colorize(Color.magenta)} in {pair.objB.name.Colorize(Color.magenta)}: {pctA:F2}% | " +
            //           $"{pair.objB.name.Colorize(Color.magenta)} in {pair.objA.name.Colorize(Color.magenta)}: {pctB:F2}% | IoU: {iou:F2}%");
        }
    }

    private static float Volume(Bounds b) => b.size.x * b.size.y * b.size.z;

    // AABB intersection bounds
    private static bool ComputeIntersection(Bounds A, Bounds B, out Bounds o)
    {
        Vector3 min = Vector3.Max(A.min, B.min);
        Vector3 max = Vector3.Min(A.max, B.max);
        bool intersects = (min.x < max.x && min.y < max.y && min.z < max.z);
        o = new Bounds();
        if (intersects) o.SetMinMax(min, max);
        return intersects;
    }

    // ───────────────────────── Trigger Callbacks ─────────────────────────────
    internal void NotifyEnter(GameObject a, GameObject b)
    {
        if (ShouldIgnore(a, b)) return;

        var key = new PairKey(a, b);
        if (activePairs.ContainsKey(key)) return;           // already overlapping
        activePairs[key] = new InteractionInfo
        {
            startTime = Time.time,
            objA = a,
            objB = b
        };
        Debug.Log($"{"▶ Interaction START".Colorize(Color.green)}: {a.name.Colorize(Color.magenta)} ↔ {b.name.Colorize(Color.magenta)}");
        OnInteractionStart?.Invoke(a, b);
    }

    internal void NotifyExit(GameObject a, GameObject b)
    {
        if (ShouldIgnore(a, b)) return; // skip ignored pairs

        var key = new PairKey(a, b);
        if (!activePairs.TryGetValue(key, out var info)) return;

        float duration = Time.time - info.startTime;
        activePairs.Remove(key);    // remove from active 

        Debug.Log($"{"■ Interaction END".Colorize(Color.red)}: {a.name.Colorize(Color.magenta)} ↔ {b.name.Colorize(Color.magenta)}  ({duration:F2}s)");
        OnInteractionEnd?.Invoke(a, b);
    }

    private bool ShouldIgnore(GameObject a, GameObject b)
    {
        // anything explicitly in the global list
        if (_excluded.Contains(a.name) || _excluded.Contains(b.name))
            return true;

        // ignore fingertip ↔ fingertip interactions)
        if (a.name.EndsWith("_tip", StringComparison.OrdinalIgnoreCase) && 
            b.name.EndsWith("_tip", StringComparison.OrdinalIgnoreCase))
        {
            return true; // skip fingertip pairs
        }

        // ignore coffee machine ↔ coffee machine subparts interactions
        if (a.name.StartsWith("coffee", StringComparison.OrdinalIgnoreCase) &&
            b.name.StartsWith("coffee", StringComparison.OrdinalIgnoreCase))
        {
            return true; // skip coffee pairs
        }

        return false;
    }
}

// ───────────────────────── Interaction Reporter ──────────────────────────
// Add this to any object with a trigger-collider.
// It forwards trigger enter/exit events to its InteractionManager.
public sealed class InteractionReporter : MonoBehaviour
{
    [HideInInspector] public InteractionManager manager;
    void OnTriggerEnter(Collider other)
    {
        if (manager) manager.NotifyEnter(gameObject, other.gameObject);
    }
    void OnTriggerExit(Collider other)
    {
        if (manager) manager.NotifyExit(gameObject, other.gameObject);
    }
}
