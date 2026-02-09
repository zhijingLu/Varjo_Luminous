using System.Collections.Generic;
using UnityEngine;
using Leap;
using Leap.Unity;

public class HandVisualizer : MonoBehaviour
{
    [Header("Required: assign your LeapXRServiceProvider here")]
    public LeapXRServiceProvider leapProvider;

    [Header("Joint‐sphere prefab (e.g. a small sphere)")]
    public GameObject spherePrefab;

    [Header("Diameter of each sphere in meters")]
    public float sphereDiameter = 0.01f;

    public bool debugMode = true; // Toggle for debug logging

    // Will hold a parent object for each hand ("LeftHand" or "RightHand")
    private Dictionary<string, Transform> handParents = new Dictionary<string, Transform>();

    // Will map a joint‐name string (e.g. "Left_INDEX_PROXIMAL") → the sphere's Transform
    private Dictionary<string, Transform> jointSpheres = new Dictionary<string, Transform>();

    void Start()
    {
        if (leapProvider == null)
        {
            leapProvider = FindObjectOfType<LeapXRServiceProvider>();
            if (leapProvider == null)
            {
                Debug.LogError("[HandVisualizer] No LeapXRServiceProvider found in scene.");
                enabled = false;
                return;
            }
        }

        if (spherePrefab == null)
        {
            // If you forgot to assign a prefab, fall back to Unity’s default sphere:
            spherePrefab = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            Destroy(spherePrefab.GetComponent<Collider>()); 
            // remove collider (optional)
        }

        // Ensure prefab has correct scale so that Instantiated spheres are sphereDiameter in size.
        spherePrefab.transform.localScale = Vector3.one * sphereDiameter;
        spherePrefab.SetActive(false);
    }

    void Update()
    {
        Frame frame = leapProvider.CurrentFrame;

        foreach (Leap.Hand hand in frame.Hands)
        {
            string handLabel = hand.IsLeft ? "Left" : "Right";

            // Ensure we have a parent Transform for this hand
            if (!handParents.ContainsKey(handLabel))
            {
                GameObject hParentGO = new GameObject(handLabel + "Hand");
                hParentGO.transform.parent = this.transform;
                hParentGO.transform.localPosition = Vector3.zero;
                hParentGO.transform.localRotation = Quaternion.identity;
                handParents[handLabel] = hParentGO.transform;
            }

            Transform handParent = handParents[handLabel];

            // 1) Elbow joint
            {
                Vector3 leapElbow = hand.Arm.ElbowPosition; // in mm, Leap space
                // Vector3 leapElbow = leapProvider.transform.TransformPoint(leapElbow * 0.001f);
                string key = handLabel + "_ELBOW";
                PlaceOrMoveSphere(key, leapElbow, handParent);
            }

            // 2) Wrist joint
            {
                Vector3 leapWrist = hand.WristPosition; // in mm
                // Vector3 leapWrist = leapProvider.transform.TransformPoint(leapWrist * 0.001f);
                string key = handLabel + "_WRIST";
                PlaceOrMoveSphere(key, leapWrist, handParent);
            }

            // 3) Palm joint (center)
            {
                Vector3 leapPalm = hand.PalmPosition; // in mm
                // Vector3 worldPalm = leapProvider.transform.TransformPoint(leapPalm * 0.001f);
                string key = handLabel + "_PALM";
                PlaceOrMoveSphere(key, leapPalm, handParent);
            }

            // 4) For each finger, for each bone, visualize the NextJoint point
            foreach (Leap.Finger finger in hand.Fingers)
            {
                string fLabel = finger.Type.ToString(); // "TYPE_INDEX", etc.
                for (int b = 0; b < 4; b++)
                {
                    Bone.BoneType boneType = (Bone.BoneType)b;
                    Leap.Bone bone = finger.Bone(boneType);

                    Vector3 leapJointPos = bone.NextJoint; // in mm
                    // Vector3 worldJoint = leapProvider.transform.TransformPoint(leapJointPos * 0.001f);

                    string key = $"{handLabel}_{fLabel}_{boneType}";
                    PlaceOrMoveSphere(key, leapJointPos, handParent);
                }
            }
        }
    }

    // Creates a sphere (once) for this key under parent, or moves it if it already exists.
    private void PlaceOrMoveSphere(string key, Vector3 worldPos, Transform parent)
    {
        if (!jointSpheres.ContainsKey(key))
        {
            // Instantiate a new sphere for this joint
            GameObject go = Instantiate(spherePrefab, worldPos, Quaternion.identity, parent);
            go.name = key;
            go.SetActive(true);
            jointSpheres[key] = go.transform;
        }
        else
        {
            // Move existing sphere
            jointSpheres[key].position = worldPos;
        }

        if (debugMode){
            Debug.Log($"[HandVisualizer] Creating sphere for {key} at {worldPos}, {parent}");
        }
    }
}
