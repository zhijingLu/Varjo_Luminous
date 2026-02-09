using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using Leap;
using Leap.Unity;
using Varjo.XR;


public class HandObjectInteractionManager : MonoBehaviour
{
    public LeapXRServiceProvider leapProvider;
    public bool debugMode = true; // Toggle for debug logging
    
    // Start is called before the first frame update
    void Start()
    {
        if (leapProvider == null)
            leapProvider = FindObjectOfType<LeapXRServiceProvider>();
    }

    // Update is called once per frame
    void Update()
    {
        // get hands data for current frame
        Frame frame = leapProvider.CurrentFrame;
        StringBuilder debugLog = new StringBuilder();

        foreach (Leap.Hand hand in frame.Hands)
        {
            string handType = hand.IsLeft ? "Left" : "Right";

            // Elbow
            Vector3 elbowPosition = hand.Arm.ElbowPosition;
            Quaternion elbowRotation = hand.Arm.Rotation;

            // Palm
            Vector3 palmPosition = hand.PalmPosition;
            Quaternion palmRotation = hand.Rotation;

            // Wrist
            Vector3 wristPosition = hand.WristPosition;

            // Store finger data for printing later
            StringBuilder fingerInfo = new StringBuilder();

            foreach (Leap.Finger finger in hand.Fingers)
            {
                if (finger.Type == Leap.Finger.FingerType.TYPE_INDEX)
                {
                    Vector3 fingerTipPosition = finger.TipPosition;
                    // Debug.Log($"Finger Tip Position: {fingerTipPosition}");
                    // calculate norm of finger tip position and marker position
                    // float distanceToMarker = Vector3.Distance(fingerTipPosition, markerPosition);
                    // Debug.Log($"Distance to Marker 201: {distanceToMarker}");
                    Vector3 fingerDirection = finger.Direction; // It's a Vector, not Quaternion

                    fingerInfo.AppendLine($"\tFinger: {finger.Type}");
                    fingerInfo.AppendLine($"\t\tTip Position: {fingerTipPosition}");
                    fingerInfo.AppendLine($"\t\tDirection: {fingerDirection}");

                    for (int b = 0; b < 4; b++)
                    {
                        Leap.Bone bone = finger.Bone((Leap.Bone.BoneType)b);
                        Vector3 bonePosition = bone.NextJoint;
                        Quaternion boneRotation = bone.Rotation;
                        float boneLength = bone.Length;

                        fingerInfo.AppendLine($"\t\tBone: {(Leap.Bone.BoneType)b}");
                        fingerInfo.AppendLine($"\t\t\tPosition: {bonePosition}");
                        fingerInfo.AppendLine($"\t\t\tRotation: {boneRotation}");
                        fingerInfo.AppendLine($"\t\t\tLength: {boneLength}");
                    }
                }
            }

            // If debug mode, assemble the log
            if (debugMode)
            {
                debugLog.AppendLine($"Hand type: {handType}");
                debugLog.AppendLine($"Elbow Position: {elbowPosition}");
                debugLog.AppendLine($"Elbow Rotation: {elbowRotation}");
                debugLog.AppendLine($"Palm Position: {palmPosition}");
                debugLog.AppendLine($"Palm Rotation: {palmRotation}");
                debugLog.AppendLine($"Wrist Position: {wristPosition}");
                debugLog.Append(fingerInfo.ToString());
            }
        }

        // Only print once per Update, if debugMode
        if (debugMode && debugLog.Length > 0)
        {
            Debug.Log(debugLog.ToString());
        }
    }
}
