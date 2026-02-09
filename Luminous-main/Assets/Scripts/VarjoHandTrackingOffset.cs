using UnityEngine;
using UnityEngine.XR;
using Leap;
using Leap.Unity;

[RequireComponent(typeof(LeapXRServiceProvider))]
public class VarjoHandTrackingOffset : MonoBehaviour
{
        private InputDevice hmd;
        public LeapXRServiceProvider xrServiceProvider;

        void Start()
        {
            hmd = InputDevices.GetDeviceAtXRNode(XRNode.Head);
            xrServiceProvider = GetComponent<LeapXRServiceProvider>();

            switch (hmd.name)
            {
                case "XR-3":
                case "VR-3":
                        xrServiceProvider.deviceOffsetMode = LeapXRServiceProvider.DeviceOffsetMode.ManualHeadOffset;
                        xrServiceProvider.deviceOffsetYAxis = -0.0112f;
                        xrServiceProvider.deviceOffsetZAxis = 0.0999f;
                        xrServiceProvider.deviceTiltXAxis = 0f;
                        Debug.Log(
                        "--- Varjo Hand Tracking Offset: Using XR-3 or VR-3 settings. ---\n" +
                        $"Device Offset Mode: {xrServiceProvider.deviceOffsetMode}\n" +
                        $"Device Offset Y Axis: {xrServiceProvider.deviceOffsetYAxis}\n" +
                        $"Device Offset Z Axis: {xrServiceProvider.deviceOffsetZAxis}\n" +
                        $"Device Tilt X Axis: {xrServiceProvider.deviceTiltXAxis}"
                        );              
                        break;
                case "VR-2 Pro":
                        xrServiceProvider.deviceOffsetMode = LeapXRServiceProvider.DeviceOffsetMode.ManualHeadOffset;
                        xrServiceProvider.deviceOffsetYAxis = -0.025734f;
                        xrServiceProvider.deviceOffsetZAxis = 0.068423f;
                        xrServiceProvider.deviceTiltXAxis = 5f;
                        break;
            }
        }

        void Update()
        {

        }
}