using System;

using System.IO;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;
using DFKI_Utilities;
using System.Threading.Tasks;
using Varjo.XR;

public class VarjoApiManager : MonoBehaviour
{
    private class VarjoStream
    {
        [DllImport("VarjoStream")]
        unsafe public static extern void StartStreaming();

        [DllImport("VarjoStream")]
        unsafe public static extern void StopStreaming();

        [DllImport("VarjoStream")]
        unsafe public static extern float* GetLeftImg(); // byte*

        [DllImport("VarjoStream")]
        unsafe public static extern float* GetRightImg(); // byte*

        [DllImport("VarjoStream")]
        unsafe public static extern double* GetLeftExtrinsics();

        [DllImport("VarjoStream")]
        unsafe public static extern double* GetRightExtrinsics();

        [DllImport("VarjoStream")]
        unsafe public static extern double* GetLeftIntrinsics();

        [DllImport("VarjoStream")]
        unsafe public static extern double* GetRightIntrinsics();

        [DllImport("VarjoStream")]
        unsafe public static extern double* GetIntrinsicsUndistorted();

        [DllImport("VarjoStream")]
        unsafe public static extern void LockImages();

        [DllImport("VarjoStream")]
        unsafe public static extern void UnlockImages();

        [DllImport("VarjoStream")]
        unsafe public static extern void LockXtrinsics();

        [DllImport("VarjoStream")]
        unsafe public static extern void UnlockXtrinsics();

        [DllImport("VarjoStream")]
        unsafe public static extern void SetUndistort(bool activate);

        [DllImport("VarjoStream")]
        unsafe public static extern void SetThresholding(bool activate, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min);

        [DllImport("VarjoStream")]
        unsafe public static extern float* GetPointcloudVertices();

        [DllImport("VarjoStream")]
        unsafe public static extern float* GetPointcloudColors();

        [DllImport("VarjoStream")]
        unsafe public static extern float* GetPointcloudNormals();

        [DllImport("VarjoStream")]
        unsafe public static extern int* GetPointcloudIndices();
    }

    // controlling variables
    public bool capturing { get; set; } = false;
    public bool undistort = true;
    private bool streamEnabled = false;
    public bool writeImages = false;

    // current public settings
    public RawImage leftImg, rightImg;
    public Texture2D leftTexture, rightTexture;
    public CameraView cameraLeft, cameraRight;
    private float[] arrayL, arrayR;
    private byte[] leftB, rightB;

    // thresholding settings
    private bool thresholding = false;
    public float H_max = 180, H_min = 0, V_max = 255, V_min = 0, S_max = 255, S_min = 0;

    // hidden settings
    public int width = 1152;
    public int height = 1152;
    public int numChannels = 3;//4;
    public int bytesPerChannel = 1; // i.e., 8 bits
    private int imgDim;
    private int leftImgs = 1;
    private int rightImgs = 1;
    private double[] extr_l, extr_r, intr_l, intr_r, intr_und;

   
    //public ActivityOne activityOne;
    //public ActivityTwo activityTwo;

    private void InitXRMode()
    {
        VarjoRendering.SetOpaque(false);
        VarjoMixedReality.StartRender();
        Camera camera = Camera.main;
        UnityEngine.Color solid_black = new(0, 0, 0, 0);  // hide 3d environment by setting alpha to 0
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.backgroundColor = solid_black;
    }

    public void SetUndistort(bool set)
    {
        undistort = set;
        unsafe
        {
            VarjoStream.SetUndistort(undistort);
        }

        if (undistort)
            Debug.Log("Enabling undistortion of streamed images");
        else
            Debug.LogWarning("Warning: Disabling undistortion of streamed images: The streamed images will require external undistortion!");
    }

    public void SetThresholding(bool set)
    {
        thresholding = set;
        unsafe
        {
            VarjoStream.SetThresholding(thresholding, H_max, H_min, S_max, S_min, V_max, V_min);
        }

        if (thresholding)
            Debug.Log("Enabling thresholding of streamed images with values: [" + H_max + ", " + H_min + ", " + S_max + ", " + S_min + ", " + V_max + ", " + V_min + "]");
        else
            Debug.Log("Disabling thresholding of streamed images");
    }

    public void Enable()
    {
        if (!streamEnabled)
        {
            VarjoStream.StartStreaming();
            streamEnabled = true;
        }

        // Enable Depth Estimation.
        // VarjoMixedReality.EnableDepthEstimation();
    }

    public void Disable()
    {
        if (streamEnabled)
        {
            VarjoStream.StopStreaming();
            streamEnabled = false;
        }

        // Disable Depth Estimation.
        // VarjoMixedReality.DisableDepthEstimation();
    }

    public bool IsEnabled()
    {
        return streamEnabled;
    }

    // Start is called before the first frame update
    void Start()
    {
        // TextToSpeechPlayer.speak("Welcome to the Luminous Project!");
        
        imgDim = width * height * numChannels * bytesPerChannel;
        
        rightTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        leftTexture = new Texture2D(width, height, TextureFormat.RGB24, false);

        cameraLeft = new CameraView("left", width, height);
        cameraRight = new CameraView("right", width, height);

        extr_l = new double[16];
        extr_r = new double[16];
        intr_l = new double[10];
        intr_r = new double[10];
        intr_und = new double[10];

        arrayL = new float[imgDim];
        arrayR = new float[imgDim];
        leftB = new byte[imgDim];
        rightB = new byte[imgDim];

        if (UnityEngine.XR.XRSettings.enabled)
        {
            Debug.Log("XR is active.");
        }
        else
        {
            Debug.Log("XR is not available.");
        }

        SetUndistort(undistort);
        // Enable();
        InitXRMode();

       
    }

    // Update is called once per frame
    void Update()
    {
        //if (Input.GetKeyDown(KeyCode.Alpha1)){
        //    TextToSpeechPlayer.Enable();
        //    if (activityOne != null){
        //        activityOne.Begin();
        //    }
        //}

        //if (Input.GetKeyDown(KeyCode.Alpha2)){
        //    TextToSpeechPlayer.Enable();
        //    if (activityTwo != null){
        //        activityTwo.Begin();
        //    }
        //}

        //if (Input.GetKeyDown(KeyCode.C)){
        //    Debug.Log($"{"Canceling all activities.".Colorize(Color.red)}");
        //    TextToSpeechPlayer.speak("Cancelling all activities", 0.5f);
        //    if (activityOne != null){
        //        activityOne.Cleanup();
        //    }

        //    if (activityTwo != null){
        //        activityTwo.Cleanup();
        //    }
            
        //    TextToSpeechPlayer.clearSpeechQueue();
        //    TextToSpeechPlayer.Disable();
        //}

        Enable();
        UpdateImages();
    }

    public void UpdateTexture2D()
    {
        // convert from floats to bytes
        Parallel.For(0, imgDim, i => {
            leftB[i] = (byte)(arrayL[i] * 255.0f);
            rightB[i] = (byte)(arrayR[i] * 255.0f);
        });

        leftTexture.LoadRawTextureData(leftB);
        leftTexture.Apply();
        leftImg.GetComponent<RawImage>().texture = leftTexture;

        rightTexture.LoadRawTextureData(rightB);
        rightTexture.Apply();
        rightImg.GetComponent<RawImage>().texture = rightTexture;

        //Writer.FlipImage(ref leftTexture);
        //Writer.FlipImage(ref rightTexture);
    }

    void UpdateImages()
    {
        unsafe
        {
            VarjoStream.LockImages();

            float* imgptrL = VarjoStream.GetLeftImg();
            Marshal.Copy((IntPtr)imgptrL, arrayL, 0, imgDim);

            float* imgptrR = VarjoStream.GetRightImg();
            Marshal.Copy((IntPtr)imgptrR, arrayR, 0, imgDim);

            VarjoStream.UnlockImages();
            // Debug.Log($"Updated images: {arrayL.Length} left, {arrayR.Length} right");

            if (arrayL != null && arrayR != null)
            {
                // Debug.Log("Updating images...");
                // convert from floats to bytes
                Parallel.For(0, imgDim, i => {
                    leftB[i] = (byte)(arrayL[i] * 255.0f);
                    rightB[i] = (byte)(arrayR[i] * 255.0f);
                });

                leftTexture.LoadRawTextureData(leftB);
                leftTexture.Apply();
                leftImg.GetComponent<RawImage>().texture = leftTexture;

                rightTexture.LoadRawTextureData(rightB);
                rightTexture.Apply();
                rightImg.GetComponent<RawImage>().texture = rightTexture;

                if (writeImages)
                {
                    WriteImages();
                }
            }
        }
    }

    void WriteImages()
    {
        string path = Application.dataPath + "/../Output/Images/";
        string leftFile = path + "left_" + leftImgs.ToString("D4") + ".png";
        string rightFile = path + "right_" + rightImgs.ToString("D4") + ".png";

        Writer.FlipImage(ref leftTexture);
        Writer.FlipImage(ref rightTexture);

        // save images
        System.IO.File.WriteAllBytes(leftFile, leftTexture.EncodeToPNG());
        System.IO.File.WriteAllBytes(rightFile, rightTexture.EncodeToPNG());

        // Debug.Log("Wrote images: " + leftFile + ", " + rightFile);

        leftImgs++;
        rightImgs++;
    }
        
    public void UpdateXtrinsics()
    {
        unsafe
        {
            VarjoStream.LockXtrinsics();

            double* extr_l_ptr = VarjoStream.GetLeftExtrinsics();
            double* intr_l_ptr = VarjoStream.GetLeftIntrinsics();
            double* extr_r_ptr = VarjoStream.GetRightExtrinsics();
            double* intr_r_ptr = VarjoStream.GetRightIntrinsics();
            double* intr_und_ptr = VarjoStream.GetIntrinsicsUndistorted();

            Marshal.Copy((IntPtr)extr_l_ptr, extr_l, 0, 16);
            Marshal.Copy((IntPtr)extr_r_ptr, extr_r, 0, 16);
            Marshal.Copy((IntPtr)intr_l_ptr, intr_l, 0, 10);
            Marshal.Copy((IntPtr)intr_r_ptr, intr_r, 0, 10);
            Marshal.Copy((IntPtr)intr_und_ptr, intr_und, 0, 10);
            // intr_und[2] = 0.5;
            // intr_und[3] = 0.5;

            VarjoStream.UnlockXtrinsics();
        }

        cameraLeft.SetXTrinsics(intr_und, extr_l);
        cameraRight.SetXTrinsics(intr_und, extr_r);

        //Debug.Log("Updated XTrinsics:\n" + cameraLeft.K + "\n" + cameraLeft.Rt + "\n" + cameraRight.K + "\n" + cameraRight.Rt);
    }

    public void SaveCameraXTrinsics()
    {
        string header = "# Double precision 4x4 matrix. The matrix usage convention is that they are stored in column-major order and transforms are stacked before column-vector points when multiplying. That is, a pure translation matrix will have the position offset in elements 12..14. Unless otherwise specified, the coordinate system is right-handed: X goes right, Y goes up and negative Z goes forward.";
        SaveArrayToFile("RightCameraExtrinsics.txt", extr_r, 16, header);
        SaveArrayToFile("LeftCameraExtrinsics.txt", extr_l, 16, header);

        header = "# principalPointX principalPointY focalLengthX focalLengthY distortionCoefficient_1 distortionCoefficient_2 distortionCoefficient_3 distortionCoefficient_4 distortionCoefficient_5 distortionCoefficient_6";
        SaveArrayToFile("RightCameraIntrinsics.txt", intr_r, 10, header);
        SaveArrayToFile("LeftCameraIntrinsics.txt", intr_l, 10, header);
        SaveArrayToFile("CameraIntrinsicsUndistorted.txt", intr_und, 10, header);
    }

    void SaveArrayToFile(string filename, double[] array, int arrayLen, string header)
    {
        string path = Application.dataPath + "/../Output/" + filename;

        using (StreamWriter sw = new StreamWriter(path))
        {
            sw.WriteLine(header);

            for(int i=0; i < arrayLen; i++)
            {
                sw.WriteLine(array[i].ToString());
            }
        }
        Debug.Log("Data saved successfully");
    }

    void OnApplicationQuit()
    {
        Disable();
    }
}
