using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class BVHManager : MonoBehaviour
{
    public BVHRecorder leftRecorder;
    public BVHRecorder rightRecorder;
    public int recordingTimeLimit = 0;
    public string filename;
    public VarjoApiManager varjoApiManager;

    private bool capturing = false;

    private Color redColor = new Color(1f, 0.1f, 0.1f, 1f);
    private Color whiteColor = new Color(1f, 1f, 1f, 1f);

    private float timeElapsed = 0f;
    private bool isCounting = false;

    private TextMeshProUGUI recordbuttonText;

    // Start is called before the first frame update
    void Start()
    {
       
    }

    // Update is called once per frame
    void Update()
    {
        RunTimer();
    }

    public void RecordAnimation(Button button)
    {
        recordbuttonText = button.GetComponentInChildren<TextMeshProUGUI>();

        leftRecorder.voilatile = false;
        rightRecorder.voilatile = false;

        leftRecorder.filename = filename + "_left";
        rightRecorder.filename = filename + "_right";

        leftRecorder.directory = Application.dataPath + "/../Output/BVHdata/";
        rightRecorder.directory = Application.dataPath + "/../Output/BVHdata/";

        ToggleCapturing();

        if (!capturing)
        {
            SaveRecording();
            recordbuttonText.SetText("Record");
            recordbuttonText.color = whiteColor;
        }
        else
        {
            isCounting = true;
            recordbuttonText.SetText("Stop");
            recordbuttonText.color = redColor;
        }
    }

    private void ToggleCapturing()
    {
        capturing = !capturing;
        if (capturing) {
            leftRecorder.clearCapture();
            rightRecorder.clearCapture();
        }
        varjoApiManager.capturing = capturing;
        leftRecorder.capturing = capturing;
        rightRecorder.capturing = capturing;
     }

    private void SaveRecording()
    {
        leftRecorder.saveBVH();
        rightRecorder.saveBVH();
        varjoApiManager.SaveCameraXTrinsics();
    }

    private void RunTimer()
    {
        if (isCounting)
        {
            timeElapsed += Time.deltaTime;
            if (timeElapsed >= recordingTimeLimit && recordingTimeLimit != 0)
            {
                StopTimer();
            }
        }
    }
    private void StopTimer()
    {
        isCounting = false;
        timeElapsed = 0f;
        ToggleCapturing();
        SaveRecording();
        recordbuttonText.SetText("Record");
        recordbuttonText.color = whiteColor;
        Debug.Log("Recording stopped!");
    }
}
