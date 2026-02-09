using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine.UI;


public static class Utility
{
    public static Coroutine ExecuteLater(this MonoBehaviour behaviour, float delay, System.Action fn)
    {
        return behaviour.StartCoroutine(_realExecute(delay, fn));
    }
    static IEnumerator _realExecute(float delay, System.Action fn)
    {
        yield return new WaitForSeconds(delay);
        fn();
    }
}

public class TextToSpeechPlayer : MonoBehaviour {
    [DllImport("WindowsVoice")]
    public static extern void initSpeech();
    [DllImport("WindowsVoice")]
    public static extern void destroySpeech();
    [DllImport("WindowsVoice")]
    public static extern void addToSpeechQueue(string s);
    [DllImport("WindowsVoice")]
    public static extern void clearSpeechQueue();
    [DllImport("WindowsVoice")]
    public static extern void statusMessage(StringBuilder str, int length);
    public static TextToSpeechPlayer theVoice = null;
	
    // Use this for initialization
    void OnEnable () {
        if (theVoice == null)
        {
            theVoice = this;
            initSpeech();
            // Debug.Log("Speech initialized");
            // speak("Welcome to the Luminous Project!");
            speak("Select your coffee from the menu and press start.");
        }
    }
    
    public void test()
    {
        speak("Testing");
    }

    public static void speak(string msg, float delay = 0f) 
    {
        if ( delay == 0f )
            addToSpeechQueue(msg);
        else
            theVoice.ExecuteLater(delay, () => speak(msg));
    }

    void OnDestroy()
    {
        if (theVoice == this)
        {
            Debug.Log("Destroying speech");
            destroySpeech();
            Debug.Log("Speech destroyed");
            theVoice = null;
        }
    }

    public static string GetStatusMessage()
    {
        StringBuilder sb = new StringBuilder(40);
        statusMessage(sb, 40);
        return sb.ToString();
    }

    public static void Enable()
    {
        if (theVoice != null) return;                      // already alive

        // Create a hidden GameObject that survives scene loads
        var go = new GameObject("[TTS]");
        DontDestroyOnLoad(go);                             // keep alive
        theVoice = go.AddComponent<TextToSpeechPlayer>();  // Awake → initSpeech()
    }

    public static void Disable()
    {
        if (theVoice == null) return;          // nothing to do

        destroySpeech();                       // P/Invoke to DLL
        Destroy(theVoice.gameObject);          // destroys component + GO
        theVoice = null;
    }
}
