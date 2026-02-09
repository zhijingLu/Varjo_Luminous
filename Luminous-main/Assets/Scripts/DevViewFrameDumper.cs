/*  DevViewFrameDumper.cs  ── drop on an always-alive GameObject
 *  Dumps one PNG per frame at captureFPS into:
 *  %AppData%\LocalLow\<Company>\<Product>\DevViewFrames\   (Win)
 *  ~/Library/Application Support/.../DevViewFrames/       (macOS)
 */
using System;
using System.Collections.Concurrent;
using System.IO;
using System.Threading.Tasks;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

using UnityEngine.Rendering;

public class DevViewFrameDumper : MonoBehaviour
{
    [Header("Source & visibility")]
    [Tooltip("Leave empty → Camera.main after Start()")]
    public Camera hmdCamera;
    [Tooltip("0 → (Everything-UI)+DevOverlay")]
    public LayerMask devMask;

    [Header("Capture")]
    public int width  = 1920;
    public int height = 1080;
    [Range(1,120)]
    public int captureFPS = 30;

    /* ─────────────────────────────────────────────────────────── */

    Camera   devCam;
    RenderTexture rt;
    string   outDir;
    float    interval;
    ConcurrentQueue<(NativeArray<byte>, int)> queue = new();

    void Start()
    {
        /* ── 1. find the HMD camera ─────────────────────────── */
        if (!hmdCamera) hmdCamera = Camera.main;
        if (!hmdCamera)
        {
            Debug.LogError("[DevView] No HMD camera found - capture disabled."); 
            enabled = false;  return;
        }

        /* ── 2. build default mask (Everything-UI)+DevOverlay ─ */
        if (devMask.value == 0)
        {
            int ui    = LayerMask.NameToLayer("UI");
            int debug = LayerMask.NameToLayer("DevOverlay");
            devMask   = ~0 & ~(1 << ui) | (1 << debug);
        }

        /* ── 3. prep output dir ─────────────────────────────── */
        outDir = Path.Combine(Application.persistentDataPath, "DevViewFrames");
        Directory.CreateDirectory(outDir);
        Debug.Log($"[DevView] dumping PNGs to: {outDir}");

        /* ── 4. build dev camera & RT ───────────────────────── */
        rt  = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32)
        { name = "DevViewRT" };

        devCam = new GameObject("DevViewCamera").AddComponent<Camera>();
        devCam.CopyFrom(hmdCamera);
        devCam.stereoTargetEye = StereoTargetEyeMask.None;
        devCam.cullingMask     = devMask;
        devCam.targetTexture   = rt;
        devCam.enabled         = false;      // we render manually

        /* ── 5. start capture loop ──────────────────────────── */
        interval = 1f / captureFPS;
        StartCoroutine(Capture());
    }

    System.Collections.IEnumerator Capture()
    {
        var eof = new WaitForEndOfFrame();
        float next = Time.unscaledTime;

        while (true)
        {
            yield return eof;

            /* throttle to captureFPS */
            if (Time.unscaledTime < next) continue;
            next += interval;

            /* mirror pose + projection each frame */
            devCam.transform.SetPositionAndRotation(
                hmdCamera.transform.position, hmdCamera.transform.rotation);
            devCam.projectionMatrix = hmdCamera.projectionMatrix;

            /* render & async readback */
            devCam.Render();
            int frame = Time.frameCount;

            AsyncGPUReadback.Request(rt, 0, TextureFormat.RGBA32,
                req => {
                    if (req.hasError) { Debug.LogWarning("[DevView] Readback error"); return; }
                    queue.Enqueue((req.GetData<byte>(), frame));
                });

            /* flush any completed readbacks → disk */
            while (queue.TryDequeue(out var item))
                SaveNativeArray(item.Item1, item.Item2);
        }
    }

void SaveNativeArray(NativeArray<byte> native, int frame)
{
    /* capture thread-unsafe data while we’re still on the main thread */
    GraphicsFormat gf = rt.graphicsFormat;          // ← SAFE here
    string pngPath   = Path.Combine(outDir, $"dev_{frame:D06}.png");

    Task.Run(() =>
    {
        try
        {
#if UNITY_2022_2_OR_NEWER
            byte[] png = ImageConversion.EncodeNativeArrayToPNG(
                            native, gf, (uint)width, (uint)height);
#else
            byte[] png = ImageConversion.EncodeArrayToPNG(
                            native.ToArray(), gf, (uint)width, (uint)height);
#endif
            File.WriteAllBytes(pngPath, png);       // pure .NET → background OK
        }
        catch (Exception e) { Debug.LogException(e); }
        finally            { native.Dispose(); }
    });
}

    void OnDestroy()
    {
        if (rt) rt.Release();
    }
}
