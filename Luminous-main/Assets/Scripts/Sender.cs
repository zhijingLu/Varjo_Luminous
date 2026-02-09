using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.UI;
using DFKI_Utilities;
using System.Threading.Tasks;
using Varjo.XR;
using System.Collections.Concurrent;
using System.Threading;
using System.Diagnostics;
public class Sender : MonoBehaviour
{
    [Header("Frame Source")]
    public VarjoApiManager varjoApiManager;

    [Header("Network")]
    public string host = "127.0.0.1";
    public int port = 5001;
    public int socketTimeoutMs = 2000;

    [Header("hidden settings")]
    private int width = 1152;
    private int height = 1152;
    private int numChannels = 3;//4;i
    private int bytesPerChannel = 1; // i.e., 8 bits
    private int imgDim;
    [Header("Rate Control")]
    public int targetFPS = 40;

    public bool autoStart = false;
    private bool started = false;      //  prevent double-start
    private bool initialized = false;

    [Header("Queue Control")]
    public int maxQueueItems = 4;

    private TcpClient client;
    private NetworkStream networkStream;

    private CancellationTokenSource sendCts;
    private Task senderTask;
    private volatile bool shuttingDown = false;

    private readonly object streamWriteLock = new object();

    // We send left then right with the same meta frameId
    private readonly System.Collections.Concurrent.ConcurrentQueue<(byte[] buffer, int eyeId, long frameId)> sendQueue
        = new System.Collections.Concurrent.ConcurrentQueue<(byte[], int, long)>();

    private float frameInterval;
    private float lastSendTime;

    //buffer
    public StereoFrameBuffer frameBuffer;
    private readonly object frameBufferLock = new object();
    // Your protocol constants (match your Python side)
    private const int dtypeCode = 0;



    void Start()
    {
        frameInterval = (targetFPS <= 0) ? 0.5f : (1f / targetFPS);

        if (autoStart)
        {
            StartStream();


        }
    }

    public void StartStream()
    {
        if (varjoApiManager == null)
        {
            UnityEngine.Debug.LogError("[VarjoTcpStreamSender] varjoApiManager is not assigned.");
            return;
        }
        if (started) return;
        started = true;

        Initialize();
        shuttingDown = false;
        ConnectToServer();
        StartSenderThread();

        UnityEngine.Debug.Log("[VarjoTcpStreamSender] Started.");
    }

    /// <summary>
    /// Stops streaming and releases network resources.
    ///
    /// Cancels the background sender task, disconnects the socket,
    /// and prevents further frame enqueuing/transmission.
    /// </summary>
    /// <remarks>
    /// Safe to call multiple times.
    /// </remarks>
    public void StopStream()
    {
        shuttingDown = true;
        StopSenderThread();
        SafeDisconnect();
        UnityEngine.Debug.Log("[VarjoTcpStreamSender] Stopped.");
    }

    /// <summary>
    /// Performs one-time initialization of internal timing, image sizing,
    /// and the stereo frame buffer.
    /// </summary>
    /// <remarks>
    /// Allocates and preallocates buffers based on the configured image dimensions.
    /// </remarks>
    private void Initialize()
    {
        if (initialized) return;
        initialized = true;
        frameInterval = 1f / targetFPS;
        UnityEngine.Debug.Log($"[INIT] targetFPS={targetFPS}, frameInterval={frameInterval}");
        imgDim = width * height * numChannels * bytesPerChannel;
        frameBuffer = new StereoFrameBuffer(50);
        frameBuffer.PreallocateImageBuffers(imgDim, imgDim);


    }



    void Update()
    {

        // Update Varjo camera extrinsics (pose) for this frame.
        varjoApiManager.UpdateXtrinsics();
        if (shuttingDown || varjoApiManager == null) return;

        if (!started) return;
        float now = Time.realtimeSinceStartup;
        // Rate limiting: if not enough time has elapsed since the last
        // frame capture/send, skip this Update() cycle.
        if (now - lastSendTime < frameInterval)
        {
            return;
        }
        lastSendTime = now;

        long commonFrameId = DateTime.UtcNow.Ticks; // ← NEW: proper frame counter

        // Drop backlog
        while (sendQueue.Count > maxQueueItems && sendQueue.TryDequeue(out _)) { }

        //get imgL and imgR
        byte[] leftCopy = null;
        byte[] rightCopy = null;
        var rawL = varjoApiManager.leftTexture.GetRawTextureData<byte>();
        var rawR = varjoApiManager.rightTexture.GetRawTextureData<byte>();
        if (rawL != null)
        {
            leftCopy = new byte[rawL.Length];
            rawL.CopyTo(leftCopy);
        }

        if (rawR != null)
        {
            rightCopy = new byte[rawR.Length];
            rawR.CopyTo(rightCopy);
        }
        //get extrinsics
        Matrix4x4 leftExt = varjoApiManager.cameraLeft.Rt;
        Matrix4x4 rightExt = varjoApiManager.cameraRight.Rt;

        // Store the stereo frame in the ring buffer for timestamp-based lookup
        // (e.g., used by CalibrationExample to match detection packets to frames).
        if (leftCopy != null && rightCopy != null)
        {
            lock (frameBufferLock)
            {
                frameBuffer.AddFromCopy(
                    commonFrameId,
                    leftCopy, rightCopy,
                    leftCopy.Length, rightCopy.Length,
                    leftExt, rightExt
                );
            }
        }

        // Enqueue left/right payloads for asynchronous network transmission.
        if (leftCopy != null)
            sendQueue.Enqueue((leftCopy, 0, commonFrameId));

        if (rightCopy != null)
            sendQueue.Enqueue((rightCopy, 1, commonFrameId));



    }



    private void ConnectToServer()
    {
        if (shuttingDown) return;

        try
        {
            SafeDisconnect();

            client = new TcpClient();
            client.NoDelay = true;
            client.SendTimeout = socketTimeoutMs;
            client.ReceiveTimeout = socketTimeoutMs;
            client.Connect(host, port);

            networkStream = client.GetStream();
            networkStream.WriteTimeout = socketTimeoutMs;
            networkStream.ReadTimeout = socketTimeoutMs;

            UnityEngine.Debug.Log($"Connected to Python: {host}:{port}");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError("[VarjoTcpStreamSender] Connect failed: " + e.Message);
            SafeDisconnect();
        }
    }

    private void SafeDisconnect()
    {
        try { networkStream?.Close(); } catch { }
        networkStream = null;

        try { client?.Close(); } catch { }
        client = null;
    }

    private bool EnsureConnected()
    {
        if (shuttingDown) return false;

        if (client != null && networkStream != null && client.Connected && networkStream.CanWrite)
            return true;

        ConnectToServer();
        return (client != null && networkStream != null && client.Connected && networkStream.CanWrite);
    }

    private void StartSenderThread()
    {
        sendCts = new CancellationTokenSource();
        var token = sendCts.Token;

        senderTask = Task.Run(() =>
        {
            while (!token.IsCancellationRequested)
            {
                if (sendQueue.TryDequeue(out var item))
                {
                    if (!TrySendFrame(item.buffer, item.eyeId, item.frameId))
                    {
                        // If send fails, reduce backlog to keep latency bounded
                        while (sendQueue.Count > 2 && sendQueue.TryDequeue(out _)) { }
                        Thread.Sleep(20);
                    }
                }
                else
                {
                    Thread.Sleep(1);
                }
            }
        }, token);
    }


    /// <summary>
    /// Sends a single frame payload (either left or right eye) to the TCP server,
    /// including a fixed-size header followed by raw image bytes.
    /// </summary>
    /// <param name="buffer">
    /// Raw image data buffer to send.
    /// </param>
    /// <param name="eyeId">
    /// Eye identifier where 0 indicates left and 1 indicates right.
    /// </param>
    /// <param name="frameId">
    /// Shared frame identifier used to match left/right images from the same capture time.
    /// </param>
    /// <returns>
    /// <c>true</c> if the payload was written successfully; otherwise <c>false</c>.
    /// </returns>
    /// <remarks>
    /// This method is thread-safe with respect to the network stream by using a lock.
    /// On IO or socket errors, the connection is closed and the method returns false.
    /// </remarks>
    private bool TrySendFrame(byte[] buffer, int eyeId, long frameId)
    {
        long t0 = Stopwatch.GetTimestamp();
        if (shuttingDown) return false;
        if (buffer == null || buffer.Length == 0) return false;
        if (!EnsureConnected()) return false;

        // Use the VarjoApiManager dimensions (single source of truth)
        int w = varjoApiManager.width;
        int h = varjoApiManager.height;
        int ch = varjoApiManager.numChannels;

        int payloadLength = buffer.Length;

        // 32-byte header:
        // int32 w,h,ch,dtype,payloadLen,eyeId + int64 frameId
        byte[] header = new byte[32];
        Buffer.BlockCopy(BitConverter.GetBytes(w), 0, header, 0, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(h), 0, header, 4, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(ch), 0, header, 8, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(dtypeCode), 0, header, 12, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(payloadLength), 0, header, 16, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(eyeId), 0, header, 20, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(frameId), 0, header, 24, 8);

        try
        {

            lock (streamWriteLock)
            {
                if (shuttingDown || networkStream == null || !networkStream.CanWrite) return false;

                networkStream.Write(header, 0, header.Length);
                networkStream.Write(buffer, 0, payloadLength);
            }
            long t1 = Stopwatch.GetTimestamp();
            double sendMs = (t1 - t0) * 1000.0 / Stopwatch.Frequency;

            //UnityEngine.Debug.Log($"[SendTime] eye={eyeId} bytes={payloadLength} sendMs={sendMs:F2}ms");
            return true;
        }
        catch (IOException e)
        {
            UnityEngine.Debug.LogError($"[VarjoTcpStreamSender] Send IO error: {e.Message}");
            SafeDisconnect();
            return false;
        }
        catch (SocketException e)
        {
            UnityEngine.Debug.LogError($"[VarjoTcpStreamSender] Send Socket error ({e.SocketErrorCode}): {e.Message}");
            SafeDisconnect();
            return false;
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"[VarjoTcpStreamSender] Send error: {e.Message}");
            SafeDisconnect();
            return false;
        }
    }


    /// <summary>
    /// Cancels the background sender task and waits briefly for shutdown.
    /// </summary>
    /// <remarks>
    /// The wait timeout is bounded to avoid blocking application quit indefinitely.
    /// </remarks>
    private void StopSenderThread()
    {
        try { sendCts?.Cancel(); } catch { }

        try
        {
            if (senderTask != null && !senderTask.IsCompleted)
                senderTask.Wait(500);
        }
        catch { }

        try { sendCts?.Dispose(); } catch { }
        sendCts = null;
        senderTask = null;
    }

    void OnApplicationQuit()
    {
        StopStream();
    }

    void OnDisable()
    {
        StopStream();
    }


}
