using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using UnityEngine;

public class DetectionReceiver : MonoBehaviour // for a count of 10 object pairs we need 110kb/s bandwidth
{
    [Serializable]
    public struct PairItem
    {
        public int clsid;
        public float lx1, ly1, lx2, ly2;
        public float rx1, ry1, rx2, ry2;
    }

    [Serializable]
    public class PairPacket
    {
        public ulong timestamp;  // uint64 from Python
        public int count;        // int32
        public PairItem[] items; // length == count
    }

    /// <summary>
    /// Variable Intialization
    /// </summary>

    [Header("Listen (Python connects to this)")]
    public string listenIp = "127.0.0.1";
    public int port = 5002;
    public int timeoutMs = 2000;

    private TcpListener listener;
    private TcpClient client;
    private NetworkStream stream;
    private CancellationTokenSource cts;
    private Task worker;

    private readonly ConcurrentQueue<PairPacket> queue = new ConcurrentQueue<PairPacket>();
    public int QueueCount => queue.Count;

    public bool TryDequeue(out PairPacket packet) => queue.TryDequeue(out packet);

    // Python formats: TODO : Change required
    //HEADER_FMT = "<QI"     # uint64 timestamp, uint32 count
    //DATA_FMT = "<iffffffff"  # int32 cls_id + 8 floats (two boxes)
    private const int HEADER_SIZE = 12;
    private const int ITEM_SIZE = 36;

    public bool autoStart = false;



    /// <summary>
    /// Functions for starting and stopping the receiver and reading data
    /// </summary>

    void Start()
    {
        if (autoStart) StartReceiver();
    }

    public void StartReceiver()
    {
        StopReceiver();

        cts = new CancellationTokenSource();

        listener = new TcpListener(IPAddress.Parse(listenIp), port);
        listener.Start();
        Debug.Log($"[DetectionReceiver] Listening on {listenIp}:{port}");

        worker = Task.Run(() => AcceptAndReadLoop(cts.Token), cts.Token);
    }

    public void StopReceiver()
    {
        try { cts?.Cancel(); } catch { }
        ResetClient();
        try { listener?.Stop(); } catch { }

        listener = null;

        try { worker?.Wait(100); } catch { }
        worker = null;

        cts?.Dispose();
        cts = null;

        while (queue.TryDequeue(out _)) { }
    }

    private async Task AcceptAndReadLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                if (client == null || !client.Connected)
                {
                    client = await listener.AcceptTcpClientAsync();
                    client.NoDelay = true;
                    client.ReceiveTimeout = timeoutMs;
                    client.SendTimeout = timeoutMs;

                    stream = client.GetStream();
                    stream.ReadTimeout = timeoutMs;

                    Debug.Log($"[DetectionReceiver] Accepted {client.Client.RemoteEndPoint}");
                }

                // 1) header
                byte[] header = ReadExact(stream, HEADER_SIZE, token);
                if (header == null) { ResetClient(); continue; }

                ulong ts = ReadUInt64LE(header, 0);
                int count = ReadInt32LE(header, 8);
                if (count < 0 || count > 1_000_000)
                    throw new Exception($"Invalid count: {count}");

                // 2) items
                int itemsBytes = checked(count * ITEM_SIZE);
                byte[] blob = itemsBytes > 0 ? ReadExact(stream, itemsBytes, token) : Array.Empty<byte>();
                if (blob == null) { ResetClient(); continue; }

                var pkt = new PairPacket
                {
                    timestamp = ts,
                    count = count,
                    items = new PairItem[count]
                };

                int off = 0;
                for (int i = 0; i < count; i++)
                {
                    PairItem it = new PairItem();

                    it.clsid = ReadInt32LE(blob, off); off += 4;

                    it.lx1 = ReadFloatLE(blob, off); off += 4;
                    it.ly1 = ReadFloatLE(blob, off); off += 4;
                    it.lx2 = ReadFloatLE(blob, off); off += 4;
                    it.ly2 = ReadFloatLE(blob, off); off += 4;

                    it.rx1 = ReadFloatLE(blob, off); off += 4;
                    it.ry1 = ReadFloatLE(blob, off); off += 4;
                    it.rx2 = ReadFloatLE(blob, off); off += 4;
                    it.ry2 = ReadFloatLE(blob, off); off += 4;

                    pkt.items[i] = it;
                }

                queue.Enqueue(pkt);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[DetectionReceiver] Receive error: {e.Message}");
                ResetClient();
                await Task.Delay(200, token).ContinueWith(_ => { });
            }
        }
    }

    private void ResetClient()
    {
        try { stream?.Close(); } catch { }
        try { client?.Close(); } catch { }
        stream = null;
        client = null;
    }

    private static byte[] ReadExact(NetworkStream s, int size, CancellationToken token)
    {
        byte[] buf = new byte[size];
        int offset = 0;
        while (offset < size && !token.IsCancellationRequested)
        {
            int read = s.Read(buf, offset, size - offset);
            if (read <= 0) return null;
            offset += read;
        }
        return token.IsCancellationRequested ? null : buf;
    }

    // Little-endian decode helpers
    private static int ReadInt32LE(byte[] b, int o)
    {
        unchecked { return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24); }
    }

    private static uint ReadUInt32LE(byte[] b, int o)
    {
        unchecked { return (uint)(b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)); }
    }

    private static ulong ReadUInt64LE(byte[] b, int o)
    {
        ulong lo = ReadUInt32LE(b, o);
        ulong hi = ReadUInt32LE(b, o + 4);
        return lo | (hi << 32);
    }

    private static float ReadFloatLE(byte[] b, int o)
    {
        uint u = ReadUInt32LE(b, o);
        return BitConverter.Int32BitsToSingle((int)u);
    }

    void OnApplicationQuit()
    {
        StopReceiver();
    }


}
