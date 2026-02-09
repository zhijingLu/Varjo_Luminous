using UnityEngine;
using System;
namespace DFKI_Utilities
{
    public class StereoFrameBuffer
    {
        private readonly StereoFrameData[] buffer;
        private int writeIndex = 0;
        private int count = 0;

        public int Capacity { get; }
        public int Count => count;

        public StereoFrameBuffer(int capacity = 50)
        {
            if (capacity <= 0) throw new ArgumentOutOfRangeException(nameof(capacity));

            Capacity = capacity;
            buffer = new StereoFrameData[capacity];

            // Pre-allocate objects once to avoid per-frame allocations
            for (int i = 0; i < capacity; i++)
                buffer[i] = new StereoFrameData();
        }

        /// <summary>
        /// Returns the next slot to write into (oldest slot when buffer is full).
        /// Fill its fields, then call CommitWriteSlot().
        /// </summary>
        public StereoFrameData AcquireWriteSlot()
        {
            return buffer[writeIndex];
        }

        /// <summary>
        /// Advances the ring after you have written into the acquired slot.
        /// </summary>
        public void CommitWriteSlot()
        {
            writeIndex = (writeIndex + 1) % Capacity;
            if (count < Capacity) count++;
        }


        /// <summary>
        /// Convenience method if you still want a single call.
        /// Note: does not allocate, just copies references into the slot.
        /// </summary>
        public void AddFromCopy(
                 long timestamp,
                 byte[] srcLeft,
                 byte[] srcRight,
                 int bytesToCopyLeft,
                 int bytesToCopyRight,
                 Matrix4x4 extrinsicsLeft,
                 Matrix4x4 extrinsicsRight)
        {
            if (srcLeft == null) throw new ArgumentNullException(nameof(srcLeft));
            if (srcRight == null) throw new ArgumentNullException(nameof(srcRight));
            if (bytesToCopyLeft <= 0 || bytesToCopyLeft > srcLeft.Length)
                throw new ArgumentOutOfRangeException(nameof(bytesToCopyLeft));
            if (bytesToCopyRight <= 0 || bytesToCopyRight > srcRight.Length)
                throw new ArgumentOutOfRangeException(nameof(bytesToCopyRight));

            var slot = AcquireWriteSlot();

            slot.timestamp = timestamp;
            slot.extrinsicsLeft = extrinsicsLeft;
            slot.extrinsicsRight = extrinsicsRight;

            // Ensure destination buffers exist and are large enough
            if (slot.leftImg == null || slot.leftImg.Length < bytesToCopyLeft)
                slot.leftImg = new byte[bytesToCopyLeft];

            if (slot.rightImg == null || slot.rightImg.Length < bytesToCopyRight)
                slot.rightImg = new byte[bytesToCopyRight];

            Buffer.BlockCopy(srcLeft, 0, slot.leftImg, 0, bytesToCopyLeft);
            Buffer.BlockCopy(srcRight, 0, slot.rightImg, 0, bytesToCopyRight);

            CommitWriteSlot();
        }


        // Get item by index relative to newest frame (0 = most recent)
        public StereoFrameData GetLatest(int offset = 0)
        {
            if (count == 0) return null;
            if (offset < 0 || offset >= count) return null;

            int index = (writeIndex - 1 - offset + Capacity) % Capacity;
            return buffer[index];
        }

        // Find by timestamp (linear scan)
        private int LogicalToPhysicalIndex(int logicalIndex)
        {
            int oldestPhysical = (writeIndex - count + Capacity) % Capacity;
            return (oldestPhysical + logicalIndex) % Capacity;
        }

        public StereoFrameData FindClosest(long timestamp)
        {
            if (count == 0)
                return null;

            int left = 0;
            int right = count - 1;

            while (left <= right)
            {
                int mid = (left + right) / 2;
                int midPhysical = LogicalToPhysicalIndex(mid);

                var item = buffer[midPhysical];
                if (item == null)
                {
                    right = mid - 1;
                    continue;
                }

                long midTs = item.timestamp;

                if (midTs == timestamp)
                    return item;

                if (midTs < timestamp)
                    left = mid + 1;
                else
                    right = mid - 1;
            }

            // If timestamp is earlier than all frames → oldest
            if (right < 0)
                return buffer[LogicalToPhysicalIndex(0)];

            // If timestamp is newer than all frames → newest
            if (left >= count)
                return buffer[LogicalToPhysicalIndex(count - 1)];

            // Compare left vs right neighbors
            var leftItem = buffer[LogicalToPhysicalIndex(left)];
            var rightItem = buffer[LogicalToPhysicalIndex(right)];

            if (leftItem == null) return rightItem;
            if (rightItem == null) return leftItem;

            return Math.Abs(leftItem.timestamp - timestamp)
                < Math.Abs(rightItem.timestamp - timestamp)
                ? leftItem
                : rightItem;
        }

        /// <summary>
        /// Optional: preallocate fixed-size image buffers per slot.
        /// Call once when you know the byte size per image.
        /// </summary>
        public void PreallocateImageBuffers(int leftSizeBytes, int rightSizeBytes)
        {
            if (leftSizeBytes < 0) throw new ArgumentOutOfRangeException(nameof(leftSizeBytes));
            if (rightSizeBytes < 0) throw new ArgumentOutOfRangeException(nameof(rightSizeBytes));

            for (int i = 0; i < Capacity; i++)
            {
                buffer[i].leftImg = leftSizeBytes == 0 ? Array.Empty<byte>() : new byte[leftSizeBytes];
                buffer[i].rightImg = rightSizeBytes == 0 ? Array.Empty<byte>() : new byte[rightSizeBytes];
            }
        }


    }


    public class StereoFrameData
    {
        public long timestamp;

        public byte[] leftImg;
        public byte[] rightImg;
    
        public Matrix4x4 extrinsicsLeft  = Matrix4x4.identity;
        public Matrix4x4 extrinsicsRight = Matrix4x4.identity;
    }

}