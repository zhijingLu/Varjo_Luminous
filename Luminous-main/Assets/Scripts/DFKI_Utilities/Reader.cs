using UnityEngine;
using System.Collections.Generic;
using System.IO;

namespace DFKI_Utilities
{
    public class Reader
    {
        private int currentId = 0;

        private string directory;
        private string[] patterns;

        private List<string[]> files;
        public int skip = 0; // frames to skip every time Next() is called
        public int start = 0; // frame to start with

        private int N; // number of views
        private int F; // number of files per view

        public Reader(string _directory, string[] _patterns, int _start = 0, int _skip = 0)
        {
            N = _patterns.Length;

            if (N <= 0)
                throw new System.ArgumentException("No valid pattern provided", "_patterns");

            directory = _directory;
            patterns = new string[N];
            System.Array.Copy(_patterns, patterns, N);

            currentId = _start;
            skip = _skip;
            start = _start;

            files = new List<string[]>();
            F = 0;
            for (var i = 0; i < N; i++)
            {
                files.Add(Directory.GetFiles(directory, patterns[i]));
                F = Mathf.Max(F, files[i].Length);
            }

            // Important: do consistency checks
            if (F <= 0)
                throw new System.ArgumentException("The provided directory and patterns resulted in no files found", "_patterns");

            for (var i = 0; i < N; i++)
            {
                if (files[i].Length != F)
                    throw new System.ArgumentException("The provided directory and patterns produce an inconsistent number of files", "_patterns");
            }

        }

        private static byte[] ReadAllBytes(BinaryReader reader)
        {
            const int bufferSize = 4096;
            using (var ms = new MemoryStream())
            {
                byte[] buffer = new byte[bufferSize];
                int count;
                while ((count = reader.Read(buffer, 0, buffer.Length)) != 0)
                    ms.Write(buffer, 0, count);
                return ms.ToArray();
            }

        }

        public string[] Next()
        {
            currentId = Mathf.Min(currentId + skip + 1, F); // Note: out of bounds (F, should be F - 1) on purpose
            return GetPaths();
        }

        public string[] Previous()
        {
            currentId = Mathf.Max(-1, currentId - skip - 1);  // Note: out of bounds (-1) on purpose
            return GetPaths();
        }

        public string[] Current()
        {
            return GetPaths();
        }

        private string[] GetPaths()
        {
            if (0 <= currentId && currentId < F)
            {
                var paths = new string[N];
                for (var i = 0; i < N; i++)
                    paths[i] = files[i][currentId];

                return paths;
            }

            return null; // nothing more to read
        }

        public void Reset()
        {
            currentId = start;
        }

        public int GetCurrentId()
        {
            return currentId;
        }

        public static void ReadBinary(string path, ref Texture2D image, int channels = 4) // 4 bytes for RGBA
        {
            BinaryReader myFile = new BinaryReader(File.Open(path, FileMode.Open));
            // TODO: check that path exists!
            var arr = ReadAllBytes(myFile);
            myFile.Close();

            if (image.width * image.height * channels != arr.Length)
                throw new System.ArgumentException("The image provided has the wrong size!", "image");

            for (int i = 0, ai = 0; i < image.width; i++)
            {
                for (int j = 0; j < image.height; j++)
                {
                    image.SetPixel(1152 - j, 1152 - i, new Color(arr[ai] / 255.0f, arr[ai + 1] / 255.0f, arr[ai + 2] / 255.0f, arr[ai + 3] / 255.0f));
                    ai += 4;
                }
            }
        }

        public static void ReadImage(string path, ref Texture2D image)
        {
            var rawData = System.IO.File.ReadAllBytes(path);
            image.LoadImage(rawData);
        }
    }
}