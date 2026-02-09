using UnityEngine;
using static DFKI_Utilities.KnotTracker;
using System.IO;
using System;

namespace DFKI_Utilities
{
    public class Writer
    {
        public class Settings
        {
            public int size;
            public Vector3[] positions;
            public Color[] colors;
            public float[] radiuses;

            public Settings(int _size)
            {
                size = _size;
                positions = new Vector3[size];
                colors = new Color[size];
                radiuses = new float[size];

                // default values
                for (int i = 0; i < size; i++)
                {
                    positions[i] = Vector3.zero;
                    colors[i] = Color.white;
                    radiuses[i] = 3.0f;
                }
            }

            public void SetPosition(Vector3 _position)
            {
                if (size != 1)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", 1, size), "_positions");

                positions[0] = _position;
            }

            public void SetPosition(Vector3[] _positions)
            {
                if (_positions.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", _positions.Length, size), "_positions");

                positions = _positions;
            }

            public void SetPosition(Vector2[] _positions)
            {
                if (_positions.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", _positions.Length, size), "_positions");

                for (int i = 0; i < size; i++)
                    positions[i] = new Vector3(_positions[i].x, _positions[i].y, 1.0f);
            }

            public void SetPosition(Vector3 _position, CameraView camera)
            {
                // Notice: when camera view is included as parameters, it counts as if the position is given in 3D

                if (size != 1)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", 1, size), "_positions");

                positions[0] = camera.GetPixelCoordinates(_position);
            }

            public void SetPosition(Vector3[] _positions, CameraView camera)
            {
                // Notice: when camera view is included as parameters, it counts as if the position is given in 3D

                if (_positions.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", _positions.Length, size), "_positions");

                for (int i = 0; i < size; ++i)
                    positions[i] = camera.GetPixelCoordinates(_positions[i]);
            }

            public void SetPosition(ImageBlob[] blobs)
            {
                if (blobs.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", blobs.Length, size), "blobs");

                for (int i = 0; i < size; ++i)
                    positions[i] = blobs[i].position;
            }

            public void SetPosition(ImageBlob blob)
            {
                if (size != 1)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", 1, size), "blob");

                positions[0] = blob.position;
            }

            public void SetPosition(ModelBlob blob, CameraView camera)
            {
                if (size != 1)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", 1, size), "blob");

                positions[0] = camera.GetPixelCoordinates(blob.position);
            }

            public void SetPosition(ModelBlob[] blobs, CameraView camera)
            {
                if (blobs.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", blobs.Length, size), "blobs");

                for (int i = 0; i < size; ++i)
                    positions[i] = camera.GetPixelCoordinates(blobs[i].position);
            }

            public void SetRadius(float _radius)
            {
                for (int i = 0; i < size; ++i)
                    radiuses[i] = _radius;
            }

            public void SetRadius(float _radius, CameraView camera)
            {
                // Notice: when camera view is included as parameters, it counts as if the radius is given in 3D
                // Notice: first input the positions!

                for (int i = 0; i < size; ++i)
                    radiuses[i] = camera.GetRadiusPixel(_radius, positions[i].z);
            }

            public void SetRadius(float[] _radiuses)
            {
                if (_radiuses.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", _radiuses.Length, size), "_radiuses");

                radiuses = _radiuses;
            }

            public void SetRadius(float[] _radiuses, CameraView camera)
            {
                // Notice: when camera view is included as parameters, it counts as if the radius is given in 3D
                // Notice: first input the positions!

                if (_radiuses.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", _radiuses.Length, size), "_radiuses");


                for (int i = 0; i < size; ++i)
                    radiuses[i] = camera.GetRadiusPixel(_radiuses[i], positions[i].z);
            }

            public void SetRadius(ImageBlob blob)
            {
                for (int i = 0; i < size; ++i)
                    radiuses[i] = blob.radius;
            }

            public void SetRadius(ImageBlob[] blobs)
            {
                if (blobs.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", blobs.Length, size), "blobs");

                for (int i = 0; i < size; ++i)
                    radiuses[i] = blobs[i].radius;
            }

            public void SetRadius(ModelBlob blob, CameraView camera)
            {
                for (int i = 0; i < size; ++i)
                    radiuses[i] = camera.GetRadiusPixel(blob.radius, positions[i].z);
            }

            public void SetRadius(ModelBlob[] blobs, CameraView camera)
            {
                if (blobs.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", blobs.Length, size), "blobs");

                for (int i = 0; i < size; ++i)
                    radiuses[i] = camera.GetRadiusPixel(blobs[i].radius, positions[i].z);
            }

            public void SetColor(Color _color, bool isHSV = true)
            {
                for (int i = 0; i < size; ++i)
                {
                    if (isHSV)
                        colors[i] = Color.HSVToRGB(_color.r, _color.g, _color.b);
                    else
                        colors[i] = _color;
                }
            }

            public void SetColor(Color[] _colors, bool isHSV = true)
            {
                if (_colors.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", _colors.Length, size), "_colors");

                if (!isHSV)
                {
                    colors = _colors;
                }
                else
                {
                    for (int i = 0; i < size; ++i)
                        colors[i] = Color.HSVToRGB(_colors[i].r, _colors[i].g, _colors[i].b);
                }
            }

            public void SetColor(Vector3 _color, bool isHSV = true)
            {
                for (int i = 0; i < size; ++i)
                {
                    if (isHSV)
                        colors[i] = Color.HSVToRGB(_color.x, _color.y, _color.z);
                    else
                        colors[i] = new Color(_color.x, _color.y, _color.z);
                }
            }

            public void SetColor(Vector3[] _colors, bool isHSV = true)
            {
                if (_colors.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", _colors.Length, size), "_colors");

                for (int i = 0; i < size; ++i)
                {
                    if (isHSV)
                        colors[i] = Color.HSVToRGB(_colors[i].x, _colors[i].y, _colors[i].z);
                    else
                        colors[i] = new Color(_colors[i].x, _colors[i].y, _colors[i].z);
                }
            }

            public void SetColor(ImageBlob blob)
            {
                for (int i = 0; i < size; ++i)
                    colors[i] = Color.HSVToRGB(blob.color.x, blob.color.y, blob.color.z); // Notice: it is assumed to be in the HSV format for image blobs!
            }

            public void SetColor(ImageBlob[] blobs)
            {
                if (blobs.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", blobs.Length, size), "blobs");

                for (int i = 0; i < size; ++i)
                    colors[i] = Color.HSVToRGB(blobs[i].color.x, blobs[i].color.y, blobs[i].color.z); // Notice: it is assumed to be in the HSV format for image blobs!
            }

            public void SetColor(ModelBlob blob)
            {
                for (int i = 0; i < size; ++i)
                    colors[i] = Color.HSVToRGB(blob.color.x, blob.color.y, blob.color.z); // Notice: it is assumed to be in the HSV format for image blobs!
            }

            public void SetColor(ModelBlob[] blobs)
            {
                if (blobs.Length != size)
                    throw new ArgumentException(String.Format("Error: the provided vector size {0} does not match the parameter's size {1}", blobs.Length, size), "blobs");

                for (int i = 0; i < size; ++i)
                    colors[i] = Color.HSVToRGB(blobs[i].color.x, blobs[i].color.y, blobs[i].color.z); // Notice: it is assumed to be in the HSV format for image blobs!
            }
        };

        private static void DrawCircle(ref Texture2D tex, Color color, int x, int y, int radius = 3)
        {
            // Draw only if the boundaries are respected
            if (x >= 0 && x + radius < tex.width - 1 && x - radius >= 0 &&
                y >= 0 && y + radius < tex.height - 1 && y - radius >= 0 &&
                radius >= 0)
            {
                float rSquared = radius * radius;

                for (int u = x - radius; u < x + radius + 1; u++)
                    for (int v = y - radius; v < y + radius + 1; v++)
                        if ((x - u) * (x - u) + (y - v) * (y - v) < rSquared)
                            tex.SetPixel(u, v, color);
                tex.Apply();
            }
        }

        private static void DrawSquare(ref Texture2D tex, Color color, int x, int y, int width, int height)
        {
            // Draw only if the boundaries are respected
            if (x >= 0 && x < tex.width && x + width < tex.width &&
                y >= 0 && y < tex.height && y + height < tex.height &&
                width >= 0 && height >= 0)
            {
                Color[] colors = new Color[width * height];
                Array.Fill(colors, color);

                tex.SetPixels(x, y, width, height, colors);
                tex.Apply();
            }
        }

        public static Texture2D CopyAndFlip(Texture2D image, bool flip = true)
        {
            Texture2D copy = new Texture2D(image.width, image.height);
            copy.SetPixels(image.GetPixels());
            copy.Apply();
            
            if (flip)
                FlipImage(ref copy);

            return copy;
        }
        public static void FlipImage(ref Texture2D tex)
        {
            Color[] pixels = tex.GetPixels();
            Color[] pixelsFlipped = new Color[pixels.Length];

            for (int i = 0; i < tex.height; i++)
                Array.Copy(pixels, i * tex.width, pixelsFlipped, (tex.height - i - 1) * tex.width, tex.width);

            tex.SetPixels(pixelsFlipped);
            tex.Apply();
        }
        
        public static void SaveSquares(Settings p, Texture2D image, string path, bool copyTexture = true, bool flip = true)
        {
            Texture2D output = PrintTextureSquares(p, image, copyTexture, flip);

            byte[] bytes = output.EncodeToPNG();
            File.WriteAllBytes(path, bytes);
        }

        public static void SaveCircles(Settings p, Texture2D image, string path, bool copyTexture = true, bool flip = true)
        {
            Texture2D output = PrintTextureCircles(p, image, copyTexture, flip);

            byte[] bytes = output.EncodeToPNG();
            File.WriteAllBytes(path, bytes);
        }

        public static Texture2D PrintTextureSquares(Settings p, Texture2D image, bool copyTexture = true, bool flip = true)
        {
            Texture2D copy = image;
            if (copyTexture)
            {
                copy = new Texture2D(image.width, image.height);
                copy.SetPixels(image.GetPixels());
                copy.Apply();
            }
            if (flip)
                FlipImage(ref copy);

            for (int e = 0; e < p.size; e++)
            {
                int x = (int)Mathf.Floor(p.positions[e].x);
                int y = (int)Mathf.Ceil(p.positions[e].y);
                int r = (int)Mathf.Ceil(p.radiuses[e]);

                DrawSquare(ref copy, p.colors[e], x - r, y - r, 2 * r, 2 * r);
            }
            copy.Apply();

            return copy;
        }

        public static Texture2D PrintTextureCircles(Settings p, Texture2D image, bool copyTexture = true, bool flip = true)
        {
            Texture2D copy = image;
            if (copyTexture)
            {
                copy = new Texture2D(image.width, image.height);
                copy.SetPixels(image.GetPixels());
                copy.Apply();
            }
            if (flip)
                FlipImage(ref copy);

            for (int e = 0; e < p.size; e++)
            {
                Vector3 coord = p.positions[e];
                float r = p.radiuses[e];
                Color color = p.colors[e];

                DrawCircle(ref copy, color, (int)coord.x, (int)coord.y, (int)Mathf.Ceil(r));
            }
            copy.Apply();

            return copy;
        }

        public static string Print(ref float[] array)
        {
            string text = "[";
            for (var i = 0; i < array.Length - 1; i++)
                text += array[i] + ",";
            if (array.Length > 0)
                text += array[array.Length - 1] + "]";

            return text;
        }

        public static string Print(ref bool[] array)
        {
            string text = "[";
            for (var i = 0; i < array.Length - 1; i++)
            {
                if (array[i])
                    text += "T,";
                else
                    text += "F,";
            }
            if (array.Length > 0)
                text += array[array.Length - 1] + "]";

            return text;
        }

        public static int GetNumChannels(ref Texture2D image)
        {
            switch (image.format)
            {
                case TextureFormat.RGB24:
                case TextureFormat.RGB48:
                    return 3;
                case TextureFormat.RGBA32:
                case TextureFormat.BGRA32:
                case TextureFormat.RGBA64:
                case TextureFormat.RGBAFloat:
                    return 4;
                default:
                    Debug.LogError("Error: texture format not supported");
                    return 0;
            }
        }

        public static int GetBitsPerChannel(ref Texture2D image)
        {
            switch (image.format)
            {
                case TextureFormat.RGB24:
                case TextureFormat.RGBA32:
                case TextureFormat.BGRA32:
                    return 8;
                case TextureFormat.RGB48:
                case TextureFormat.RGBA64:
                    return 16;
                case TextureFormat.RGBAFloat:
                    return 32;
                default:
                    Debug.LogError("Error: texture format not supported");
                    return 0;
            }
        }

    }

}