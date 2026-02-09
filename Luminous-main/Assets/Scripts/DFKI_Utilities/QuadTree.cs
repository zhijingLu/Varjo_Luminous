using UnityEngine;
using System.Collections.Generic;
using static DFKI_Utilities.KnotTracker; // for the image blobs output!

namespace DFKI_Utilities
{
    public class QuadTree
    {
        public class Region
        {
            public int x, y, width, height;

            public Region(int _x, int _y, int _width, int _height)
            {
                x = _x;
                y = _y;
                width = _width;
                height = _height;
            }

            public Vector3 GetCenter()
            {
                return new Vector3(x + width / 2.0f, y + height / 2.0f, 1.0f);
            }

            public float GetRadius()
            {
                return Mathf.Min(width, height) / 2.0f;
            }
        };

        // main input
        public Vector3[,] image; // Representation of the current Image as 2D Vector3 array
        private int width;
        private int height;

        // current output values
        private List<ImageBlob> list = new List<ImageBlob>();
        private List<List<System.Tuple<int, float>>> influencedProjBlobs = new List<List<System.Tuple<int, float>>>(); // For each image blob, a list of influenced projBlobs
        private float energy_ii_sum = 0.0f;
        private Region imageRegion;

        // additional input
        private ImageBlob[] projModelBlobs;
        public float colorDiffThreshold = 0.01f;
        public int minRegionSize = 3;
        public float colorDistThreshold = 0.15f;
        public int pixelDistThreshold = 50;

        public QuadTree(int _width, int _height)
        {
            width = _width;
            height = _height;
            imageRegion = new Region(0, 0, width, height);
        }

        public ImageBlob[] Compute(out List<List<System.Tuple<int, float>>> influeces, out float energy, bool debug = true)
        {
            return Compute(imageRegion.x, imageRegion.y, imageRegion.width, imageRegion.height, out influeces, out energy, debug);
        }

        private ImageBlob[] Compute(int x, int y, int w, int h, out List<List<System.Tuple<int, float>>> influences, out float energy, bool debug = true)
        {
            // clean up
            list.Clear();
            influencedProjBlobs.Clear();
            energy_ii_sum = 0.0f;

            // decompose...
            Decompose(new Region(x, y, w, h));

            if (debug)
                Debug.Log("Decompose: found " + list.Count + " blobs");

            // output
            influences = influencedProjBlobs;
            energy = energy_ii_sum;

            return list.ToArray();
        }

        public void UpdateImage(Texture2D texture, bool flip = true)
        {
            // Transform texture to convenient HSV image structure
            // Notice: assuming width and height are correct!
            image = new Vector3[width, height];

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    // Transform the pixel to H,S,V format
                    Color pixel = texture.GetPixel(i, j);
                    Color.RGBToHSV(pixel, out float H, out float S, out float V);

                    if (flip)
                        image[i, height - 1 - j] = new Vector3(H, S, V);
                    else
                        image[i, j] = new Vector3(H, S, V);
                }
            }
        }

        public void UpdateImage(string path, bool flip = true)
        {
            var rawData = System.IO.File.ReadAllBytes(path);
            Texture2D image = new Texture2D(width, height);
            image.LoadImage(rawData);

            UpdateImage(image, flip);
        }

        public void UpdateProjectedModelBlobs(ImageBlob[] blobs, bool updateImageRegion = true)
        {
            projModelBlobs = blobs;

            if (updateImageRegion)
            {
                imageRegion = new Region(width, height, 0, 0);
                int xTo = 0, yTo = 0;

                foreach (ImageBlob blob in blobs)
                {
                    if (blob.position.x - blob.radius < imageRegion.x)
                        imageRegion.x = (int)blob.position.x;
                    if (blob.position.y - blob.radius < imageRegion.y)
                        imageRegion.y = (int)blob.position.y;
                    if (blob.position.x + blob.radius > xTo)
                        xTo = (int)blob.position.x;
                    if (blob.position.y + blob.radius > yTo)
                        yTo = (int)blob.position.y;
                }

                imageRegion.x = Mathf.Max(imageRegion.x - pixelDistThreshold, 0);
                imageRegion.y = Mathf.Max(imageRegion.y - pixelDistThreshold, 0);
                xTo = Mathf.Min(xTo + pixelDistThreshold, width);
                yTo = Mathf.Min(yTo + pixelDistThreshold, height);
                imageRegion.width = xTo - imageRegion.x;
                imageRegion.height = yTo - imageRegion.y;
            }
        }

        private float GetColorSimilarity(Vector3 color1, Vector3 color2)
        {
            float dist = (color1 - color2).sqrMagnitude;

            if (dist < colorDistThreshold)
            {
                float distd = dist / colorDistThreshold;
                return Mathf.Pow(1.0f - distd, 4.0f) * (4.0f * distd + 1); // wendland
            }

            return 0.0f;
        }

        private float ComputeColor(in Region r, out Vector3 color)
        {
            // Compute color average
            color = Vector3.zero;
            for (int i = r.x; i < r.x + r.width; i++)
                for (int j = r.y; j < r.y + r.height; j++)
                    color += image[i, j];
            int size = (r.width * r.height);
            color /= (float)size;

            // Compute color standard deviation
            float colorStdDev = 0.0f;
            for (int i = r.x; i < r.x + r.width; i++)
                for (int j = r.y; j < r.y + r.height; j++)
                    colorStdDev += (image[i, j] - color).sqrMagnitude;
            colorStdDev /= (float)size;

            return colorStdDev;
        }

        private void Decompose(Region region)
        {
            float stdDev = ComputeColor(in region, out Vector3 color);

            if (region.width <= minRegionSize || region.height <= minRegionSize || stdDev < colorDiffThreshold)
            {
                // Check whether we can add this image Blob to the list
                bool add = projModelBlobs.Length == 0;
                List<System.Tuple<int, float>> influences = new List<System.Tuple<int, float>>();

                for (int m = 0; m < projModelBlobs.Length; ++m)
                {
                    float similarity = GetColorSimilarity(projModelBlobs[m].color, color);

                    if (similarity > 0)
                    {
                        influences.Add(new System.Tuple<int, float>(m, similarity));
                        add = true;
                    }
                }

                if (add)
                {
                    // We can add it!
                    ImageBlob blob = new ImageBlob(color, 0);
                    blob.position = region.GetCenter();
                    blob.radius = region.GetRadius();
                    blob.sigma2 = blob.radius * blob.radius;
                    blob.energy = Mathf.PI * blob.sigma2;

                    list.Add(blob);
                    energy_ii_sum += blob.energy;

                    influencedProjBlobs.Add(influences);
                }
            }
            else
            {
                int wh = region.width / 2;
                int hh = region.height / 2;
                Region r0 = new Region(region.x, region.y, wh, hh);
                Region r1 = new Region(region.x + wh, region.y, wh, hh);
                Region r2 = new Region(region.x, region.y + hh, wh, hh);
                Region r3 = new Region(region.x + wh, region.y + hh, wh, hh);

                Decompose(r0);
                Decompose(r1);
                Decompose(r2);
                Decompose(r3);
            }
        }
    }

}