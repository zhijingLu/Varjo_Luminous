using UnityEngine;

namespace DFKI_Utilities
{
    public class CameraView
    {
        // camera name
        public string name;

        // camera matrices and image details
        public Matrix4x4 Rt = Matrix4x4.identity; // extrinsics
        public Matrix4x4 K = Matrix4x4.identity;  // intrinsics
        public Matrix4x4 P = Matrix4x4.identity;  // projection = K * Rt
        public float focalLength = 1.0f;
        public int width = 1152;
        public int height = 1152;

        public CameraView(string _name, int _width, int _height)
        {
            name = _name;
            width = _width;
            height = _height;
        }

        public bool SetXTrinsics(Matrix4x4 _Rt, Matrix4x4 _K, Matrix4x4 _P)
        {
            Rt = _Rt;
            K = _K;
            P = _P;
            focalLength = _K[1, 1]; // fy

            return true;
        }

        public bool SetXTrinsics(double[] intr, double[] extr)
        {
            // This function assumes the the provided parameters are in the VarjoLib format, i.e. as they are obtained from the DLL

            Matrix4x4 _Rt = Matrix4x4.identity, _K = Matrix4x4.identity, _P = Matrix4x4.identity;

            _Rt.SetRow(0, new Vector4((float)extr[0], (float)extr[4], (float)extr[8], (float)extr[12]));
            _Rt.SetRow(1, new Vector4((float)extr[1], (float)extr[5], (float)extr[9], (float)extr[13]));
            _Rt.SetRow(2, new Vector4((float)extr[2], (float)extr[6], (float)extr[10], (float)extr[14]));
            _Rt.SetRow(3, new Vector4((float)extr[3], (float)extr[7], (float)extr[11], (float)extr[15]));

            _K.SetRow(0, new Vector4((float)intr[2] * width, 0.0f, (float)intr[0] * width, 0.0f));
            _K.SetRow(1, new Vector4(0.0f, (float)intr[3] * height, (float)intr[1] * height, 0.0f));

            _P = _K * _Rt;

            return SetXTrinsics(_Rt, _K, _P);
        }

        public Vector3 GetForward()
        {
            return Rt.MultiplyPoint3x4(Vector3.forward);
        }

        public Vector3 GetCenter()
        {
            return Rt.MultiplyPoint3x4(Vector3.zero);
        }

        public Vector3 GetPixelCoordinates(Vector3 point)
        {
            var result = P.MultiplyPoint3x4(point);
            return new Vector3(result.x / result.z, result.y / result.z, result.z);
        }

        public float GetRadiusPixel(float radius3D, float coords_z)
        {
            return (radius3D * focalLength) / coords_z;
        }
    }

}