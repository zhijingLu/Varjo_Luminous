using UnityEngine;

namespace DFKI_Utilities
{
    public class UnityUtils
    {
        public static GameObject CreateNewGameObject(string name, Vector3 position, UnityEngine.Color color, bool useLocalPosition = false, GameObject parent = null, float scale = 0.01f)
        {
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obj.name = name;

            if (parent != null)
                obj.transform.parent = parent.transform;

            if (useLocalPosition)
                obj.transform.localPosition = position;
            else
                obj.transform.position = position;

            obj.transform.localScale = new Vector3(scale, scale, scale);
            var renderer = obj.GetComponent<Renderer>();
            renderer.material.SetColor("_Color", color);

            return obj;
        }

        public static float[] toFloatArray(ref Vector2[] values, bool flip = false)
        {
            float[] result = new float[values.Length * 2];

            for (int i = 0; i < values.Length; i++)
            {
                result[i * 2 + 0] = values[i].x;
                if (flip)
                    result[i * 2 + 1] = 1152 - values[i].y;
                else
                    result[i * 2 + 1] = values[i].y;
            }

            return result;
        }

        public static float[] toFloatArray(Vector2 value, bool flip = false)
        {
            float[] result = new float[2];

            result[0] = value.x;
            if (flip)
                result[1] = 1152 - value.y;
            else
                result[1] = value.y;

            return result;
        }

        public static Vector3[] toVectorArray(ref float[] points3D)
        {
            int size = points3D.Length / 3;
            Vector3[] result = new Vector3[size];

            for (int i = 0; i < size; i++)
            {
                result[i].x = points3D[i * 3 + 0];
                result[i].y = points3D[i * 3 + 1];
                result[i].z = points3D[i * 3 + 2];
            }

            return result;
        }

        public static float[] toFloatArray3x4(ref Matrix4x4 values)
        {
            float[] result = new float[3 * 4];

            for (int row = 0; row < 3; row++)
                for (int col = 0; col < 4; col++)
                    result[row * 3 + col] = values[row, col];

            return result;
        }

        public static Vector3 toHomogeneous(Vector2 point)
        {
            return new Vector3(point.x, point.y, 1.0f);
        }

        public static Vector2 fromHomogeneous(Vector3 point)
        {
            return new Vector2(point.x, point.y);
        }

        public static Vector3 RotateVector(Vector3 vector, Vector3 axis, float angle)
        {
            Vector3 vxp = Vector3.Cross(axis, vector);
            Vector3 vxvxp = Vector3.Cross(axis, vxp);
            return vector + Mathf.Sin(angle) * vxp + (1 - Mathf.Cos(angle)) * vxvxp;
        }

        public static Vector3 RotatePointAboutAxisWithPivot(Vector3 point, Vector3 pivot, Vector3 axis, float angle)
        {
            return pivot + RotateVector(point - pivot, axis, angle);
        }

    }

}