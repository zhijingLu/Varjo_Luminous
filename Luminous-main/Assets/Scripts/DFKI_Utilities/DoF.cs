using UnityEngine;

namespace DFKI_Utilities
{
    public abstract class Dof
    {
        public Vector3 center;
        public Vector3 offset; // local coordinates from the joint parent
        public Vector3 axis;
        public Vector3 axis_init;
        public float value;
        public bool active = true;

        public abstract Matrix4x4 GetLocalTransform();
        public abstract Vector3 GetDerivative(Vector3 point);
    }

    public class Revolute : Dof
    {
        public override Vector3 GetDerivative(Vector3 point)
        {
            if (!active)
                return Vector3.zero;

            return Vector3.Cross(axis, point - center);
        }

        public override Matrix4x4 GetLocalTransform()
        {
            return Matrix4x4.Rotate(Quaternion.AngleAxis(value, axis_init));
        }
    }

    public class Prismatic : Dof
    {
        public override Vector3 GetDerivative(Vector3 point)
        {
            if (!active)
                return Vector3.zero;

            return axis;
        }

        public override Matrix4x4 GetLocalTransform()
        {
            return Matrix4x4.Translate(value * axis_init);
        }
    }

    public class Static : Dof
    {
        public override Vector3 GetDerivative(Vector3 point)
        {
            return Vector3.zero;
        }

        public override Matrix4x4 GetLocalTransform()
        {
            return Matrix4x4.identity;
        }
    }

}