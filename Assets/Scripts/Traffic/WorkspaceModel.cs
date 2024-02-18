using UnityEngine;

namespace vo
{
    public class VelocityObstacle
    {
        public Vector3 transl_vB_vA;
        public Vector3 bound_left;
        public Vector3 bound_right;
        public float dist_BA;
        public float rad;

        public VelocityObstacle(Vector3 transl_vB_vA, Vector3 bound_left, Vector3 bound_right, float dist_BA, float rad)
        {
            this.transl_vB_vA = transl_vB_vA;
            this.bound_left = bound_left;
            this.bound_right = bound_right;
            this.dist_BA = dist_BA;
            this.rad = rad;
        }
    }

    public class Agent
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 DesiredVelocity;
        public float Radius;

        public Agent(Vector3 position, Vector3 velocity, Vector3 desiredVelocity, float radius)
        {
            Position = position;
            Velocity = velocity;
            DesiredVelocity = desiredVelocity;
            Radius = radius;
        }

        public void Update(Agent toCopy) {
            Position = toCopy.Position;
            Velocity = toCopy.Velocity;
            DesiredVelocity = toCopy.DesiredVelocity;
            Radius = toCopy.Radius;
        }
    }
}