using UnityEngine;

namespace vo
{
    public class VelocityObstacle
    {
        public Agent agentA;
        public Agent agentB;
        public Vector2 apex; // Vo offset in velocity space
        public Vector2 bound_left;
        public Vector2 bound_right;
        public float dist_BA;
        public float rad;

        public VelocityObstacle(Agent agentA, Agent agentB, Vector2 vo_apex, Vector2 bound_left, Vector2 bound_right, float dist_BA, float rad)
        {
            this.agentA = agentA;
            this.agentB = agentB;
            this.apex = vo_apex;
            this.bound_left = bound_left;
            this.bound_right = bound_right;
            this.dist_BA = dist_BA;
            this.rad = rad;
        }
    }

    public class Agent
    {
        public Vector2 Position;
        public Vector2 Velocity;
        public Vector2 DesiredVelocity;
        public float Radius;

        public Agent(Vector2 position, Vector2 velocity, Vector2 desiredVelocity, float radius)
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