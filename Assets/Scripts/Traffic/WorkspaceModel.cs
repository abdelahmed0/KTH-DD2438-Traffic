using UnityEngine;

namespace avoidance
{
    public class VelocityObstacle
    {
        public Agent agentA;
        public Agent agentB;
        public Vector2 apex; // Vo offset in velocity space
        public Vector2 boundLeft;
        public Vector2 boundRight;
        public float dist_BA;
        public float combinedRadius;

        public VelocityObstacle(Agent agentA, Agent agentB, Vector2 apex, Vector2 bound_left, Vector2 bound_right, float dist_BA, float rad)
        {
            this.agentA = agentA;
            this.agentB = agentB;
            this.apex = apex;
            this.boundLeft = bound_left;
            this.boundRight = bound_right;
            this.dist_BA = dist_BA;
            this.combinedRadius = rad;
        }

        public bool VelocityRightOfCenterLine(Vector2 velocity)
        {
            Vector2 centerlineDir = (boundLeft + boundRight).normalized;
            Vector2 velocityToVoSpace = velocity - apex;
            return Vector2.SignedAngle(centerlineDir, velocityToVoSpace) < 0f;
        }

        public float CollisionTimeFromVelocity(Vector2 velocity)
        {
            float timeToCollision;

            if (velocity.magnitude == 0f)
                timeToCollision = float.MaxValue;
            else if (dist_BA - combinedRadius < 0f)
                timeToCollision = 0.00001f;
            else
                timeToCollision = (dist_BA - combinedRadius) / velocity.magnitude;
            
            return timeToCollision;
        }

        public bool ContainsVelocity(Vector2 velocity)
        {
            Vector2 dirVelVOSpace = (velocity - apex).normalized;

            Vector2 dirToLeft = boundLeft.normalized - dirVelVOSpace; 
            Vector2 dirToRight = boundRight.normalized - dirVelVOSpace; 

            return Vector2.Dot(dirToLeft, dirToRight) < 0f;
        }
    }

    public class Agent
    {
        public Vector2 Position;
        public Vector2 Velocity;
        public Vector2 DesiredVelocity;
        public float Radius;
        public float aggresiveness = 1f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less

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