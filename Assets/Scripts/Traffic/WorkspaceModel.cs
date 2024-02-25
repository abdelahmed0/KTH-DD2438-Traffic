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
            Vector2 velocityInVoSpace = velocity - apex;
            float theta = Vector2.Angle(Vector2.right, velocityInVoSpace);

            float thetaRight = Vector2.Angle(boundRight, Vector2.right);
            float thetaLeft = Vector2.Angle(boundLeft, Vector2.right);

            return InBetween(thetaRight, theta, thetaLeft);
        }

        private bool InBetween(float thetaRight, float thetaDif, float thetaLeft)
        {
            thetaRight = NormalizeAngle(thetaRight);
            thetaDif = NormalizeAngle(thetaDif);
            thetaLeft = NormalizeAngle(thetaLeft);

            if (Mathf.Abs(thetaRight - thetaLeft) <= 180)
            {
                return thetaRight <= thetaDif && thetaDif <= thetaLeft;
            }
            else
            {
                if (thetaLeft < thetaRight)
                {
                    thetaLeft += 360;
                    if (thetaDif < thetaRight) thetaDif += 360;
                }
                return thetaRight <= thetaDif && thetaDif <= thetaLeft;
            }
        }

        private float NormalizeAngle(float angle)
        {
            while (angle < 0) angle += 360;
            while (angle > 360) angle -= 360;
            return angle;
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