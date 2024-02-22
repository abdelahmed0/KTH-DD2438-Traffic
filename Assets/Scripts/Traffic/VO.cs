using System.Collections.Generic;
using System.Linq;
using PathPlanning;
using UnityEngine;

namespace vo
{
    public class VOManager
    {
        public static bool DebugOn = false;

        public const float TimeLookAhead = 4f; // Functions as a truncation of the VOs
        public const float st_TimeLookaHead = 3f; // Static obstacle collision time lookahead
        public float maxSpeed = 50f;
        public float maxAngle = 30f;
        public float maxAccelaration = 5f;
        public bool allowReversing = false;
        public CollisionDetector Detector = null;

        private const float RvoAlpha = 1f; // RVO: VO apex is translated by alpha from velocity B to velocity A

        private List<Agent> agents;

        public VOManager()
        {
            agents = new List<Agent>();
        }

        public void AddAgent(Agent agent)
        {
            agents.Add(agent);
        }

        public void UpdateAgent(Agent toUpdate, Agent toCopy)
        {
            agents.Find(a => a == toUpdate).Update(toCopy);
        }

        public void CalculateNewRVOVelocity(Agent agent, float deltaTime, out bool isColliding, out Vector2 newVelocity)
        {
            List<VelocityObstacle> vos = CalculateVelocityObstacles(agent);
            isColliding = false;

            newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float angleStep = 5;
            float wi = 1f; // Aggressiveness factor, can vary among agents

            // Sample in VO space around wanted velocity
            for (float alpha = -maxAngle; alpha <= maxAngle; alpha += angleStep)
            {
                float lowerSpeedBound = Mathf.Clamp(agent.Velocity.magnitude - maxAccelaration * deltaTime, allowReversing ? -maxSpeed : 0f, maxSpeed);
                float upperSpeedBound = Mathf.Clamp(agent.Velocity.magnitude + maxAccelaration * deltaTime, allowReversing ? -maxSpeed : 0f, maxSpeed);

                float speedStep = (upperSpeedBound - lowerSpeedBound) / 5f;
                for (float speed = lowerSpeedBound; speed <= upperSpeedBound; speed += speedStep)
                {
                    Vector2 sampleVelocity = Quaternion.Euler(0, 0, alpha) * agent.Velocity.normalized * speed;
                    float penalty = Vector2.Distance(sampleVelocity, agent.DesiredVelocity);
                    float minTimeToCollision = float.MaxValue;
                    
                    // Check static obstacles
                    if (Detector.LineCollision(agent.Position, agent.Position + sampleVelocity.normalized * st_TimeLookaHead))
                    {
                        // Debug.DrawLine(Vec2To3(agent.Position), Vec2To3(agent.Position + sampleVelocity.normalized * st_TimeLookaHead), Color.magenta);
                        continue;
                    }
                    
                    // Check dynamic obstacles
                    foreach (VelocityObstacle vo in vos)
                    {
                        if (IsInVO(sampleVelocity, vo))
                        {
                            // float timeToCollision = CalculateTimeToCollision(vo, sampleVelocity);
                            float timeToCollision = sampleVelocity.magnitude == 0f ? float.MaxValue : (vo.dist_BA - vo.rad) / sampleVelocity.magnitude;
                            
                            minTimeToCollision = Mathf.Min(minTimeToCollision, timeToCollision);
                        }
                    }

                    if (minTimeToCollision < float.MaxValue)
                    {
                        penalty += wi / minTimeToCollision; // Apply penalty based on time to collision
                    }

                    if (penalty < minPenalty)
                    {
                        minPenalty = penalty;
                        newVelocity = sampleVelocity;
                        isColliding = minTimeToCollision < TimeLookAhead; // Update collision status based on closest obstacle
                    }
                }
            }

            if (newVelocity == Vector2.positiveInfinity)
            {
                newVelocity = agent.DesiredVelocity; // Fallback to desired velocity if no suitable velocity found
            }
        }

        // Calculate velocity obstacles for an agent if collision would happen within the time threshold 
        private List<VelocityObstacle> CalculateVelocityObstacles(Agent agentA)
        {
            List<VelocityObstacle> vos = new();
            
            // Exclude self
            foreach (Agent agentB in agents.Where(b => b != agentA))
            {
                Vector2 transl_vB_vA = agentB.Velocity - agentA.Velocity;
                Vector2 direction_BA = (agentB.Position - agentA.Position).normalized;
                float dist_BA = Vector2.Distance(agentA.Position, agentB.Position);
                float rad = agentA.Radius + agentB.Radius;

                Vector2 vo_apex = (agentA.Velocity + agentB.Velocity) * RvoAlpha - direction_BA * dist_BA; 

                Vector2 perpendicular_BA = Vector2.Perpendicular(direction_BA);
                Vector2 bound_left = direction_BA * dist_BA + perpendicular_BA * rad;
                Vector2 bound_right = direction_BA * dist_BA - perpendicular_BA * rad;

                var vo = new VelocityObstacle(agentA, agentB, vo_apex, bound_left, bound_right, dist_BA, rad);
                vos.Add(vo);
            }
            return vos;
        }

        private bool IsInVO(Vector2 sampleVelocity, VelocityObstacle vo)
        {
            Vector2 velocityInVoSpace = sampleVelocity - vo.apex;
            float theta = Vector2.Angle(Vector2.right, velocityInVoSpace);

            float thetaRight = Vector2.Angle(vo.bound_right, Vector2.right);
            float thetaLeft = Vector2.Angle(vo.bound_left, Vector2.right);

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

        public void DebugDraw(Agent agent)
        {
            if (agent == agents[0])
            {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(Vec2To3(agent.Position), agent.Radius);

                var vos = CalculateVelocityObstacles(agent);
                foreach (var vo in vos)
                {
                    Gizmos.color = Color.red;

                    Vector2 boundLeftWorld = agent.Position + vo.bound_left;
                    Vector2 boundRightWorld = agent.Position + vo.bound_right;
                    
                    Gizmos.DrawLine(Vec2To3(agent.Position), Vec2To3(boundLeftWorld));
                    Gizmos.DrawLine(Vec2To3(agent.Position), Vec2To3(boundRightWorld));
                }
            } else {
                Gizmos.color = Color.cyan;
                Gizmos.DrawSphere(Vec2To3(agent.Position), agent.Radius);
            }
        }

        private Vector3 Vec2To3(Vector2 vec)
        {
            return new Vector3(vec.x, 3f, vec.y);
        }
    }
}
