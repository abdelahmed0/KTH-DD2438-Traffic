
using System.Collections.Generic;
using UnityEngine;

namespace avoidance
{
    public class RVOAlgorithm : CollisionAvoidanceAlgorithm
    {

        public override Vector2 CalculateNewVelocity(Agent agent, float deltaTime, List<Agent> agents, out bool isColliding)
        {
            // RVO: VO apex is translated by alpha from velocity B to velocity A
            float rvoAlpha = 0.5f;
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, agents, rvoAlpha);

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float angleStep = 5;
            float w = 5f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less
            isColliding = false;

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
                    penalty += Vector2.Angle(sampleVelocity, agent.DesiredVelocity) / 180f; // Use angle for more stable paths
                    float minTimeToCollision = float.MaxValue;

                    // Check static obstacles
                    if (Detector.LineCollision(agent.Position, agent.Position + sampleVelocity.normalized * st_TimeLookaHead))
                    {
                        // Debug.DrawLine(Vec2To3(agent.Position), Vec2To3(agent.Position + sampleVelocity.normalized * st_TimeLookaHead), Color.magenta);
                        continue;
                    }

                    // Check dynamic obstacles
                    foreach (VelocityObstacle rvo in rvos)
                    {
                        if (rvo.ContainsVelocity(sampleVelocity))
                        {
                            float timeToCollision = rvo.CollisionTimeFromVelocity(sampleVelocity);
                            minTimeToCollision = Mathf.Min(minTimeToCollision, timeToCollision);
                        }
                    }

                    bool sampleColliding;
                    if (minTimeToCollision < TimeLookAhead)
                    {
                        penalty += w / minTimeToCollision; // Apply penalty based on time to collision
                        sampleColliding = true;
                    }
                    else
                        sampleColliding = false;

                    if (penalty < minPenalty)
                    {
                        minPenalty = penalty;
                        newVelocity = sampleVelocity;
                        isColliding = sampleColliding;
                    }
                }
            }
            return newVelocity;
        }

        // Draw agent velocity obstacles in velocity space
        public override void DrawDebug(Agent agent, List<Agent> agents)
        {
            if (agent == agents[1] || agent == agents[^1])
            {
                var rvos = CalculateVelocityObstacles(agent, agents);
                foreach (var rvo in rvos)
                {
                    // Draw RVO
                    Gizmos.color = Color.grey;

                    if (rvo.ContainsVelocity(rvo.agentB.Velocity) 
                        && rvo.CollisionTimeFromVelocity(rvo.agentB.Velocity) < TimeLookAhead)
                    {
                        Vector2 boundLeftWorld = rvo.apex + rvo.boundLeft;
                        Vector2 boundRightWorld = rvo.apex + rvo.boundRight;
                        
                        DrawTriangle(agent.Position + rvo.apex, // Offset to be closer over agent
                                    agent.Position + boundLeftWorld,
                                    agent.Position + boundRightWorld);
                    }
                }

                Gizmos.color = Color.green;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.Velocity), agent.Radius);

                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.DesiredVelocity), agent.Radius);
            }
        }

    }
}