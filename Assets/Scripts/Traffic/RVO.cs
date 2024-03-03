
using System.Collections.Generic;
using UnityEngine;

namespace avoidance
{
    public class RVOAlgorithm : CollisionAvoidanceAlgorithm
    {
        float RvoAlpha;

        // RVO: VO apex is translated relatively by alpha from velocity B to velocity A, so rvoAlpha = 0f are usual VOs 
        public RVOAlgorithm(float rvoAlpha)
        {
            RvoAlpha = rvoAlpha;
        }

        public override Vector2 CalculateNewVelocity(Agent agent, List<Agent> agents, out bool isColliding)
        {
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, agents, RvoAlpha);
            isColliding = false;

            bool desiredPossible = true;
            // Check velocity obstacles
            foreach (VelocityObstacle rvo in rvos)
            {
                if (rvo.ContainsVelocity(agent.DesiredVelocity))
                {
                    desiredPossible = false;
                }
            }
            if (desiredPossible)
                return agent.DesiredVelocity;

            float speedSamples = 3f; // per orientation
            const float angleSamples = 3f;
            float w = agent.aggresiveness; // Aggressiveness factor, lower is more aggressive since collisions are penalized less

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float lowerSpeedBound = Mathf.Clamp(agent.Velocity.magnitude - maxAccelaration * TimeLookAhead, allowReversing ? -maxSpeed : 0f, maxSpeed);
            float upperSpeedBound = Mathf.Clamp(agent.Velocity.magnitude + maxAccelaration * TimeLookAhead, allowReversing ? -maxSpeed : 0f, maxSpeed);

            float speedStep = Mathf.Max(1f, (upperSpeedBound - lowerSpeedBound) / speedSamples);
            float angleStep = Mathf.Max(1f, maxAngle / angleSamples);
            
            // Sample in VO space around wanted velocity
            for (float alpha = -maxAngle; alpha <= maxAngle; alpha += angleStep)
            {
                for (float speed = lowerSpeedBound; speed <= upperSpeedBound; speed += speedStep)
                {
                    Vector2 direction;
                     if (agent.Velocity.magnitude > Mathf.Epsilon)
                    {
                        direction = agent.Velocity.normalized;
                    }
                    else
                    {
                        // If current velocity is almost zero, use desired velocity direction or a default direction if that's also zero
                        direction = agent.DesiredVelocity.magnitude > Mathf.Epsilon ? agent.DesiredVelocity.normalized : Vector2.up;
                    }
                    Vector2 sampleVelocity = Quaternion.Euler(0, 0, alpha) * direction * speed;

                    float penalty = Vector2.Distance(sampleVelocity, agent.DesiredVelocity);
                    penalty += Vector2.Angle(sampleVelocity, agent.DesiredVelocity) / 180f; // Use angle for more stable paths
                    float minTimeToCollision = float.MaxValue;

                    // Check static obstacles
                    if (Detector.LineCollision(agent.Position, agent.Position + sampleVelocity * st_TimeLookaHead))
                    {
                        // Debug.DrawLine(Vec2To3(agent.Position), Vec2To3(agent.Position + sampleVelocity * st_TimeLookaHead), Color.white);
                        continue;
                    }

                    // Check velocity obstacles
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
                    {
                        sampleColliding = false;
                        penalty -= 100000f; // practically guarantees sampling free velocities if possible
                    }

                    if (penalty < minPenalty)
                    {
                        Debug.DrawLine(new Vector3(agent.Position.x, 0.5f, agent.Position.y),
                            new Vector3(agent.Position.x + sampleVelocity.x, 0.5f, agent.Position.y + sampleVelocity.y),
                            new Color(1f, 0f, 0f, 0.1f));
                        minPenalty = penalty;
                        newVelocity = sampleVelocity;
                        isColliding = sampleColliding;
                    }
                }
            }
            return newVelocity;
        }

        public override void DrawDebug(Agent agent, List<Agent> agents)
        {
            if (agent == agents[1] || agent == agents[^1])
            {                
                var rvos = CalculateVelocityObstacles(agent, agents);
                foreach (var rvo in rvos)
                {
                    // Draw RVO
                    Gizmos.color = new Color(189f, 195f, 199f, 0.7f); // grey

                    if (rvo.ContainsVelocity(rvo.agentB.Velocity) 
                        && rvo.CollisionTimeFromVelocity(rvo.agentB.Velocity) < TimeLookAhead)
                    {
                        Vector2 boundLeftWorld = rvo.boundLeft * TimeLookAhead + rvo.apex;
                        Vector2 boundRightWorld = rvo.boundRight * TimeLookAhead + rvo.apex;
                        
                        DrawTriangle(agent.Position + rvo.apex, // Offset to be closer over agent
                                    agent.Position + boundLeftWorld,
                                    agent.Position + boundRightWorld);
                    }

                }

            }
        }

    }
}