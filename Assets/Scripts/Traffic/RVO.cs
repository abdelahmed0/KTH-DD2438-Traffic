
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

        public override Vector2 CalculateNewVelocity(Agent agent, List<Agent> agents, out bool velColliding)
        {
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, agents, RvoAlpha);
            velColliding = false;

            const float speedSamples = 5f;
            const float angleSamples = 5f;
            float w = agent.aggresiveness; // Aggressiveness factor, lower is more aggressive since collisions are penalized less

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float lowerSpeedBound = Mathf.Clamp(agent.Velocity.magnitude - maxAccelaration * TimeLookAhead, allowReversing ? -maxSpeed : 0f, maxSpeed);
            float upperSpeedBound = Mathf.Clamp(agent.Velocity.magnitude + maxAccelaration * TimeLookAhead, allowReversing ? -maxSpeed : 0f, maxSpeed);

            float speedStep = Mathf.Max(1f, (upperSpeedBound - lowerSpeedBound) / speedSamples);
            float angleStep = Mathf.Max(1f, 2f * maxAngle / angleSamples);
            
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
                    Vector2 sampleVelocity = Quaternion.Euler(0f, 0f, alpha) * direction * speed;

                    float penalty = Vector2.Distance(sampleVelocity, agent.DesiredVelocity);
                    penalty += Vector2.Angle(sampleVelocity, agent.DesiredVelocity) / 90; // Use angle for more stable paths

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
                        // Practically guarantees sampling free velocities if a non colliding sample exists
                        penalty -= 100000f;
                        sampleColliding = false;
                    }

                    if (penalty < minPenalty)
                    {
                        Debug.DrawLine(new Vector3((agent.Position + agent.Velocity).x, 1f, (agent.Position + agent.Velocity).y),
                                       new Vector3((agent.Position + sampleVelocity).x, 1f, (agent.Position + sampleVelocity).y),
                                       new Color(1f, 0f, 0f, 0.1f));
                        minPenalty = penalty;
                        newVelocity = sampleVelocity;
                        velColliding = sampleColliding;
                    }
                }
            }
            return newVelocity;
        }

        public override void DrawDebug(Agent agent, List<Agent> agents)
        {
            if (agent == agents[1])// || agent == agents[^1])
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
                        
                        // Draw line to agent causing this VO
                        Debug.DrawLine(Vec2To3(rvo.agentB.Position), Vec2To3(agent.Position + rvo.apex), new Color(189f, 195f, 199f, 0.7f));
                    }
                }

                Gizmos.color = Color.green;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.Velocity), 1f);

                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.DesiredVelocity), 1f);

                var avoidanceVelocity = CalculateNewVelocity(agent, agents, out bool velColliding);
                Gizmos.color = velColliding ? Color.red : Color.yellow;
                Gizmos.DrawSphere(Vec2To3(agent.Position + avoidanceVelocity), 1f);
            }
        }

    }
}