
using System.Collections.Generic;
using System.Linq;
using PathPlanning;
using UnityEngine;

namespace avoidance 
{
    public class HRVOAlgorithm : CollisionAvoidanceAlgorithm
    {
        public override Vector2 CalculateNewVelocity(Agent agent, float deltaTime, List<Agent> agents, out bool isColliding)
        {
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, agents, 0.5f);

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float angleStep = 10f;
            float w = 1f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less
            isColliding = false;

            // Sample in VO space around wanted velocity
            for (float alpha = -maxAngle; alpha <= maxAngle; alpha += angleStep)
            {
                float lowerSpeedBound = Mathf.Clamp(agent.Velocity.magnitude - maxAccelaration * deltaTime, allowReversing ? -maxSpeed : 0f, maxSpeed);
                float upperSpeedBound = Mathf.Clamp(agent.Velocity.magnitude + maxAccelaration * deltaTime, allowReversing ? -maxSpeed : 0f, maxSpeed);
                
                float speedStep = Mathf.Max((upperSpeedBound - lowerSpeedBound) / 3f, 0.1f);
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
                    // Check velocity obstacles
                    foreach (VelocityObstacle rvo in rvos)
                    {
                        bool isRightOfCenterline = rvo.VelocityRightOfCenterLine(sampleVelocity);

                        // Construct HRVO based on the position relative to the centerline
                        VelocityObstacle hrvo = ConstructHRVO(rvo, isRightOfCenterline);

                        if (hrvo.ContainsVelocity(sampleVelocity))
                        {
                            float timeToCollision = hrvo.CollisionTimeFromVelocity(sampleVelocity);
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

        private VelocityObstacle ConstructHRVO(VelocityObstacle rvo, bool isRightOfCenterline)
        {
            Vector2 direction_BA = (rvo.agentB.Position - rvo.agentA.Position).normalized;
            float dist_BA = Vector2.Distance(rvo.agentA.Position, rvo.agentB.Position);
            float radius = rvo.combinedRadius;

            Vector2 perpendicular_BA = Vector2.Perpendicular(direction_BA);
            Vector2 bound_left = direction_BA * dist_BA + perpendicular_BA * radius;
            Vector2 bound_right = direction_BA * dist_BA - perpendicular_BA * radius;

            Vector2 vo_apex_offset = rvo.apex - rvo.agentB.Velocity;

            Vector2 vo_apex = rvo.apex;

            // Replace one side of RVO with VO based on centerline
            if (isRightOfCenterline)
            {
                // Project apex onto VOs left bound for HRVO
                if (TryGetLineIntersection(vo_apex_offset, bound_right.normalized, Vector2.zero, rvo.boundLeft.normalized,
                        out Vector2 intersection))
                    vo_apex = intersection + rvo.apex;
            }
            else
            {
                // Project apex onto VOs right bound for HRVO
                if (TryGetLineIntersection(vo_apex_offset, bound_left.normalized, Vector2.zero, rvo.boundRight.normalized,
                        out Vector2 intersection))
                    vo_apex = intersection + rvo.apex;
            }

            return new VelocityObstacle(rvo.agentA, rvo.agentB, vo_apex, bound_left, bound_right, dist_BA, radius);
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

                    // Draw HRVO
                    Gizmos.color = Color.cyan;

                    bool isRight = rvo.VelocityRightOfCenterLine(agent.Velocity);
                    VelocityObstacle hrvo = ConstructHRVO(rvo, isRight);
                    
                    if (hrvo.ContainsVelocity(hrvo.agentB.Velocity) 
                        && hrvo.CollisionTimeFromVelocity(hrvo.agentB.Velocity) < TimeLookAhead)
                    {

                        Vector2 boundLeftWorld = hrvo.apex + hrvo.boundLeft;
                        Vector2 boundRightWorld = hrvo.apex + hrvo.boundRight;
                        
                        DrawTriangle(agent.Position + hrvo.apex, // Offset to be closer over agent
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
