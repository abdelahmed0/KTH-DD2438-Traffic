
using System.Collections.Generic;
using UnityEngine;

namespace avoidance
{
    public class HRVODroneAlgorithm : CollisionAvoidanceAlgorithm
    {
        public override Vector2 CalculateNewVelocity(Agent agent, List<Agent> agents, out bool isColliding)
        {
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, agents, 0.5f);

            const float speedSamples = 3f;
            const float angleSamples = 3f;
            const float w = 10f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float lowerSpeedBound = Mathf.Clamp(agent.Velocity.magnitude - maxAccelaration, allowReversing ? -maxSpeed : 0f, maxSpeed);
            float upperSpeedBound = Mathf.Clamp(agent.Velocity.magnitude + maxAccelaration, allowReversing ? -maxSpeed : 0f, maxSpeed);
            
            lowerSpeedBound = -maxSpeed;
            upperSpeedBound = maxSpeed;

            float speedStep = Mathf.Max(1f, (upperSpeedBound - lowerSpeedBound) / speedSamples);
            float angleStep = Mathf.Max(1f, maxAngle / angleSamples);

            isColliding = false;
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
                    } else
                    {
                        Debug.DrawLine(new Vector3(agent.Position.x, 0.1f, agent.Position.y),
                            new Vector3(agent.Position.x + sampleVelocity.x, 0.1f, agent.Position.y + sampleVelocity.y),
                            new Color(1f, 0f, 0f, 0.1f));
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
            if (agent == agents[11] || agent == agents[13])
            {
                var rvos = CalculateVelocityObstacles(agent, agents);
                foreach (var rvo in rvos)
                {
                    // // Draw RVO
                    // Gizmos.color = new Color(189f, 195f, 199f, 0.7f); // grey

                    // if (rvo.ContainsVelocity(rvo.agentB.Velocity) 
                    //     && rvo.CollisionTimeFromVelocity(rvo.agentB.Velocity) < TimeLookAhead)
                    // {
                    //     Vector2 boundLeftWorld = rvo.boundLeft * TimeLookAhead + rvo.apex;
                    //     Vector2 boundRightWorld = rvo.boundRight * TimeLookAhead + rvo.apex;
                        
                    //     DrawTriangle(agent.Position + rvo.apex, // Offset to be closer over agent
                    //                 agent.Position + boundLeftWorld,
                    //                 agent.Position + boundRightWorld);
                    // }

                    // Draw HRVO
                    bool isRight = rvo.VelocityRightOfCenterLine(agent.Velocity);
                    VelocityObstacle hrvo = ConstructHRVO(rvo, isRight);

                    var avoidanceVelocity = CalculateNewVelocity(agent, agents, out bool isColliding);
                    if (isColliding)
                    {
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawSphere(Vec2To3(agent.Position + avoidanceVelocity), 1f);
                    }
                    
                    Gizmos.color = new Color(0f, 1f, 1f, 0.7f); // cyan
                    if (hrvo.ContainsVelocity(agent.Velocity)
                            && hrvo.CollisionTimeFromVelocity(agent.Velocity) < TimeLookAhead)
                    {
                        Vector2 boundLeftWorld =  hrvo.boundLeft * TimeLookAhead + hrvo.apex;
                        Vector2 boundRightWorld = hrvo.boundRight * TimeLookAhead + hrvo.apex;
                        
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


    public class HRVOCarAlgorithm : CollisionAvoidanceAlgorithm
    {
        public override Vector2 CalculateNewVelocity(Agent agent, List<Agent> agents, out bool isColliding)
        {
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, agents, 0.2f);

            float speedSamples = 3f; // per orientation
            const float angleSamples = 3f;
            float w = 1f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float lowerSpeedBound = Mathf.Clamp(agent.Velocity.magnitude - maxAccelaration, allowReversing ? -maxSpeed : 0f, maxSpeed);
            float upperSpeedBound = Mathf.Clamp(agent.Velocity.magnitude + maxAccelaration, allowReversing ? -maxSpeed : 0f, maxSpeed);
            
            lowerSpeedBound = -maxSpeed;
            upperSpeedBound = maxSpeed;

            float speedStep = Mathf.Max(1f, (upperSpeedBound - lowerSpeedBound) / speedSamples);
            float angleStep = Mathf.Max(1f, maxAngle / angleSamples);
            
            isColliding = false;
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
                    } else
                    {
                        Debug.DrawLine(new Vector3(agent.Position.x, 0.1f, agent.Position.y),
                            new Vector3(agent.Position.x + sampleVelocity.x, 0.1f, agent.Position.y + sampleVelocity.y),
                            new Color(1f, 0f, 0f, 0.1f));
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
            if (agent == agents[0])// || agent == agents[13])
            {
                var rvos = CalculateVelocityObstacles(agent, agents);
                foreach (var rvo in rvos)
                {
                    // // Draw RVO
                    // Gizmos.color = new Color(189f, 195f, 199f, 0.7f); // grey

                    // if (rvo.ContainsVelocity(rvo.agentB.Velocity) 
                    //     && rvo.CollisionTimeFromVelocity(rvo.agentB.Velocity) < TimeLookAhead)
                    // {
                    //     Vector2 boundLeftWorld = rvo.boundLeft * TimeLookAhead + rvo.apex;
                    //     Vector2 boundRightWorld = rvo.boundRight * TimeLookAhead + rvo.apex;
                        
                    //     DrawTriangle(agent.Position + rvo.apex, // Offset to be closer over agent
                    //                 agent.Position + boundLeftWorld,
                    //                 agent.Position + boundRightWorld);
                    // }

                    // Draw HRVO
                    bool isRight = rvo.VelocityRightOfCenterLine(agent.Velocity);
                    VelocityObstacle hrvo = ConstructHRVO(rvo, isRight);

                    var avoidanceVelocity = CalculateNewVelocity(agent, agents, out bool isColliding);
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawSphere(Vec2To3(agent.Position + avoidanceVelocity), 1f);
                    
                    Gizmos.color = new Color(0f, 1f, 1f, 0.7f); // cyan
                    if (hrvo.ContainsVelocity(agent.Velocity)
                            && hrvo.CollisionTimeFromVelocity(agent.Velocity) < TimeLookAhead)
                    {
                        Vector2 boundLeftWorld =  hrvo.boundLeft * TimeLookAhead + hrvo.apex;
                        Vector2 boundRightWorld = hrvo.boundRight * TimeLookAhead + hrvo.apex;
                        
                        DrawTriangle(agent.Position + hrvo.apex, // Offset to be closer over agent
                                    agent.Position + boundLeftWorld,
                                    agent.Position + boundRightWorld);
                    }

                }

                Gizmos.color = Color.green;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.Velocity), 1f);

                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.DesiredVelocity), 1f);
            }
        }
    }

}
