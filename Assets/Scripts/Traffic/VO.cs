using System.Collections.Generic;
using System.Linq;
using PathPlanning;
using UnityEngine;

namespace vo
{
    public class VOManager
    {
        public static bool DebugOn = false;

        public const float TimeLookAhead = 5f; // Functions as a truncation of the VOs
        public const float st_TimeLookaHead = 3f; // Static obstacle collision time lookahead
        public float maxSpeed = 50f;
        public float maxAngle = 30f;
        public float maxAccelaration = 5f;
        public bool allowReversing = false;
        public CollisionDetector Detector = null;

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

        // Calculate new velocity considering HRVO
        public Vector2 CalculateNewHRVOVelocity(Agent agent, float deltaTime, out bool isColliding)
        {
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, 0.5f);

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float angleStep = 10;
            float w = 1f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less
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

        
        public Vector2 CalculateNewRVOVelocity(Agent agent, float deltaTime, out bool isColliding)
        {
            // RVO: VO apex is translated by alpha from velocity B to velocity A
            float rvoAlpha = 0.5f;
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, rvoAlpha);

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float angleStep = 5;
            float w = 1f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less
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

        private List<VelocityObstacle> CalculateVelocityObstacles(Agent agentA, float rvoAlpha = 1f)
        {
            List<VelocityObstacle> vos = new();
            
            // Exclude self
            foreach (Agent agentB in agents.Where(b => b != agentA))
            {
                Vector2 direction_BA = (agentB.Position - agentA.Position).normalized;
                float dist_BA = Vector2.Distance(agentA.Position, agentB.Position);
                float rad = agentA.Radius + agentB.Radius;

                Vector2 vo_apex = agentA.Velocity * (1f - rvoAlpha) + agentB.Velocity * rvoAlpha - direction_BA * dist_BA; 

                Vector2 perpendicular_BA = Vector2.Perpendicular(direction_BA);
                Vector2 bound_left = direction_BA * dist_BA + perpendicular_BA * rad;
                Vector2 bound_right = direction_BA * dist_BA - perpendicular_BA * rad;

                var vo = new VelocityObstacle(agentA, agentB, vo_apex, bound_left, bound_right, dist_BA, rad);
                vos.Add(vo);
            }
            return vos;
        }

        public bool TryGetLineIntersection(Vector2 p1, Vector2 dir1, Vector2 p2, Vector2 dir2, out Vector2 intersection)
        {
            intersection = Vector2.zero;

            // Calculate determinants
            float det = dir1.x * dir2.y - dir1.y * dir2.x;
            float det1 = (p2.x - p1.x) * dir2.y - (p2.y - p1.y) * dir2.x;
            float det2 = (p2.x - p1.x) * dir1.y - (p2.y - p1.y) * dir1.x;

            // If determinants are zero, lines are parallel and have no intersection
            if (Mathf.Approximately(det, 0f))
            {
                return false;
            }

            // Calculate the t value for the intersection point on the first line
            float t1 = det1 / det;

            // Use t1 to find the intersection point
            intersection = p1 + t1 * dir1;

            return true;
        }

        // // Draw agent velocity obstacles in agents position space
        // public void DebugDraw(Agent agent)
        // {
        //     if (agent == agents[0])
        //     {
        //         Gizmos.color = Color.green;
        //         Gizmos.DrawSphere(Vec2To3(agent.Position), agent.Radius);

        //         var vos = CalculateVelocityObstacles(agent);
        //         foreach (var vo in vos)
        //         {
        //             Gizmos.color = Color.red;

        //             Vector2 boundLeftWorld = agent.Position + vo.boundLeft;
        //             Vector2 boundRightWorld = agent.Position + vo.boundRight;
                    
        //             Gizmos.DrawLine(Vec2To3(agent.Position), Vec2To3(boundLeftWorld));
        //             Gizmos.DrawLine(Vec2To3(agent.Position), Vec2To3(boundRightWorld));
        //         }
        //     } else {
        //         Gizmos.color = Color.cyan;
        //         Gizmos.DrawSphere(Vec2To3(agent.Position), agent.Radius);
        //     }
        // }

        // Draw agent velocity obstacles in velocity space
        public void DebugDraw(Agent agent)
        {
            if (agent == agents[0]) //if (agent == agents[1] || agent == agents[agents.Count - 1])
            {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.Velocity), agent.Radius);

                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(Vec2To3(agent.Position + agent.DesiredVelocity), agent.Radius);

                var rvos = CalculateVelocityObstacles(agent);
                foreach (var rvo in rvos)
                {
                    bool isRight = rvo.VelocityRightOfCenterLine(agent.Velocity);
                    VelocityObstacle hrvo = ConstructHRVO(rvo, isRight);
                    
                    if (hrvo.ContainsVelocity(hrvo.agentB.Velocity) 
                        && hrvo.CollisionTimeFromVelocity(hrvo.agentB.Velocity) < TimeLookAhead)
                    {
                        Gizmos.color = Color.cyan;

                        Vector2 boundLeftWorld = hrvo.apex + hrvo.boundLeft;
                        Vector2 boundRightWorld = hrvo.apex + hrvo.boundRight;
                        
                        DrawTriangle(agent.Position + hrvo.apex, // Offset to be closer over agent
                                    agent.Position + boundLeftWorld,
                                    agent.Position + boundRightWorld);
                    }

                    // Gizmos.color = Color.grey;

                    // Vector2 boundLeftWorld = rvo.apex + rvo.boundLeft;
                    // Vector2 boundRightWorld = rvo.apex + rvo.boundRight;
                    
                    // DrawTriangle(agent.Position + rvo.apex, // Offset to be closer over agent
                    //              agent.Position + boundLeftWorld,
                    //              agent.Position + boundRightWorld);
                }
            }
        }

        private void DrawTriangle(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            Mesh m = new()
            {
                vertices = new Vector3[3]
            {
                new(p1.x, 7f, p1.y),
                new(p2.x, 7f, p2.y),
                new(p3.x, 7f, p3.y)
            },

                triangles = new int[]
            {
                0, 1, 2
            },

                normals = new Vector3[]
            {
                Vector3.up,
                Vector3.up,
                Vector3.up
            }
            };
            Gizmos.DrawMesh(m);
        }

        private Vector3 Vec2To3(Vector2 vec)
        {
            return new Vector3(vec.x, 10f, vec.y);
        }
    }
}
