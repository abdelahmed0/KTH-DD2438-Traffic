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
        
        public Vector2 CalculateNewRVOVelocity(Agent agent, float deltaTime, out bool isColliding)
        {
            // RVO: VO apex is translated by alpha from velocity B to velocity A
            float rvoAlpha = 0.5f;
            List<VelocityObstacle> rvos = CalculateVelocityObstacles(agent, rvoAlpha);

            Vector2 newVelocity = Vector2.positiveInfinity;
            float minPenalty = float.MaxValue;
            float angleStep = 5;
            float wi = 1f; // Aggressiveness factor, lower is more aggressive since collisions are penalized less
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
                    bool sampleColliding = false;

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

                            float timeToCollision;
                            if (sampleVelocity.magnitude == 0f)
                                timeToCollision = float.MaxValue;
                            else if (rvo.dist_BA - rvo.combinedRadius < 0f)
                                timeToCollision = 0.00001f;
                            else
                                timeToCollision = (rvo.dist_BA - rvo.combinedRadius) / sampleVelocity.magnitude;
                            
                            minTimeToCollision = Mathf.Min(minTimeToCollision, timeToCollision);
                        }
                    }

                    if (minTimeToCollision < TimeLookAhead)
                    {
                        penalty += wi / minTimeToCollision; // Apply penalty based on time to collision
                        sampleColliding = true;
                    } else
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
                Vector2 bound_left = direction_BA * dist_BA + perpendicular_BA * rad * 1.2f; // TODO: Try making bounds un symmetric
                Vector2 bound_right = direction_BA * dist_BA - perpendicular_BA * rad;

                var vo = new VelocityObstacle(agentA, agentB, vo_apex, bound_left, bound_right, dist_BA, rad);
                vos.Add(vo);
            }
            return vos;
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

                var vos = CalculateVelocityObstacles(agent);
                foreach (var vo in vos)
                {
                    Gizmos.color = Color.grey;

                    Vector2 boundLeftWorld = vo.apex + vo.boundLeft;
                    Vector2 boundRightWorld = vo.apex + vo.boundRight;
                    
                    DrawTriangle(agent.Position + vo.apex, // Offset to be closer over agent
                                 agent.Position + boundLeftWorld,
                                 agent.Position + boundRightWorld);
                }
            } else {
                Gizmos.color = Color.cyan;
                Gizmos.DrawSphere(Vec2To3(agent.Velocity), agent.Radius);
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
