using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace vo
{
    public class VOManager
    {
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

        // Calculate new velocity for the agent to avoid multiple obstacles, always go right of obstacles
        public void CalculateNewVelocity(Agent agent, float maxSpeed, float maxAngle, bool allowReversing, out bool isColliding, out Vector2 newVelocity) // maxAngle in degrees
        {
            List<VelocityObstacle> vos = CalculateVelocityObstacles(agent);
            isColliding = false;

            foreach (VelocityObstacle vo in vos)
            {
                Vector2 velocityInVoSpace = agent.Velocity - vo.transl_vB_vA - agent.Velocity;
                float theta = Vector2.Angle(Vector2.right, velocityInVoSpace);

                float thetaRight = Vector2.Angle(vo.bound_right, Vector2.right);
                float thetaLeft = Vector2.Angle(vo.bound_left, Vector2.right);
                
                if (InBetween(thetaRight, theta, thetaLeft))
                {
                    isColliding = true;
                    break;
                }
            }

            if (!isColliding)
            {
                newVelocity = Vector2.zero;
                return;
            }

            newVelocity = Vector2.positiveInfinity;
            float angleStep = 5;
            float speedStep = maxSpeed / 5f;

            // Sample in VO space around wanted velocity
            for (float alpha = -maxAngle; alpha <= maxAngle; alpha += angleStep)
            {
                for (float speed = allowReversing ? -maxSpeed : 0f; speed <= maxSpeed; speed += speedStep)
                {
                    bool suitable = true;
                    Vector2 sampleVelocity = Quaternion.Euler(0, 0, alpha) * agent.Velocity.normalized * speed;

                    foreach (VelocityObstacle vo in vos)
                    {
                        Vector2 velocityInVoSpace = sampleVelocity - vo.transl_vB_vA - agent.Velocity;
                        float theta = Vector2.Angle(Vector2.right, velocityInVoSpace);

                        float thetaRight = Vector2.Angle(vo.bound_right, Vector2.right);
                        float thetaLeft = Vector2.Angle(vo.bound_left, Vector2.right);
                        
                        if (InBetween(thetaRight, theta, thetaLeft))
                        {
                            suitable = false;
                            break;
                        }
                    }
                    
                    if (suitable && Vector2.Distance(sampleVelocity, agent.Velocity) < Vector2.Distance(newVelocity, agent.Velocity))
                    {
                        newVelocity = sampleVelocity;
                    }
                }
            }

            // DebugVos(agent, vos);
            // DebugAgent(agent);
        }

        // Calculate velocity obstacles for an agent
        private List<VelocityObstacle> CalculateVelocityObstacles(Agent agentA)
        {
            List<VelocityObstacle> vos = new();
            
            // Exclude self and really slow agents
            foreach (Agent agentB in agents.Where(b => b != agentA 
                                                         && b.Velocity.magnitude > 0.1f))
            {
                Vector2 transl_vB_vA = agentB.Velocity - agentA.Velocity;
                Vector2 direction_BA = (agentB.Position - agentA.Position).normalized;
                float dist_BA = Vector2.Distance(agentA.Position, agentB.Position);
                float rad = agentA.Radius + agentB.Radius;

                Vector2 perpendicular_BA = Vector2.Perpendicular(direction_BA);
                Vector2 bound_left = direction_BA * dist_BA + perpendicular_BA * rad;
                Vector2 bound_right = direction_BA * dist_BA - perpendicular_BA * rad;

                var vo = new VelocityObstacle(transl_vB_vA, bound_left, bound_right, dist_BA, rad);
                vos.Add(vo);
            }
            return vos;
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

        private void DebugVos(Agent agent, List<VelocityObstacle> vos)
        {
            if (vos.Count > 0){
                VelocityObstacle vo = vos[0];
                Debug.DrawLine(Vec2To3(agent.Position), Vec2To3(agent.Position + vo.bound_left), Color.magenta);
                Debug.DrawLine(Vec2To3(agent.Position), Vec2To3(agent.Position + vo.bound_right), Color.magenta);
            }
        }

        private void DebugAgent(Agent agent)
        {
            Debug.DrawLine(Vec2To3(agent.Position), Vec2To3(agent.Position + agent.Velocity.normalized * agent.Radius), Color.red);
            Debug.DrawLine(Vec2To3(agent.Position), Vec2To3(agent.Position - agent.Velocity.normalized * agent.Radius), Color.red);
        }

        private Vector3 Vec2To3(Vector2 vec)
        {
            return new Vector3(vec.x, 0f, vec.y);
        }
    }
}
