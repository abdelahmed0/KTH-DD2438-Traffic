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
        public Vector3 CalculateNewVelocity(Agent agent)
        {
            // TODO: Sample in VO space around wanted velocity

            Vector3 newVelocity = agent.DesiredVelocity;
            List<VelocityObstacle> vos = CalculateVelocityObstacles(agent);
            
            foreach (VelocityObstacle vo in vos)
            {
                float thetaRight = Vector3.Angle(vo.bound_right, Vector3.right);
                float thetaLeft = Vector3.Angle(vo.bound_left, Vector3.right);
                float theta = Vector3.Angle(agent.Velocity - vo.transl_vB_vA, Vector3.right);
                
                if (InBetween(thetaRight, theta, thetaLeft)) 
                {
                    newVelocity = vo.bound_right.normalized * newVelocity.magnitude;
                }
            }
            Debug.DrawLine(agent.Position, agent.Position + agent.Velocity, Color.blue);
            Debug.DrawLine(agent.Position, agent.Position + newVelocity, Color.red);
            // DebugVos(agent, vos);
            // DebugAgent(agent);

            return newVelocity;
        }

        // Calculate velocity obstacles for an agent
        private List<VelocityObstacle> CalculateVelocityObstacles(Agent agentA)
        {
            List<VelocityObstacle> vos = new();
            foreach (Agent agentB in agents.Where(b => b != agentA))
            {
                Vector3 transl_vB_vA = agentB.Velocity - agentA.Velocity;
                Vector3 direction_BA = (agentB.Position - agentA.Position).normalized;
                float dist_BA = Vector3.Distance(agentA.Position, agentB.Position);
                float rad = agentA.Radius + agentB.Radius;

                Vector2 perendicular_BA = Vector2.Perpendicular(new Vector2(direction_BA.x, direction_BA.z));
                Vector3 bound_left = direction_BA * dist_BA + new Vector3(perendicular_BA.x, direction_BA.y, perendicular_BA.y) * rad;
                Vector3 bound_right = direction_BA * dist_BA - new Vector3(perendicular_BA.x, direction_BA.y, perendicular_BA.y) * rad;

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
            VelocityObstacle vo = vos[0];
            Debug.DrawLine(agent.Position, agent.Position + vo.bound_left, Color.blue);
            Debug.DrawLine(agent.Position, agent.Position + vo.bound_right, Color.blue);
        }

        private void DebugAgent(Agent agent)
        {
            Debug.DrawLine(agent.Position, agent.Position + agent.Velocity.normalized * agent.Radius, Color.red);
            Debug.DrawLine(agent.Position, agent.Position - agent.Velocity.normalized * agent.Radius, Color.red);
        }
    }
}
