using System.Collections.Generic;
using System.Linq;
using PathPlanning;
using UnityEngine;

namespace avoidance
{
    public class VOManager
    {
        public static bool DebugOn = false;

        private CollisionAvoidanceAlgorithm collisionAvoidanceAlgorithm;

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

        public VOManager(CollisionAvoidanceAlgorithm collisionAvoidanceAlgorithm)
        {
            this.collisionAvoidanceAlgorithm = collisionAvoidanceAlgorithm;
        }

        public void SetCollisionAvoidanceAlgorithm(CollisionAvoidanceAlgorithm collisionAvoidanceAlgorithm)
        {
            this.collisionAvoidanceAlgorithm = collisionAvoidanceAlgorithm;
        }

        public Vector2 CalculateNewVelocity(Agent agent, float deltaTime, out bool isColliding)
        {
            return collisionAvoidanceAlgorithm.CalculateNewVelocity(agent, deltaTime, agents, out isColliding);
        }

        public void DrawDebug(Agent agent)
        {
            collisionAvoidanceAlgorithm.DrawDebug(agent, agents);
        }
    }
}
