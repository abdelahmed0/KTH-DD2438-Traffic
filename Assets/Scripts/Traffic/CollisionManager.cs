using System.Collections.Generic;
using System.Linq;
using PathPlanning;
using UnityEngine;

namespace avoidance
{
    public class CollisionManager
    {
        public static bool DebugOn = false;

        private CollisionAvoidanceAlgorithm collisionAvoidanceAlgorithm;

        private List<Agent> agents;

        public CollisionManager()
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

        public CollisionManager(CollisionAvoidanceAlgorithm collisionAvoidanceAlgorithm)
        {
            this.collisionAvoidanceAlgorithm = collisionAvoidanceAlgorithm;
        }

        public void SetCollisionAvoidanceAlgorithm(CollisionAvoidanceAlgorithm collisionAvoidanceAlgorithm)
        {
            this.collisionAvoidanceAlgorithm = collisionAvoidanceAlgorithm;
        }

        public CollisionAvoidanceAlgorithm GetCollisionAvoidanceAlgorithm()
        {
            return collisionAvoidanceAlgorithm;
        }

        public Vector2 CalculateNewVelocity(Agent agent, out bool isColliding)
        {
            return collisionAvoidanceAlgorithm.CalculateNewVelocity(agent, agents, out isColliding);
        }

        public void DrawDebug(Agent agent)
        {
            collisionAvoidanceAlgorithm.DrawDebug(agent, agents);
        }
    }
}
