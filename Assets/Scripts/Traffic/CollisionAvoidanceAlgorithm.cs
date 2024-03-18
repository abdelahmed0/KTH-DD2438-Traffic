
using System.Collections.Generic;
using System.Linq;
using PathPlanning;
using UnityEngine;

namespace avoidance 
{
    public abstract class CollisionAvoidanceAlgorithm
    {
        public float TimeLookAhead = 5f; // Functions as a truncation of the VOs
        public float st_TimeLookaHead = 3f; // Static obstacle collision time lookahead
        public float maxSpeed = 50f;
        public float maxAngle = 30f;
        public float maxAccelaration = 5f;
        public bool allowReversing = false;
        public CollisionDetector Detector = null;
    

        abstract public Vector2 CalculateNewVelocity(Agent agent, List<Agent> agents, out bool velChanged);
        abstract public void DrawDebug(Agent agent, List<Agent> agents);


        protected List<VelocityObstacle> CalculateVelocityObstacles(Agent agentA, List<Agent> agents, float rvoAlpha = 1f)
        {
            List<VelocityObstacle> vos = new();
            
            // Exclude self
            foreach (Agent agentB in agents.Where(b => b != agentA))
            {
                Vector2 direction_BA = (agentB.Position - agentA.Position).normalized;
                float dist_BA = Vector2.Distance(agentA.Position, agentB.Position);
                float rad = agentA.Radius + agentB.Radius;

                Vector2 vo_apex = agentB.Velocity * rvoAlpha + agentA.Velocity * (1f - rvoAlpha) - direction_BA * dist_BA; 

                Vector2 perpendicular_BA = Vector2.Perpendicular(direction_BA);
                Vector2 bound_left = direction_BA * dist_BA + perpendicular_BA * rad;
                Vector2 bound_right = direction_BA * dist_BA - perpendicular_BA * rad;

                var vo = new VelocityObstacle(agentA, agentB, vo_apex, bound_left, bound_right, dist_BA, rad);
                vos.Add(vo);
            }
            return vos;
        }

        protected void DrawTriangle(Vector2 p1, Vector2 p2, Vector2 p3)
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

        protected bool TryGetLineIntersection(Vector2 p1, Vector2 dir1, Vector2 p2, Vector2 dir2, out Vector2 intersection)
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

        protected Vector3 Vec2To3(Vector2 vec)
        {
            return new Vector3(vec.x, 10f, vec.y);
        }
    }
}
