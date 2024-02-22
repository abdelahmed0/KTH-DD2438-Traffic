using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace FormationGame
{
    public class FormationTargetManager : MonoBehaviour
    {
        public bool isComplete;
        private bool isStarted;

        public double startTime;
        public double completionTime;

        private List<Renderer> m_Renderers = new();
        private List<GateTrigger> m_Triggers = new();

        public void Start()
        {
            foreach (Transform child in transform)
            {
                m_Renderers.Add(child.GetComponent<Renderer>());
                m_Triggers.Add(child.GetComponentInChildren<GateTrigger>());
            }
        }

        public void FixedUpdate()
        {
            if (!isComplete && m_Triggers.TrueForAll(trigger => trigger.hasBeenPassed))
            {
                completionTime = Time.timeAsDouble;
                isComplete = true;
                m_Renderers.ForEach(renderer => renderer.materials[2].color = Color.green);
            }

            if (!isStarted && m_Triggers.Any(trigger => trigger.hasBeenPassed))
            {
                isStarted = true;
                startTime = Time.timeAsDouble;
            }
        }

        public double PenaltyTime()
        {
            if (!isStarted) return 0.0;
            if (isComplete) return completionTime - startTime;
            return Time.time - startTime;
        }
    }
}