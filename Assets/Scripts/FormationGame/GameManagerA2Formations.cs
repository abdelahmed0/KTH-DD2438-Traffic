using System.Collections.Generic;
using System.Linq;
using Scripts.Game;
using UnityEngine;

namespace FormationGame
{
    public class GameManagerA2Formations : AbstractGameManager
    {
        public float penaltyTime = 0;
        public float finalTime = 0;

        private bool isComplete;
        private List<FormationGoal> m_FormationGoals;

        public override List<Goal> CreateGoals(List<GameObject> vehicles)
        {
            m_FormationGoals = mapManager.GetTargetObjects().Select(target => new FormationGoal(target)).ToList();
            return m_FormationGoals.Select(formationGoal => (Goal)formationGoal).ToList();
        }

        public void FixedUpdate()
        {
            if (!isComplete)
            {
                completionTime = goals.Max(goals => goals.CurrentTime());
                penaltyTime = (float)m_FormationGoals.Sum(goal => 2 * goal.target.PenaltyTime());
            }

            if (isComplete)
            {
                finalTime = completionTime + penaltyTime;
            }

            isComplete = goals.ToList().TrueForAll(goal => goal.CheckAchieved(null));
        }
    }
}