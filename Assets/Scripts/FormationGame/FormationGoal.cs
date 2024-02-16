using Scripts.Game;
using UnityEngine;

namespace FormationGame
{
    public class FormationGoal : AbstractGoal
    {
        public readonly FormationTargetManager target;

        public FormationGoal(GameObject target)
        {
            this.target = target.GetComponent<FormationTargetManager>();
        }
        
        //TODO Need to rewrite so this stuff is managed in a interface method.
        public override bool CheckAchieved(GameObject objectToCheck)
        {
            achieved = target.isComplete;
            completionTime = Time.time;
            return achieved;
        }
    }
}