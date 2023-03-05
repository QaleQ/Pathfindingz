using System.Collections.Generic;

public struct Path
{
    public State[] States;
    public readonly HashSet<State> VisitedStates;
    public int PathCost;
    
    public Path(State[] states, HashSet<State> visitedStates, int pathCost)
    {
        States = states;
        VisitedStates = visitedStates;
        PathCost = pathCost;
    }
}