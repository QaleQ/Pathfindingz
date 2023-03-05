using System.Collections.Generic;

public struct Path
{
    public State[] States;
    public readonly HashSet<State> VisitedStates;
    public int TotalCost;

    public Path(State[] states, HashSet<State> visitedStates, int totalCost)
    {
        States = states;
        VisitedStates = visitedStates;
        TotalCost = totalCost;
    }
}