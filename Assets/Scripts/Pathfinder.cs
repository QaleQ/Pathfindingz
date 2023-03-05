using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class Pathfinder
{
    public static (IEnumerable<State> states, int bestScore) FindHighestPathWithoutExceedingThreshold(State start)
    {
        Queue<Path> todoPaths = new();
        Dictionary<int, Path> bestPath = new();
        var startPath = new Path(new[] { start }, new HashSet<State> { start }, Game.CurrentScore);
        todoPaths.Enqueue(startPath);
        while (todoPaths.Count > 0)
        {
            Path currentPath = todoPaths.Dequeue();
            State currNode = currentPath.States[^1];
            bool foundNextNeighbor = false;
            foreach (var neighbor in currNode.GetSuccessors())
            {
                Cell neighborCell = neighbor.Grid.GetCell(neighbor.playerPosition);
                int neighborCellCost = neighborCell.cost;
                
                if (currentPath.VisitedStates.Contains(neighbor) || neighborCell.visited ||
                    neighborCellCost + currentPath.PathCost > Game.Threshold)
                    continue;
                
                if (neighborCellCost + currentPath.PathCost == Game.Threshold)
                    return (currentPath.States.Skip(1).Concat(new[] { neighbor }).ToArray(), Game.Threshold);

                foundNextNeighbor = true;
                
                Path newPath = new Path(
                    currentPath.States,
                    new HashSet<State>(currentPath.VisitedStates),
                    currentPath.PathCost);
                
                newPath.PathCost += neighborCellCost;
                newPath.VisitedStates.Add(neighbor);
                newPath.States = newPath.States.Concat(new[] { neighbor }).ToArray();

                todoPaths.Enqueue(newPath);
            }
            if (!foundNextNeighbor) bestPath[currentPath.PathCost] = currentPath;
        }
        int maxCost = 0;
        foreach (var keyValuePair in bestPath) maxCost = Math.Max(keyValuePair.Key, maxCost);
        return (bestPath[maxCost].States.Skip(1), bestPath[maxCost].PathCost);
    }
}