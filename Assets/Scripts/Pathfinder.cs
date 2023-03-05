using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Utils;

public static class Pathfinder
{
    public static IEnumerable<State> BreadthFirstModified(State start, State end)
    {
        // make a queue of paths
        Queue<Path> todoPaths = new();
        
        // make a dictionary for paths, using the TotalCost for that path as a key
        Dictionary<int, Path> finalizedPaths = new();
        
        // start off by making a new path out of the start state, where it considers itself visited
        var startPath = new Path(new[] { start }, new HashSet<State> { start }, 0);
        
        todoPaths.Enqueue(startPath);
        
        //!!!! update threshold when moving
        
        while (todoPaths.Count > 0)
        {
            Path currentPath = todoPaths.Dequeue();
            State currNode = currentPath.States[^1];
            
            // loop over all adjacent states
            foreach (var neighbor in currNode.GetSuccessors())
            {
                // neighbor already visited in this path, continue
                if (currentPath.VisitedStates.Contains(neighbor) || neighbor.Grid.GetCell(neighbor.playerPosition).visited) continue;
                
                // found end, append the neighbor and return path
                if (neighbor.Equals(end)) return currentPath.States.Concat(new[] { neighbor }).ToArray();

                // get the cost of the neighbor cell
                var cellCost = neighbor.Grid.GetCell(neighbor.playerPosition).cost;
                
                // create a clone clone the path as a new path
                Path newPath = new Path(currentPath.States, new HashSet<State>(currentPath.VisitedStates), currentPath.TotalCost);
                
                // append the neighbor state to the end of the new path
                newPath.States = newPath.States.Concat(new[] { neighbor }).ToArray();

                newPath.VisitedStates.Add(neighbor);
                
                // add neighbour cell's cost to path's total cost
                newPath.TotalCost += cellCost;
                
                // go to next iteration
                if (cellCost + currentPath.TotalCost > Game.Threshold) finalizedPaths.TryAdd(newPath.TotalCost, newPath);
                else todoPaths.Enqueue(newPath);
            }
        }
        // no paths left to dequeue, and we didnt find the goal.
        // loop over the finalizedPaths list, and return the path with the highest cost
        int maxCost = 0;
        foreach (var keyValuePair in finalizedPaths) maxCost = Math.Max(keyValuePair.Key, maxCost);
        return finalizedPaths[maxCost].States;
    }
    
    public static IEnumerable<State> FindHighestPathWithoutExceedingThreshold(State start)
    {
        Queue<Path> todoPaths = new();
        Dictionary<int, Path> bestPath = new();
        var startPath = new Path(new[] { start }, new HashSet<State> { start }, 0);
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
                    neighborCellCost + currentPath.TotalCost > Game.Threshold)
                    continue;
                
                if (neighborCellCost + currentPath.TotalCost == Game.Threshold) 
                    return currentPath.States.Concat(new[] { neighbor }).ToArray();
                
                foundNextNeighbor = true;
                
                Path newPath = new Path(
                    currentPath.States,
                    new HashSet<State>(currentPath.VisitedStates),
                    currentPath.TotalCost);
                
                newPath.TotalCost += neighborCellCost;
                newPath.VisitedStates.Add(neighbor);
                newPath.States = newPath.States.Concat(new[] { neighbor }).ToArray();
                
                todoPaths.Enqueue(newPath);
            }
            if (!foundNextNeighbor) bestPath[currentPath.TotalCost] = currentPath;
        }
        int maxCost = 0;
        foreach (var keyValuePair in bestPath) maxCost = Math.Max(keyValuePair.Key, maxCost);
        return bestPath[maxCost].States;
    }
    
    
    
    // public static IEnumerable<State> BreadthFirstSearchPaths(State start, State end)
    // {
    //     HashSet<State> visited = new HashSet<State> { start };
    //     Queue<State[]> todoPaths = new Queue<State[]>();
    //     Dictionary<State[], HashSet<State>> visitedDict = new();
    //
    //     todoPaths.Enqueue(new []{ start });
    //     
    //     while (todoPaths.Count > 0)
    //     {
    //         State[] currentPath = todoPaths.Dequeue();
    //         State currNode = currentPath[^1];
    //             
    //         foreach (var neighbor in currNode.GetSuccessors())
    //         {
    //             var cell = neighbor.Grid.GetCell(neighbor.playerPosition);
    //             if (neighbor.Equals(end)) return currentPath.Concat(new[] { neighbor }).ToArray();
    //             if (visited.Contains(neighbor)) continue;
    //             cell.visited = true;
    //             visited.Add(neighbor);
    //             todoPaths.Enqueue(currentPath.Concat(new[] { neighbor }).ToArray());
    //         }
    //     }
    //     return null;
    // }
    
    public static IEnumerable<State> DepthFirstSearchModified(State start, State end)
    {
        Stack<State> mainPath = new();
        Dictionary<int, Stack<State>> storedPaths = new();
        mainPath.Push(start);
        int currentCost = Game.CurrentCost;
        
        while (mainPath.Count > 0)
        {
            bool foundNextNode = false;
            foreach (var neighbor in mainPath.Peek().GetSuccessors())
            {
                var cell = neighbor.Grid.GetCell(neighbor.playerPosition);
                
                // go to the next neighbor if the cell has previously been visited
                if (cell.visited) continue;
                if (cell.cost + currentCost > Game.Threshold)
                {
                    // store a copy of the state in the dictionary with the cost to reach it as the key
                    // reverse the stack to get a copy, since it iterates over the stack to add it
                    storedPaths[cell.cost + currentCost] = new Stack<State>(mainPath.Reverse());
                    // push the state that causes the limit to be exceeded to the top of the stack copy
                    storedPaths[cell.cost + currentCost].Push(neighbor);
                    continue;
                }


                currentCost += cell.cost;
                mainPath.Push(neighbor);
                cell.visited = true;
                if (neighbor.Equals(end)) return mainPath.Reverse();

                foundNextNode = true;
                break;
            }

            if (!foundNextNode)
            {
                var poppedState = mainPath.Pop();
                var poppedCell = poppedState.Grid.GetCell(poppedState.playerPosition);
                currentCost -= poppedCell.cost;
            }
        }
        
        // get the highest cost path
        int maxWeight = 0;
        foreach (var weight in storedPaths.Keys) maxWeight = Math.Max(maxWeight, weight);
        
        // mark the last tile as visited
        var lastTile = storedPaths[maxWeight].Peek();
        lastTile.Grid.GetCell(lastTile.playerPosition).visited = true;
        
        return storedPaths[maxWeight].Reverse();
    }
    
    public static IEnumerable<State> DijkstraSearchModified(State start)
    {
        Dictionary<State, State> predecessors = new();
        Dictionary<State, int> costs = new();
        PriorityQueue<State, int> todo = new();
        todo.Enqueue(start, 0);
        costs[start] = 0;
        
        while (todo.Count > 0)
        {
            todo.TryDequeue(out var current, out var queueCosts);
            
            if (queueCosts < costs[current]) continue;
            
            foreach (var neighbour in current.GetSuccessors())
            {
                var cell = neighbour.Grid.GetCell(neighbour.playerPosition);
                var newCost = costs[current] + cell.cost;
                if (costs.ContainsKey(neighbour) && costs[neighbour] >= newCost) continue;
                predecessors[neighbour] = current;
                costs[neighbour] = newCost;
                if (newCost < Game.CurrentCost) todo.Enqueue(neighbour, newCost);
            }
        }

        Debug.Log($"hit a ded end");
        
        return null;
    }
    
    
    public static IEnumerable<State> DepthFirstSearch(State start, State end)
    {
        Stack<State> path = new Stack<State>();
        path.Push(start);

        while (path.Count > 0)
        {
            bool foundNextNode = false;
            foreach (var neighbor in path.Peek().GetSuccessors())
            {
                var cell = neighbor.Grid.GetCell(neighbor.playerPosition); 
                if (cell.visited) continue;
                path.Push(neighbor);
                cell.visited = true;
                if (neighbor.Equals(end)) return path.Reverse();

                foundNextNode = true;
                break;
            }

            if (!foundNextNode) path.Pop();
        }
        return null;
    }

    public static IEnumerable<State> BreadthFirstSearchPredecessors(State start, State end)
    {
        Dictionary<State, State> predecessors = new();
        Queue<State> todo = new();
        todo.Enqueue(start);

        // prevent potential infinite loop
        int maxloops = 500;
        while (todo.Count > 0)
        {
            if (maxloops < 1) break;
            var current = todo.Dequeue();
            Debug.Log($"evaluating state with player pos: {current.playerPosition}");
            foreach (var neighbor in current.GetSuccessors())
            {
                if (neighbor.Equals(end))
                {
                    Debug.Log($"Found the end at pos: {neighbor.playerPosition}");
                    Stack<State> path = new Stack<State>();
                    path.Push(neighbor);
                    path.Push(current);
                    while (true)
                    {
                        if (predecessors.ContainsKey(current))
                        {
                            current = predecessors[current];
                            path.Push(current);
                        }
                        else break;
                    }

                    return path;
                }

                todo.Enqueue(neighbor);
                predecessors[neighbor] = current;
            }

            maxloops--;
        }
        Debug.Log($"End not found. ");
        return null;
    }
    
    
    
    public static IEnumerable<State> DijkstraSearch(State start, State end)
    {
        Dictionary<State, State> predecessors = new Dictionary<State, State>();
        Dictionary<State, int> costs = new Dictionary<State, int>();
        PriorityQueue<State, int> todo = new PriorityQueue<State, int>();
        todo.Enqueue(start, 0);
        costs[start] = 0;

        while (todo.Count > 0)
        {
            todo.TryDequeue(out var current, out var queueCosts);
            if (current.Equals(end))    
                return BuildPath(predecessors, current);
            if(queueCosts > costs[current]) continue;
                
            foreach (var neighbour in current.GetSuccessors())
            {
                var cell = neighbour.Grid.GetCell(neighbour.playerPosition);
                var newCost = costs[current];
                newCost += cell.cost;
                if (costs.ContainsKey(neighbour) && costs[neighbour] <= newCost) continue;
                predecessors[neighbour] = current;
                cell.visited = true;
                costs[neighbour] = newCost;
                todo.Enqueue(neighbour, newCost);
            }
        }
        return null;
    }
    

    static IEnumerable<State> BuildPath(Dictionary<State,State> predecessors, State neighbor)
    {
        Stack<State> path = new Stack<State>();
        while(true)
        {
            path.Push(neighbor);
            if (!predecessors.ContainsKey(neighbor))
                break;
            neighbor = predecessors[neighbor];
        }
        return path;
    }
    
    public static IEnumerable<State> FindLargestPath(State source)
    {
        // Initialize a stack to hold the current path
        var pathStack = new Stack<State>();
        pathStack.Push(source);

        // Initialize a dictionary to keep track of visited states
        var visited = new HashSet<State>();
        visited.Add(source);

        // Initialize a dictionary to keep track of each state's predecessor
        var predecessors = new Dictionary<State, State>();
        predecessors[source] = default;

        // Initialize a dictionary to keep track of the best cost of each state
        var bestCosts = new Dictionary<State, int>();
        bestCosts[source] = 0;

        // Retrieve the threshold from the game
        var threshold = Game.Threshold;

        // Depth-first search to find the largest path
        while (pathStack.Count > 0)
        {
            var currentNode = pathStack.Peek();

            // Check if the current path exceeds the threshold
            if (bestCosts[currentNode] > threshold)
            {
                pathStack.Pop();
                continue;
            }

            // Check all unvisited neighbors
            var foundUnvisitedNeighbor = false;
            foreach (var neighbor in currentNode.GetSuccessors())
            {
                if (!visited.Contains(neighbor))
                {
                    foundUnvisitedNeighbor = true;
                    pathStack.Push(neighbor);
                    visited.Add(neighbor);
                    predecessors[neighbor] = currentNode;
                    bestCosts[neighbor] = bestCosts[currentNode] + 1;
                    break;
                }
            }

            // If no unvisited neighbors are found, backtrack
            if (!foundUnvisitedNeighbor)
            {
                pathStack.Pop();
            }
        }

        // Reconstruct the largest path from the predecessors dictionary
        var largestPath = new List<State>();
        var current = source;
        while (!current.Equals(default(State)))
        {
            Debug.Log("once");
            largestPath.Add(current);
            current = predecessors[current];
        }
        largestPath.Reverse();

        return largestPath;
    }
}