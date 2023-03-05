using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class Game : MonoBehaviour
{
    [SerializeField] State state;
    public Vector2Int startPosition;
    public Vector2Int goalPosition;
    [HideInInspector] public State goal;
    public static int Threshold;
    [SerializeField] int threshold;
    public static int CurrentCost;
    public event Action<State> StateChanged;
    
    void Start()
    {
        state.playerPosition = startPosition;
        goal.SetGrid(state.Grid);

        foreach (var gridCell in state.Grid.cells) if (gridCell.walkable) gridCell.cost = Random.Range(5, 30);
        state.Grid.GetCell(startPosition.x, startPosition.y).cost = 0;
        state.Grid.GetCell(goalPosition.x, goalPosition.y).cost = 0;
        
        Threshold = 34;
    }

    public State State
    {
        get => state;
        set
        {
            state = value;
            StateChanged?.Invoke(value);
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.LeftArrow)) Move(Vector2Int.left);
        if (Input.GetKeyDown(KeyCode.RightArrow)) Move(Vector2Int.right);
        if (Input.GetKeyDown(KeyCode.DownArrow)) Move(Vector2Int.down);
        if (Input.GetKeyDown(KeyCode.UpArrow)) Move(Vector2Int.up);
    }

    void Move(Vector2Int vec2direction)
    {
        State current = State;
        Vector2Int newPosition = current.playerPosition + vec2direction;
        if (State.PositionExistsAndIsWalkable(newPosition))
        {
            // prevent walking on previously visited
            if (!State.Grid.GetCell(newPosition).visited) current.playerPosition += vec2direction;
            // normal behavior
            // current.playerPosition += vec2direction;

            // current.currentCost += current.Grid.GetCell(current.playerPosition).cost;
        }
        State = current;
    }

    // play path
    IEnumerator Co_PlayPath(IEnumerable<State> path)
    {
        foreach (var state in path)
        {
            yield return new WaitForSeconds(0.25f);
            State = state;
        }
    }

    [ContextMenu("UpdateThreshold")]
    public void UpdateThreshold()
    {
        Threshold = threshold;
    }
    
    [ContextMenu("Depth First Search")]
    public void DepthFirstSearch()
    {
        var path = Pathfinder.DepthFirstSearch(state, goal);
        StartCoroutine(Co_PlayPath(path));
    }
    
    [ContextMenu("Depth First Search Modified")]
    public void DepthFirstSearchModified()
    {
        var path = Pathfinder.DepthFirstSearchModified(state, goal);
        if (path != null) StartCoroutine(Co_PlayPath(path));
    }

    
    [ContextMenu("Breadth First Search Paths")]
    public void BreadthFirstSearchPaths()
    {
        var path = Pathfinder.BreadthFirstModified(state, goal);
        StartCoroutine(Co_PlayPath(path));
    }
    
    [ContextMenu("SecondVersion Search")]
    public void SecondVersion()
    {
        var path = Pathfinder.FindHighestPathWithoutExceedingThreshold(state);
        StartCoroutine(Co_PlayPath(path));
    }
    
    [ContextMenu("Breadth First Search Predecessors")]
    public void BreadthFirstSearchPredecessors()
    {
        var path = Pathfinder.BreadthFirstSearchPredecessors(state, goal);
        StartCoroutine(Co_PlayPath(path));
    }
    
    
    [ContextMenu("Dijkstra Search")]
    public void DijkstraSearch()
    {
        var path = Pathfinder.DijkstraSearch(state, goal);
        StartCoroutine(Co_PlayPath(path));
    }
    
    
    [ContextMenu("Dijkstra Search Modified")]
    public void DijkstraSearchModified()
    {
        var path = Pathfinder.DijkstraSearchModified(state);
        if (path != null) StartCoroutine(Co_PlayPath(path));
    }
    
    [ContextMenu("FindLargestPath")]
    public void FindLargestPath()
    {
        var path = Pathfinder.FindLargestPath(state);
        if (path != null) StartCoroutine(Co_PlayPath(path));
    }
}