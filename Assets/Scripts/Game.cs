using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;

public class Game : MonoBehaviour
{
    [SerializeField] State state;
    [HideInInspector] public State goal;
    public Vector2Int startPosition;
    public Vector2Int goalPosition;
    public event Action<State> StateChanged;
    [SerializeField] int threshold;
    
    public static bool GameIsActive = true;
    public static int Threshold;
    public static int CurrentScore;
    public static int BestScore;
    public static int OptimalScore;
    
    void Start() => StartGame();

    void StartGame()
    {
        GameIsActive = true;
        state.playerPosition = startPosition;
        goal.playerPosition = goalPosition;
        goal.SetGrid(state.Grid);

        foreach (var gridCell in state.Grid.cells) if (gridCell.walkable) gridCell.cost = Random.Range(5, 30);
        
        state.Grid.GetCell(startPosition.x, startPosition.y).cost = 0;
        state.Grid.GetCell(goalPosition.x, goalPosition.y).cost = 0;

        Threshold = threshold;
        State = state;

        OptimalScore = Pathfinder.FindHighestPathWithoutExceedingThreshold(state).bestScore;
    }

    public State State
    {
        get => state;
        private set
        {
            state = value;
            StateChanged?.Invoke(value);
        }
    }

    void Update()
    {
        if (!GameIsActive) return;
        if (Input.GetKeyDown(KeyCode.LeftArrow) || Input.GetKeyDown(KeyCode.A)) Move(Vector2Int.left);
        if (Input.GetKeyDown(KeyCode.RightArrow) || Input.GetKeyDown(KeyCode.D)) Move(Vector2Int.right);
        if (Input.GetKeyDown(KeyCode.DownArrow) || Input.GetKeyDown(KeyCode.S)) Move(Vector2Int.down);
        if (Input.GetKeyDown(KeyCode.UpArrow) || Input.GetKeyDown(KeyCode.W)) Move(Vector2Int.up);
    }

    void Move(Vector2Int vec2direction)
    {
        State current = State;
        Vector2Int newPosition = current.playerPosition + vec2direction;
        if (!State.PositionExistsAndIsWalkableAndIsNotVisited(newPosition)) return;
        current.playerPosition += vec2direction;
        State = current;
    }
    
    IEnumerator Co_PlayPath(IEnumerable<State> path)
    {
        foreach (var state in path)
        {
            yield return new WaitForSeconds(0.25f);
            State = state;
        }
    }
    
    IEnumerator Co_ShowPath(IEnumerable<State> path)
    {
        foreach (var state in path)
        {
            yield return new WaitForSeconds(0.25f);
            State.Grid.GetCell(state.playerPosition.x, state.playerPosition.y).highlighted = true;
            State = State;
        }
    }

    [ContextMenu("Find Full Path")]
    public void FindPath()
    {
        var path = Pathfinder.FindHighestPathWithoutExceedingThreshold(state).states;
        StartCoroutine(Co_PlayPath(path));
    }
    
    [ContextMenu("Find Next Step")]
    public void FindNextStep()
    {
        var path = Pathfinder.FindHighestPathWithoutExceedingThreshold(state).states;
        if (path != null) State = path.ElementAt(0);
    }
    
    [ContextMenu("Show Best Path")]
    public void ShowBestPath()
    {
        var path = Pathfinder.FindHighestPathWithoutExceedingThreshold(state).states;
        StartCoroutine(Co_ShowPath(path));
    }
}