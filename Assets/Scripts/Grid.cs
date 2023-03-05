using System;
using UnityEngine;

[Serializable]
public class Cell
{
    // these are used to configure your grid:
    public bool walkable;
    public int defaultCost;
    [HideInInspector] public int cost;
    
    // These fields are used for the pathfinding process only 
    [HideInInspector] public bool visited;
}

[CreateAssetMenu]
public class Grid : ScriptableObject
{
    public Cell[] cells;
    public int width;
    public int Height => cells.Length / width;
    
    public Cell GetCell(int x, int y) => cells[x + y * width];
    public Cell GetCell(Vector2Int pos) => GetCell(pos.x, pos.y);
    bool IsCellWalkable(int x, int y) => GetCell(x, y).walkable;
    public bool IsCellWalkable(Vector2Int pos) => IsCellWalkable(pos.x, pos.y);
    
    void OnEnable()
    {
        foreach (var cell in cells)
        {
            cell.cost = cell.defaultCost;
            cell.visited = false;
        }
    }
}