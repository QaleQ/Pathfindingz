using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public struct State
{
    [HideInInspector] public Vector2Int playerPosition;
    [SerializeField] Grid m_grid;
    [HideInInspector] public int cost;

    public void SetGrid(Grid grid) => m_grid = grid;

    public Grid Grid => m_grid;
    
    bool PositionExists(Vector2Int newPosition)
    {
        return newPosition.x > -1 && newPosition.x < m_grid.width &&
               newPosition.y > -1 && newPosition.y < m_grid.Height;
    }
    
    public bool PositionExistsAndIsWalkable(Vector2Int newPosition)
    {
        return PositionExists(newPosition) &&
               m_grid.IsCellWalkable(newPosition);
    }

    public bool PositionExistsAndIsWalkableAndIsNotVisited(Vector2Int newPosition)
    {
        return PositionExistsAndIsWalkable(newPosition) &&
               !m_grid.GetCell(newPosition).visited;
    }
    
    public IEnumerable<State> GetSuccessors()
    {
        Vector2Int[] directions =
        {
            Vector2Int.left,
            Vector2Int.right,
            Vector2Int.down,
            Vector2Int.up
        };
        foreach (var direction in directions)
        {
            Vector2Int newPosition = playerPosition + direction;
            if (PositionExistsAndIsWalkableAndIsNotVisited(newPosition))
            {
                yield return new State
                {
                    playerPosition = newPosition,
                    m_grid = m_grid
                };
            }
        }
    }
}