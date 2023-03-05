using System.Collections.Generic;
using UnityEngine;

public class GameView : MonoBehaviour
{
    public Game Game;
    public GameObject PlayerView;
    [SerializeField] CellView TilePrefab;
    
    List<CellView> _tiles = new();

    void Start()
    {
        Game.StateChanged += OnGameStateChanged; // subscribe for future changes
        OnGameStateChanged(Game.State); // update for current state
    }

    void OnGameStateChanged(State state)
    {
        PlayerView.transform.position = new Vector3(state.playerPosition.x, state.playerPosition.y, 0);
        var newCell = state.Grid.GetCell(state.playerPosition.x, state.playerPosition.y); 
        newCell.visited = true;
        
        // attempt to increase currentCost
        Game.CurrentCost += newCell.cost;
        
        // ** used to 'pick up' weight from a tile when entered
        newCell.cost = 0;
        
        // destroy all tiles
        foreach (var tile in _tiles) Destroy(tile.gameObject);
        _tiles.Clear();
        
        // create all tiles
        for (int y = 0; y < state.Grid.Height; y++)
        {
            for (int x = 0; x < state.Grid.width; x++)
            {
                var tile = Instantiate(TilePrefab, new Vector3(x, y), Quaternion.identity);
                tile.SetCell(state.Grid.GetCell(x, y));
                _tiles.Add(tile);
            }
        }
    }
}