using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class GameView : MonoBehaviour
{
    public Game Game;
    public GameObject PlayerView;
    [SerializeField] CellView TilePrefab;
    public TextMeshProUGUI OptimalScoreView;
    public TextMeshProUGUI CurrentScoreView;

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

        Game.BestScore = Game.CurrentScore;
        Game.CurrentScore += newCell.cost;
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

        if (Game.CurrentScore >= Game.OptimalScore)
        {
            GameOver();
            return;
        }

        Game.BestScore = Game.CurrentScore;
        OptimalScoreView.text = $"Optimal score: {Game.OptimalScore}";
        CurrentScoreView.text = $"Current weight: {Game.CurrentScore}";
    }

    void GameOver()
    {
        Game.GameIsActive = false;
        CurrentScoreView.text = Game.CurrentScore == Game.OptimalScore
            ? "Optimal solution found!"
            : $"Final score: {Game.BestScore - Game.OptimalScore}";
    }
}