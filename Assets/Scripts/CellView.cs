using UnityEngine;
using UnityEngine.UI;

public class CellView : MonoBehaviour
{
    public SpriteRenderer sprite;
    public Text text;
    
    public void SetCell(Cell cell)
    {
        text.text = cell.cost.ToString();
        sprite.color = !cell.walkable ? Color.black : cell.visited ? Color.cyan : Color.grey;
        if (cell.highlighted) sprite.color = Color.green;
    }
}