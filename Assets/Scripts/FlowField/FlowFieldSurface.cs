using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

public class FlowFieldSurface {
  private bool[,] visited = null;
  public Vector3 Origin { get; private set; }
  public int Height { get; private set; }
  public int Width { get; private set; }

  public bool Valid { get { return Height > 0 && Width > 0; } }

  public int[,] Costs = null;
  public float[,] Potentials = null;
  public Vector3[,] Gradients = null;

  public float ArrivalPotential = 0f;

  public FlowFieldSurface(Vector3 origin, int height, int width) {
    Origin = origin;
    Height = height;
    Width = width;

    Costs = new int[Height, Width];
    Potentials = new float[Height, Width];
    Gradients = new Vector3[Height, Width];

    visited = new bool[Height, Width];
  }

  public void Calculate(Vector2Int[] goals, int unitsCount) {
    Queue<Vector2Int> queue = new Queue<Vector2Int>();
    System.Array.Clear(visited, 0, visited.Length);
    foreach (Vector2Int goal in goals) {
      if (Costs[goal.y, goal.x] == 0) {
        queue.Enqueue(goal);
        visited[goal.y, goal.x] = true;
        Potentials[goal.y, goal.x] = 0f;
      }
    }

    int counter = 0;
    while (queue.Count > 0) {
      Vector2Int tile = queue.Dequeue();
      float potential = Potentials[tile.y, tile.x];
      if (counter++ <= unitsCount) {
        ArrivalPotential = potential + 1f;
      }

      if (0 <= tile.x - 1 && Costs[tile.y, tile.x - 1] == 0 && !visited[tile.y, tile.x - 1]) {
        queue.Enqueue(new Vector2Int(tile.x - 1, tile.y));
        visited[tile.y, tile.x - 1] = true;
        Potentials[tile.y, tile.x - 1] = potential + 1f;
      }
      if (tile.x + 1 < Width && Costs[tile.y, tile.x + 1] == 0 && !visited[tile.y, tile.x + 1]) {
        queue.Enqueue(new Vector2Int(tile.x + 1, tile.y));
        visited[tile.y, tile.x + 1] = true;
        Potentials[tile.y, tile.x + 1] = potential + 1f;
      }
      if (0 <= tile.y - 1 && Costs[tile.y - 1, tile.x] == 0 && !visited[tile.y - 1, tile.x]) {
        queue.Enqueue(new Vector2Int(tile.x, tile.y - 1));
        visited[tile.y - 1, tile.x] = true;
        Potentials[tile.y - 1, tile.x] = potential + 1f;
      }
      if (tile.y + 1 < Height && Costs[tile.y + 1, tile.x] == 0 && !visited[tile.y + 1, tile.x]) {
        queue.Enqueue(new Vector2Int(tile.x, tile.y + 1));
        visited[tile.y + 1, tile.x] = true;
        Potentials[tile.y + 1, tile.x] = potential + 1f;
      }
    }

    for (int i = 0; i < Height; i++) {
      for (int j = 0; j < Width; j++) {
        if (Costs[i, j] == 0) {
          float default_potential = Potentials[i, j] + 1f;
          float up_potential = default_potential;
          float down_potential = default_potential;
          float left_potential = default_potential;
          float right_potential = default_potential;
          if (0 <= i - 1 && Costs[i - 1, j] == 0)
            up_potential = Potentials[i - 1, j];
          if (i + 1 < Height && Costs[i + 1, j] == 0)
            down_potential = Potentials[i + 1, j];
          if (0 <= j - 1 && Costs[i, j - 1] == 0)
            left_potential = Potentials[i, j - 1];
          if (j + 1 < Width && Costs[i, j + 1] == 0)
            right_potential = Potentials[i, j + 1];

          Vector3 gradient = new Vector3(left_potential - right_potential, 0f, up_potential - down_potential);
          Gradients[i, j] = gradient.normalized;
        }
      }
    }
  }

  public Vector2Int GetAdjacentTile(Vector3 position) {
    Vector3 coords = position - Origin;
    float x_m = coords.x - Mathf.Floor(coords.x);
    float y_m = coords.z - Mathf.Floor(coords.z);
    int base_x = (x_m < 0.5f) ? Mathf.FloorToInt(coords.x) : Mathf.CeilToInt(coords.x);
    int base_y = (y_m < 0.5f) ? Mathf.FloorToInt(coords.z) : Mathf.CeilToInt(coords.z);
    base_x = Mathf.Clamp(base_x, 1, Width - 1);
    base_y = Mathf.Clamp(base_y, 1, Height - 1);
    return new Vector2Int(base_x, base_y);
  }

  public Vector3 GetTileGradient(Vector3 position) {
    Vector3 offset = position - Origin;
    int tile_x = Mathf.Clamp((int)offset.x, 0, Width - 1);
    int tile_y = Mathf.Clamp((int)offset.z, 0, Height - 1);
    return Gradients[tile_y, tile_x];
  }

  public bool IsWalkable(Vector3 position) {
    Vector3 offset = position - Origin;
    int xi = (int)offset.x;
    int yi = (int)offset.z;
    int tile_x = Mathf.Clamp(xi, 0, Width - 1);
    int tile_y = Mathf.Clamp(yi, 0, Height - 1);
    return xi == tile_x && yi == tile_y && Costs[tile_y, tile_x] == 0;
  }
}