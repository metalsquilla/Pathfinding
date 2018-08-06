using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

public class FlowFieldManager : MonoBehaviour {
  public float AgentRadius = 0.5f;

  private Vector3 center {
    get { return transform.position; }
  }
  private Vector3 origin {
    get { return center - new Vector3(width / 2f, 0f, height / 2f); }
  }

  private Vector2Int[] clickCoords = new Vector2Int[4];

  const float kDefaultPlaneHeight = 10f;
  const float kDefaultPlaneWidth = 10f;
  private int height {
    get { return Mathf.FloorToInt(kDefaultPlaneHeight * transform.localScale.z); }
  }
  private int width {
    get { return Mathf.FloorToInt(kDefaultPlaneWidth * transform.localScale.x); }
  }

  private FlowFieldSurface surface;
  private GameObject[] units = null;

  // Use this for initialization
  void Start() {
    surface = new FlowFieldSurface(origin, height, width);

    int layer_mask = LayerMask.GetMask("Obstacle");
    for (uint y = 0u; y < height; y++) {
      for (uint x = 0u; x < width; x++) {
        Vector3 position = origin + new Vector3(x + 0.5f, 0f, y + 0.5f);
        bool walkable = !(Physics.CheckSphere(position, AgentRadius, layer_mask));
        surface.Costs[y, x] = walkable ? 0 : 1;
      }
    }

    units = GameObject.FindGameObjectsWithTag("GroundUnit");
    foreach (GameObject unit in units) {
      unit.GetComponent<FlowFieldAgent>().Surface = surface;
    }
  }

  // Update is called once per frame
  void Update() {
    CalcCursorCoords();

    if (Input.GetMouseButtonDown(0)) {
      surface.Calculate(clickCoords, units.Length);
      foreach (GameObject unit in units) {
        unit.GetComponent<FlowFieldAgent>().Stopped = false;
      }
    }

    // Stop all agents if they all have arrived
    bool stop = true;
    foreach (GameObject unit in units) {
      stop &= unit.GetComponent<FlowFieldAgent>().Arrived;
    }
    if (stop) {
      foreach (GameObject unit in units) {
        unit.GetComponent<FlowFieldAgent>().Stopped = true;
      }
    }

  }

  void OnDrawGizmos() {
    Gizmos.DrawWireCube(transform.position, new Vector3(width, 1f, height));
    if (surface != null && surface.Valid) {
      float max_potential = surface.Potentials.Cast<float>().Max();
      for (uint y = 0u; y < height; y++) {
        for (uint x = 0u; x < width; x++) {
          Vector3 position = origin + new Vector3(x + 0.5f, 0f, y + 0.5f);
          Gizmos.color = Color.red;
          if (surface.Costs[y, x] == 0)
            Gizmos.color = Color.Lerp(Color.black, Color.green, surface.Potentials[y, x] / max_potential);
          Gizmos.DrawWireCube(position, new Vector3(0.2f, 0.2f, 0.2f));
          Gizmos.color = Color.white;
          if (surface.Potentials[y, x] <= surface.ArrivalPotential)
            Gizmos.color = Color.magenta;
          Gizmos.DrawRay(position + 0.1f * Vector3.up, 0.5f * surface.Gradients[y, x]);
        }
      }
    }
    Gizmos.color = Color.yellow;
    foreach (var click in clickCoords) {
      Vector3 position = origin + new Vector3(click.x + 0.5f, 0f, click.y + 0.5f);
      if (Input.GetMouseButton(0)) {
        Gizmos.DrawCube(position, new Vector3(0.5f, 0.5f, 0.5f));
      }
      else {
        Gizmos.DrawWireCube(position, new Vector3(0.5f, 0.5f, 0.5f));
      }
    }

  }

  private void CalcCursorCoords() {
    Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
    RaycastHit hit;
    int layer_mask = LayerMask.GetMask("Terrain");
    if (Physics.Raycast(ray, out hit, 1000f, layer_mask)) {
      Vector2Int tile = surface.GetNearestTile(hit.point);
      int base_x = tile.x;
      int base_y = tile.y;
      int shift_x = base_x - 1;
      int shift_y = base_y - 1;
      // TODO: Make sure all 4 tiles are traversable
      clickCoords[0] = new Vector2Int(base_x, base_y);
      clickCoords[1] = new Vector2Int(shift_x, base_y);
      clickCoords[2] = new Vector2Int(base_x, shift_y);
      clickCoords[3] = new Vector2Int(shift_x, shift_y);
    }
  }
}
