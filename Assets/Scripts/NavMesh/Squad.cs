using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Assertions;

public class Squad {
  // public uint ID { get; set; }

  private float areaOfAllUnits = 0f;
  private int requiredFinishedCount = 0;
  private float arrivalCheckDensity = 0.3f;
  private float arrivalCheckPercentage = 0.9f;

  private List<Boid> boids = new List<Boid>();
  private List<NavMeshAgent> agents = new List<NavMeshAgent>();

  public Squad(GameAIPresets presets) {
    arrivalCheckDensity = presets.SquadArrivalCheckDensity;
    arrivalCheckPercentage = presets.SquadArrivalCheckPercentage;
  }

  public void Add(GameObject unit) {
    Boid boid = unit.GetComponent<Boid>();
    NavMeshAgent agent = unit.GetComponent<NavMeshAgent>();
    Assert.IsNotNull(boid, "Boid component needed!");
    Assert.IsNotNull(agent, "NavMeshAgent component needed!");
    boids.Add(boid);
    agents.Add(agent);

    areaOfAllUnits += Mathf.PI * agent.radius * agent.radius;
    requiredFinishedCount = Mathf.CeilToInt(arrivalCheckPercentage * boids.Count);
  }

  public void MonitorNavigation() {
    int count = 0;
    float radius = float.Epsilon;
    float threshold = float.Epsilon;
    foreach (Boid boid in boids) {
      float distance = boid.RemainingDistance;
      count += boid.PathFinished ? 1 : 0;
      radius = Mathf.Max(distance, radius);
      // Optimize the threshold function if necessary
      if (boid.PathFinished || boid.OkayToStop)
        threshold = Mathf.Max(distance, threshold);
    }

    float area = Mathf.PI * radius * radius;
    float density = areaOfAllUnits / area;
    bool arrived = (count >= requiredFinishedCount) || (density >= arrivalCheckDensity);

    bool stop = true;
    foreach (Boid boid in boids) {
      // Allow boid to stop upon squad arrival
      if (boid.RemainingDistance <= threshold)
        boid.OkayToStop = arrived;
      stop &= boid.OkayToStop;
    }

    // Stop path following if all boids are okay to stop
    if (stop) {
      foreach (Boid boid in boids) {
        boid.Stop();
      }
    }
  }

  public void Dispatch(Vector3 destination) {
    if (boids.Count == 0)
      return;

    // Utility.SingleLinkMetric<NavMeshAgent> metric = (NavMeshAgent a, NavMeshAgent b) =>
    // {
    //   NavMeshHit hit;
    //   bool connected = a.Raycast(b.transform.position, out hit);
    //   Vector3 offset = a.transform.position - b.transform.position;
    //   return connected ? offset.sqrMagnitude : float.MaxValue;
    // };

    // List<List<int>> clusters = Utility.SingleLink(agents, metric, 20f);

    Vector3 centroid = Vector3.zero;
    foreach (Boid boid in boids) {
      centroid += boid.transform.position;
    }
    centroid /= boids.Count;

    NavMeshAgent leader_agent = null;
    float min_distance = float.MaxValue;
    foreach (NavMeshAgent agent in agents) {
      Vector3 position = agent.transform.position;
      float distance = (position - centroid).sqrMagnitude;
      if (distance < min_distance) {
        min_distance = distance;
        leader_agent = agent;
      }
    }

    leader_agent.enabled = true;
    NavMeshPath path = new NavMeshPath();
    if (leader_agent.CalculatePath(destination, path)) {
      List<Vector3> corners = Utility.UpsamplePath(path);

      foreach (Boid boid in boids) {
        boid.Guide(destination, corners);
      }

    }
    else {
      Debug.Log("Failed to calculate path!");
      // Or do something else
    }
    Debug.Log(path.status);
  }
}
