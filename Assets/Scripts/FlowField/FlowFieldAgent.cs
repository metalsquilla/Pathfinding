using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

[RequireComponent(typeof(CapsuleCollider))]
public class FlowFieldAgent : MonoBehaviour {
  public float Mass = 50f;
  public float Priority = 50f;
  public float MaxSpeed = 5.5f;
  public float MaxAcceleration = 8f;

  [HideInInspector]
  public bool Stopped { get; set; }

  [HideInInspector]
  public Vector3 Velocity { get; set; }

  [HideInInspector]
  public FlowFieldSurface Surface { get; set; }

  public Vector3 DesiredVelocity {
    get { return MaxSpeed * Surface.GetTileGradient(transform.position); }
  }

  public bool Arrived {
    get {
      Vector2Int tile = Surface.GetAdjacentTile(transform.position);
      bool arrived = Surface.Potentials[tile.y, tile.x] <= Surface.ArrivalPotential;
      return Surface.IsWalkable(transform.position) && arrived;
    }
  }

  private CapsuleCollider capsule;
  public float Radius { get { return capsule.radius; } }
  public float Height { get { return capsule.height; } }

  public float MaxObstacleDistance = 1f;
  public float ObstacleCheckStep = 0.5f;

  public float MaxNeighborDistance = 2f;
  public float NeighborUpdateTimeout = 0.2f;
  private float neighborUpdateTimer = 0f;
  private int neighborCheckLayerMask = 0;
  private List<FlowFieldAgent> neighborAgents;

  const int kNumberTestDirections = 60;
  Vector3[] testVelocities = new Vector3[kNumberTestDirections];

  void Awake() {
    capsule = GetComponent<CapsuleCollider>();
    for (int i = 0; i < kNumberTestDirections; i++) {
      float angle = 2f * Mathf.PI * i / kNumberTestDirections;
      testVelocities[i] = Vector3.zero;
      testVelocities[i].x = Mathf.Cos(angle);
      testVelocities[i].z = Mathf.Sin(angle);
      testVelocities[i] *= MaxSpeed;
    }
  }

  void Start() {
    neighborUpdateTimer = Random.Range(0f, NeighborUpdateTimeout);
    neighborCheckLayerMask = LayerMask.GetMask("Unit");
    neighborAgents = new List<FlowFieldAgent>();
    Stopped = true;
  }

  void Update() {

    neighborUpdateTimer += Time.deltaTime;
    if (neighborUpdateTimer >= NeighborUpdateTimeout) {
      neighborUpdateTimer %= NeighborUpdateTimeout;

      neighborAgents.Clear();
      var neighbors = Physics.OverlapSphere(transform.position, MaxNeighborDistance, neighborCheckLayerMask);
      foreach (Collider neighbor in neighbors) {
        var agent = neighbor.GetComponent<FlowFieldAgent>();
        if (agent != null && agent != this)
          neighborAgents.Add(agent);
      }
    }

    // Velocity = CalcRVOVelocity();
    Velocity = CalcFlockingVelocity();

    if (!Stopped) {
      Vector3 next_position = transform.position + Velocity * Time.deltaTime;
      if (Surface.IsWalkable(next_position))
        transform.position = next_position;
    }
  }

  void OnDrawGizmos() {
    // Gizmos.color = Color.yellow;
    // Gizmos.DrawRay(transform.position, Velocity);
    // if (Surface != null) {
    //   Gizmos.color = Color.green;
    //   Gizmos.DrawRay(transform.position, DesiredVelocity);
    // }
  }

  private Vector3 CalcFlockingVelocity() {
    Vector3 steering = DesiredVelocity - Velocity;
    if (Arrived) {
      // Auto-brake if arrived
      steering *= 0.5f;
    }

    Vector3 separation = Vector3.zero;
    const float kSeparationForce = 1.0f;
    foreach (FlowFieldAgent neighbor in neighborAgents) {
      Vector3 offset = transform.position - neighbor.transform.position;
      separation += kSeparationForce * offset.normalized / offset.sqrMagnitude;
    }
    steering += separation;

    Vector3 avoidance = Vector3.zero;
    const float kAvoidanceForce = 5.0f;
    for (float length = Radius; length < MaxObstacleDistance; length += ObstacleCheckStep) {
      Vector3 probe = transform.position + length * Velocity.normalized;
      if (!Surface.IsWalkable(probe)) {
        avoidance += kAvoidanceForce * (-Velocity.normalized) / Velocity.sqrMagnitude;
        break;
      }
    }
    steering += avoidance;

    return Vector3.ClampMagnitude(Velocity + steering, MaxSpeed);
  }

  // TODO: this is totally wrong...
  private Vector3 CalcRVOVelocity() {
    float min_penalty = float.MaxValue;
    Vector3 best_velocity = Velocity;
    // Do RVO avoidance if agent is running and has path
    if (!Stopped && DesiredVelocity.sqrMagnitude > 0) {
      foreach (Vector3 test_velocity in testVelocities) {
        Vector3 direction = test_velocity.normalized;

        float collision_time = float.MaxValue - 1f;
        // Check collision against dynamic agents
        foreach (FlowFieldAgent neighbor in neighborAgents) {
          float time = CalcCollisionTime(neighbor, test_velocity);
          collision_time = Mathf.Min(time, collision_time);
        }
        // Check collision against static obstacles
        for (float length = Radius; length < MaxObstacleDistance; length += ObstacleCheckStep) {
          Vector3 probe = transform.position + length * direction;
          if (!Surface.IsWalkable(probe)) {
            float time = length / test_velocity.magnitude;
            collision_time = Mathf.Min(time, collision_time);
            break;
          }
        }

        float penalty = (Priority / (collision_time + 0.001f)) + (test_velocity - DesiredVelocity).magnitude;
        if (penalty < min_penalty) {
          min_penalty = penalty;
          best_velocity = test_velocity;
        }
      }
    }

    return best_velocity;
  }

  private float CalcCollisionTime(FlowFieldAgent other, Vector3 vTest) {
    float ma = Priority;
    float mb = other.Priority;
    float r = Radius + other.Radius;
    Vector3 p = transform.position - other.transform.position;
    Vector3 d = (ma + mb) / mb * vTest - (ma / mb) * Velocity - other.Velocity;
    float a = d.sqrMagnitude;
    float b = 2 * Vector3.Dot(d, p);
    float c = p.sqrMagnitude - r * r;
    float discriminant = b * b - 4 * a * c;

    const float infinity = float.MaxValue - 1f;
    if (discriminant < 0) {
      // No collision if choose vTest
      return infinity;
    }
    float delta = Mathf.Sqrt(discriminant);
    float t1 = (-b - delta) / (2 * a);
    float t2 = (-b + delta) / (2 * a);
    if (t1 >= 0)
      return t1;
    else if (t2 < 0)
      return infinity;
    else return t2;
  }
}