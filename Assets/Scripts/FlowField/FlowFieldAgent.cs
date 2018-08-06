using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

[RequireComponent(typeof(CapsuleCollider))]
public class FlowFieldAgent : MonoBehaviour {
  /// <summary>Agents with larger mass are more likely to push others away</summary>
  public float Mass = 50f;

  /// <summary>Agents with higher priority are more aggressive in collision avoidance</summary>
  public float Priority = 30f;

  public float MaxSpeed = 5.5f;
  // public float MaxAngularSpeed = 120f;
  // public float MaxAcceleration = 8f;

  [HideInInspector]
  public bool Stopped { get; set; }

  [HideInInspector]
  public Vector3 Velocity { get; set; }

  [HideInInspector]
  public FlowFieldSurface Surface { get; set; }

  public Vector3 DesiredVelocity {
    get {
      if (Surface != null && Surface.Gradients != null)
        return MaxSpeed * Surface.GetTileGradient(transform.position);
      return Vector3.zero;
    }
  }

  public bool Arrived {
    get {
      bool close = Surface.GetTilePotential(transform.position) <= Surface.ArrivalPotential;
      return Surface.IsWalkable(transform.position) && close;
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

  const int kNumberTestSpeeds = 3;
  const int kNumberTestDirections = 30;
  const int kNumberTestVelocities = kNumberTestSpeeds * kNumberTestDirections;
  Vector3[] testVelocities = new Vector3[kNumberTestVelocities];

  void Awake() {
    capsule = GetComponent<CapsuleCollider>();
    for (int i = 0; i < kNumberTestDirections; i++) {
      float angle = 2f * Mathf.PI * i / kNumberTestDirections;
      Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
      for (int j = 0; j < kNumberTestSpeeds; j++) {
        Vector3 velocity = direction * Random.Range(0f, MaxSpeed);
        testVelocities[i * kNumberTestSpeeds + j] = velocity;
      }
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

    Velocity = CalcRVOVelocity();
    // Velocity = CalcFlockingVelocity();

    if (!Stopped) {
      Vector3 next_position = transform.position + Velocity * Time.deltaTime;
      if (Surface.IsWalkable(next_position))
        transform.position = next_position;
      else Velocity = Vector3.zero;
    }
  }

  void OnDrawGizmos() {
    Gizmos.color = Color.yellow;
    Gizmos.DrawRay(transform.position, Velocity);
    Gizmos.color = Color.green;
    Gizmos.DrawRay(transform.position, DesiredVelocity);
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
      // TODO: this is too simple, too naive!
      if (!Surface.IsWalkable(probe)) {
        avoidance += kAvoidanceForce * (-Velocity.normalized) / Velocity.sqrMagnitude;
        break;
      }
    }
    steering += avoidance;

    return Vector3.ClampMagnitude(Velocity + steering, MaxSpeed);
  }


  private Vector3 CalcRVOVelocity() {
    float priority = Priority;
    Vector3 desired_velocity = DesiredVelocity;
    // TODO: need polishing...
    if (Arrived) {
      float potential = Surface.GetTilePotential(transform.position);
      float slow_down = potential / Surface.ArrivalPotential;
      desired_velocity *= slow_down;
      priority *= 5f;
    }

    Vector3 best_velocity = Velocity;

    // Do RVO avoidance if agent is running
    if (!Stopped) {
      float min_penalty = float.MaxValue;

      foreach (Vector3 test_velocity in testVelocities) {
        Vector3 direction = test_velocity.normalized;

        float shortest_collision_enter_time = float.MaxValue - 1f;
        float longest_collision_escape_time = 0f;
        // Check collision against dynamic agents
        foreach (FlowFieldAgent neighbor in neighborAgents) {
          var result = TestCollision(neighbor, test_velocity);
          bool is_colliding = result.Key;
          float time = result.Value;
          if (is_colliding) {
            longest_collision_escape_time = Mathf.Max(time, longest_collision_escape_time);
          }
          else {
            shortest_collision_enter_time = Mathf.Min(time, shortest_collision_enter_time);
          }

        }
        // Check collision against static obstacles
        bool is_inside_obstacle = !Surface.IsWalkable(transform.position);
        for (float length = ObstacleCheckStep; length < MaxObstacleDistance; length += ObstacleCheckStep) {
          Vector3 probe = transform.position + length * direction;
          float time = length / test_velocity.magnitude;

          if (is_inside_obstacle && Surface.IsWalkable(probe)) {
            longest_collision_escape_time = Mathf.Max(time, longest_collision_escape_time);
            break;
          }
          else if (!is_inside_obstacle && !Surface.IsWalkable(probe)) {
            shortest_collision_enter_time = Mathf.Min(time, shortest_collision_enter_time);
            break;
          }
        }

        float seek_penalty = (test_velocity - desired_velocity).magnitude;
        float avoid_penalty = priority / shortest_collision_enter_time;
        float escape_penalty = priority * longest_collision_escape_time;
        float penalty = seek_penalty + avoid_penalty + escape_penalty;
        if (penalty < min_penalty) {
          min_penalty = penalty;
          best_velocity = test_velocity;
        }
      }
    }

    return best_velocity;
  }

  /// <summary>return result is a pair of {is_colliding, min_positive_time}</summary>
  private KeyValuePair<bool, float> TestCollision(FlowFieldAgent other, Vector3 vTest) {
    float ma = Mass;
    float mb = other.Mass;
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
      return new KeyValuePair<bool, float>(false, infinity);
    }
    float delta = Mathf.Sqrt(discriminant);
    float t1 = (-b - delta) / (2 * a);
    float t2 = (-b + delta) / (2 * a);

    bool is_colliding = (t1 <= 0f && t2 > 0f);
    float min_positive = infinity;      // won't collide, minimize collision penalty
    if (t1 > 0f) min_positive = t1;     // will collide, return time of collision enter
    else if (t2 > 0f) min_positive = t2;// is colliding, return time of collision escape

    return new KeyValuePair<bool, float>(is_colliding, min_positive);
  }
}