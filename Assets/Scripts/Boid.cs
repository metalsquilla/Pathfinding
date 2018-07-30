using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(Animator))]
[RequireComponent(typeof(NavMeshAgent))]
[RequireComponent(typeof(CapsuleCollider))]
public class Boid : MonoBehaviour {
  [HideInInspector]
  public bool OkayToStop { get; set; }

  [HideInInspector]
  public bool PathFinished {
    get {
      return (pathCorners.Count > 0 && nextCornerIndex == pathCorners.Count);
    }
  }

  [HideInInspector]
  public float RemainingDistance {
    get {
      return (pathCorners.Count == 0) ? 0f : (transform.position - pathCorners[pathCorners.Count - 1]).magnitude;
    }
  }

  private Animator animator;
  private NavMeshAgent agent;

  private Vector3 originalDestination = Vector3.zero;
  private List<Vector3> pathCorners = new List<Vector3>();
  private int nextCornerIndex = 0;

  private bool hasBeenBlocked = false;
  private float beginRepathTimer = 0;
  const float kBeginRepathTimeout = 3f;
  const float kMaxProbedPathLength = 24f;

  private Vector3 dodgeDirection = Vector3.zero;
  const float kAvoidanceForce = 1.5f;
  const float kRetractionForce = 0.2f;

  const float kCollisionCheckDist = 2f;
  const float kArrivalCheckRadius = 2f;

  // Use this for initialization
  void Start() {
    animator = GetComponent<Animator>();
    agent = GetComponent<NavMeshAgent>();

    agent.isStopped = true;
    agent.updatePosition = true;
    agent.updateRotation = false;

    if (agent.updatePosition && animator.applyRootMotion) {
      Debug.LogWarning("NavMesh Agent and Animator with Root Motion can cause race condition!");
    }
  }

  // Update is called once per frame
  void Update() {
    Navigate();
    Animate();

    if (nextCornerIndex < pathCorners.Count) {
      Debug.DrawLine(transform.position + Vector3.up,
                     pathCorners[nextCornerIndex] + Vector3.up,
                     Color.cyan, Time.deltaTime);
      for (int i = nextCornerIndex; i < pathCorners.Count - 1; i++) {
        Debug.DrawLine(pathCorners[i] + Vector3.up,
                       pathCorners[i + 1] + Vector3.up,
                       Color.cyan, Time.deltaTime);
      }
    }
    Debug.DrawRay(transform.position + Vector3.up * 0.5f, agent.velocity, Color.yellow, Time.deltaTime);
  }

  public void Stop() {
    pathCorners.Clear();
    nextCornerIndex = 0;
    agent.isStopped = true;
  }

  // TODO: handle PathInvalid cases
  public void Guide(Vector3 destination, List<Vector3> corners) {
    OkayToStop = false;
    agent.isStopped = false;

    originalDestination = destination;
    pathCorners = corners;
    nextCornerIndex = 0;
    for (int i = 1; i < corners.Count; i++) {
      Vector3 a = corners[i] - corners[i - 1];
      Vector3 b = transform.position - corners[i - 1];
      Vector3 c = transform.position - corners[i];
      float dotBA = Vector3.Dot(b, a);
      float dotCA = Vector3.Dot(c, a);
      if (dotCA < 0) {
        if (dotBA >= 0)
          nextCornerIndex = i;
        break;
      }
      nextCornerIndex = i;
    }
    // Skip to the farthest unobstructed corner
    for (NavMeshHit hit;
         nextCornerIndex + 1 < corners.Count &&
         !agent.Raycast(corners[nextCornerIndex + 1], out hit);
         nextCornerIndex++) { }
  }

  private void Navigate() {
    float max_speed = agent.speed;
    Vector3 steering = Vector3.zero;

    // Pathfinding + steering
    bool arrived = false;
    bool blocked = false;
    Vector3 avoidance = Vector3.zero;
    Vector3 attraction = Vector3.zero;
    Vector3 retraction = Vector3.zero;
    if (PathFinished) {
      Vector3 end = pathCorners[pathCorners.Count - 1];
      Vector3 offset = end - transform.position;
      retraction = kRetractionForce * offset;
      if (offset.magnitude <= kArrivalCheckRadius)
        arrived = true;
    }
    else if (nextCornerIndex < pathCorners.Count) {
      NavMeshHit navmesh_hit;
      bool navmesh_blocked = agent.Raycast(pathCorners[nextCornerIndex], out navmesh_hit);

      // Probe the path ahead of next corner if it is blocked on navmesh
      // If there's a non-blocked probe, switch current navigation target to it
      bool probe_blocked = navmesh_blocked;
      float probed_length = 0f;
      Vector3 previous_probe = pathCorners[nextCornerIndex];
      Vector3 viable_target = pathCorners[nextCornerIndex];
      for (int index = nextCornerIndex; index > 0 && probe_blocked && probed_length < kMaxProbedPathLength; index--) {
        Vector3 far = pathCorners[index];
        Vector3 near = pathCorners[index - 1];
        for (float alpha = 0f; alpha < 1f && probe_blocked && probed_length < kMaxProbedPathLength; alpha += 0.34f) {
          Vector3 probe = (1f - alpha) * far + alpha * near;
          probed_length += (probe - previous_probe).magnitude;
          previous_probe = probe;

          NavMeshHit probe_hit;
          probe_blocked = agent.Raycast(probe, out probe_hit);
          if (!probe_blocked) {
            viable_target = probe;
            navmesh_hit = probe_hit;
            navmesh_blocked = false;
          }
        }
      }

      // Avoid physical obstacle along the route
      RaycastHit raycast_hit;
      int layer_mask = LayerMask.GetMask("Obstacle");
      Vector3 origin = transform.position + agent.height * transform.up;
      Vector3 direction = (viable_target - transform.position).normalized;
      bool raycast_blocked = Physics.Raycast(origin, direction, out raycast_hit, kCollisionCheckDist, layer_mask);

      blocked = raycast_blocked || navmesh_blocked;
      if (blocked) {
        if (dodgeDirection == Vector3.zero) {
          if (raycast_blocked) {
            Vector3 reflection = Vector3.Reflect(direction, raycast_hit.normal);
            dodgeDirection += direction + reflection;
          }
          if (navmesh_blocked) {
            Vector3 reflection = Vector3.Reflect(direction, navmesh_hit.normal);
            dodgeDirection += direction + reflection;
          }
        }
        // Turn back if there's obstacle ahead
        if (Physics.Raycast(origin, transform.forward, kCollisionCheckDist, layer_mask)) {
          dodgeDirection -= transform.forward;
        }
        avoidance = kAvoidanceForce * dodgeDirection.normalized;
        Debug.DrawRay(origin, avoidance, Color.magenta, Time.deltaTime);
      }
      else {
        dodgeDirection = Vector3.zero;
        Vector3 target_offset = viable_target - transform.position;
        Vector3 desired_velocity = max_speed * target_offset.normalized;
        attraction = desired_velocity - agent.velocity;
      }

      // Update next corner
      Vector3 next_corner = pathCorners[nextCornerIndex];
      Vector3 next_offset = transform.position - next_corner;
      if (next_offset.magnitude < kArrivalCheckRadius) {
        nextCornerIndex += 1;
      }
    }

    steering += avoidance + attraction + retraction;

    // Accumulate timer if all path probes are blocked
    if (blocked) {
      if (hasBeenBlocked)
        beginRepathTimer += Time.deltaTime;
      hasBeenBlocked = true;
    }
    else {
      beginRepathTimer = 0f;
      hasBeenBlocked = false;
    }

    // Apply steering
    if (beginRepathTimer >= kBeginRepathTimeout) {
      beginRepathTimer %= kBeginRepathTimeout;
      Debug.Log("agent blocked! start repathing");

      NavMeshPath path = new NavMeshPath();
      if (agent.CalculatePath(originalDestination, path)) {
        List<Vector3> corners = Utility.UpsamplePath(path);
        Guide(originalDestination, corners);
      }
      else {
        Debug.DrawRay(transform.position, 3f * Vector3.up, Color.red, Time.deltaTime);
      }
    }
    else if (arrived || OkayToStop) {
      agent.velocity = Vector3.zero;
    }
    else {
      steering = Vector3.ClampMagnitude(steering, max_speed);
      steering = Vector3.ProjectOnPlane(steering, Vector3.up);
      agent.velocity = Vector3.ClampMagnitude(agent.velocity + steering, max_speed);
    }

    if (OkayToStop) {
      Debug.DrawRay(transform.position, 3f * Vector3.up, Color.green, Time.deltaTime);
    }
  }

  private void Animate() {
    if (agent.velocity.sqrMagnitude > 0.1f) {
      Vector3 direction = Vector3.ProjectOnPlane(agent.velocity, Vector3.up);
      float angle = Vector3.SignedAngle(transform.forward, direction, Vector3.up);
      float scale = agent.angularSpeed * Time.deltaTime / Mathf.Abs(angle);
      transform.Rotate(Vector3.up, angle * scale);
    }

    Vector3 forward_velocity = Vector3.Project(agent.velocity, transform.forward);
    float forward_speed = Vector3.Dot(forward_velocity, transform.forward);
    forward_speed = Mathf.Clamp(forward_speed, 0f, forward_speed);
    animator.SetFloat("MoveSpeed", forward_speed);
  }
}
