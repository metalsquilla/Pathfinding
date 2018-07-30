using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Assertions;

[RequireComponent(typeof(Animator))]
[RequireComponent(typeof(NavMeshAgent))]
[RequireComponent(typeof(CharacterController))]
public class Boid : MonoBehaviour {
  [HideInInspector]
  public uint SquadID { get; set; }

  [HideInInspector]
  public uint UnitID { get; set; }

  [HideInInspector]
  public bool OkayToStop { get; set; }

  [HideInInspector]
  public bool PathFinished {
    get { return (pathCorners.Count > 0 && nextCornerIndex == pathCorners.Count); }
  }

  [HideInInspector]
  public float RemainingDistance {
    get { return (pathCorners.Count == 0) ? 0f : (transform.position - pathCorners.Last()).magnitude; }
  }

  public bool Verbose = false;

  public GameAIPresets AIPresets;

  private Animator animator;
  private NavMeshAgent agent;

  private Vector3 originalDestination = Vector3.zero;
  private List<Vector3> pathCorners = new List<Vector3>();
  private int nextCornerIndex = 0;

  private bool isLeftHanded = false;
  private bool hasBeenBlocked = false;
  private float beginRepathTimer = 0;

  // Use this for initialization
  void Start() {
    Assert.IsNotNull(AIPresets, "GameAIPresets needed!");

    animator = GetComponent<Animator>();
    agent = GetComponent<NavMeshAgent>();

    agent.isStopped = true;
    agent.updatePosition = false;
    agent.updateRotation = false;

    isLeftHanded = Random.Range(0f, 1f) > 0.5f;

    if (agent.updatePosition && animator.applyRootMotion) {
      Debug.LogWarning("NavMesh Agent and Animator with Root Motion can cause race condition!");
    }
  }

  // Update is called once per frame
  void Update() {
    Navigate();
    Animate();

    if (Verbose) {
      if (nextCornerIndex < pathCorners.Count) {
        Debug.DrawLine(transform.position + Vector3.up,
                       pathCorners[nextCornerIndex] + Vector3.up,
                       Color.cyan, Time.deltaTime);
        for (int i = nextCornerIndex; i < pathCorners.Count - 1; i++) {
          Debug.DrawLine(pathCorners[i] + Vector3.up,
                         pathCorners[i + 1] + Vector3.up,
                         Color.cyan, Time.deltaTime);
          Debug.DrawRay(pathCorners[i], 3f * Vector3.up, Color.red, Time.deltaTime);
        }
      }
      Debug.DrawRay(transform.position + Vector3.up * 0.5f, agent.velocity, Color.yellow, Time.deltaTime);
    }

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
    Vector3 steering = Vector3.zero;

    // Pathfinding + steering
    bool arrived = false;
    bool blocked = false;
    Vector3 avoidance = Vector3.zero;
    Vector3 attraction = Vector3.zero;
    Vector3 retraction = Vector3.zero;
    if (PathFinished) {
      Vector3 offset = pathCorners.Last() - transform.position;
      retraction = AIPresets.BoidRetractionForce * offset;
      if (offset.magnitude <= AIPresets.BoidArrivalCheckRadius)
        arrived = true;
    }
    else if (nextCornerIndex < pathCorners.Count) {
      // Don't set the Agent Radius parameter in navmesh baking to be exactly what is on your agent!
      // Otherwise, you won't get correct steering target since navmesh raycast will return true if
      // the probe is on the navmesh border! So, let the radius to be a bit smaller than actual!
      NavMeshHit navmesh_hit;
      blocked = agent.Raycast(pathCorners[nextCornerIndex], out navmesh_hit);

      // Probe the path ahead of next corner if it is blocked on navmesh
      // If there's a non-blocked probe, set it as current steering target
      bool probe_blocked = blocked;
      float probed_length = 0f;
      Vector3 previous_probe = pathCorners[nextCornerIndex];
      Vector3 steering_target = pathCorners[nextCornerIndex];
      for (int index = nextCornerIndex;
           index > 0 && probe_blocked && probed_length < AIPresets.BoidMaxProbingLength;
           index--) {
        Vector3 far = pathCorners[index];
        Vector3 near = pathCorners[index - 1];
        for (float alpha = 0f;
             alpha < 1f && probe_blocked && probed_length < AIPresets.BoidMaxProbingLength;
             alpha += 0.34f) {
          Vector3 probe = (1f - alpha) * far + alpha * near;
          probed_length += (probe - previous_probe).magnitude;
          previous_probe = probe;

          NavMeshHit probe_hit;
          probe_blocked = agent.Raycast(probe, out probe_hit);
          if (!probe_blocked) {
            steering_target = probe;
            navmesh_hit = probe_hit;
            blocked = false;
          }
        }
      }

      if (blocked) {
        Vector3 desire = (navmesh_hit.position - transform.position).normalized;
        Vector3 normal = Vector3.ProjectOnPlane(navmesh_hit.normal, Vector3.up);
        bool close = navmesh_hit.distance < AIPresets.BoidCollisionCheckDistance;
        Vector3 dodge = close ? Vector3.Cross(Vector3.up, normal) : Vector3.Cross(desire, Vector3.up);
        if (isLeftHanded) dodge = -dodge;

        Vector3 ahead = transform.position + dodge * AIPresets.BoidCollisionCheckDistance;
        NavMeshHit ahead_hit;
        // Adjust dodge direction if the position ahead is also blocked
        if (agent.Raycast(ahead, out ahead_hit)) {
          desire = (ahead_hit.position - transform.position).normalized;
          dodge = Vector3.Cross(desire, Vector3.up);
          if (isLeftHanded) dodge = -dodge;
        }

        avoidance = AIPresets.BoidAvoidanceForce * dodge.normalized;
      }
      else {
        Vector3 target_offset = steering_target - transform.position;
        attraction = agent.speed * target_offset.normalized;
        // Vector3 desired_velocity = agent.speed * target_offset.normalized;
        // attraction = desired_velocity - agent.velocity;
      }

      // Update next corner
      Vector3 next_corner = pathCorners[nextCornerIndex];
      Vector3 next_offset = transform.position - next_corner;
      if (next_offset.magnitude < AIPresets.BoidArrivalCheckRadius) {
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
    if (beginRepathTimer >= AIPresets.BoidBeginRepathTimeout) {
      beginRepathTimer %= AIPresets.BoidBeginRepathTimeout;
      Debug.Log("agent blocked! start repathing");

      NavMeshPath path = new NavMeshPath();
      if (agent.CalculatePath(originalDestination, path)) {
        List<Vector3> corners = Utility.UpsamplePath(path);
        Guide(originalDestination, corners);
      }
    }
    else if (arrived || OkayToStop) {
      agent.velocity = Vector3.zero;
    }
    else {
      steering = Vector3.ClampMagnitude(steering, agent.speed);
      steering = Vector3.ProjectOnPlane(steering, Vector3.up);
      Vector3 velocity = agent.velocity + steering;
      agent.velocity = Vector3.ClampMagnitude(velocity, agent.speed);
    }

    if (OkayToStop && Verbose) {
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
    transform.position += forward_speed * transform.forward * Time.deltaTime;
    agent.nextPosition = transform.position;
    animator.SetFloat("MoveSpeed", forward_speed);
  }
}
