using UnityEngine;

[CreateAssetMenu(fileName = "GameAIPresets", menuName = "Data/GameAIPresets")]
public class GameAIPresets : ScriptableObject {
  /* Boid */
  [Range(0f, 5f)] public float BoidCollisionCheckDistance = 2f;
  [Range(0f, 5f)] public float BoidArrivalCheckRadius = 2f;
  [Range(0f, 50f)] public float BoidMaxProbingLength = 24f;
  [Range(0f, 10f)] public float BoidAvoidanceForce = 1.5f;
  [Range(0f, 10f)] public float BoidRetractionForce = 0.2f;
  [Range(0f, 10f)] public float BoidBeginRepathTimeout = 3f;

  /* Squad */
  [Range(0f, 1f)] public float SquadArrivalCheckDensity = 0.3f;
  [Range(0f, 1f)] public float SquadArrivalCheckPercentage = 0.9f;
}