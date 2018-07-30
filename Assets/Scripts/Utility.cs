using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Utility {
  [Range(0f, 1f)] const float kDefaultPathUpsampleGranularity = 0.25f;
  [Range(0f, 100f)] const float kDefaultSingleLinkThreshold = 10f;

  public static List<Vector3> UpsamplePath(NavMeshPath path, float granularity = kDefaultPathUpsampleGranularity) {
    List<Vector3> output = new List<Vector3>();
    for (int i = 1; i < path.corners.Length; i++) {
      for (float alpha = 0f; alpha < 1f; alpha += granularity) {
        output.Add((1f - alpha) * path.corners[i - 1] + alpha * path.corners[i]);
      }
    }
    output.Add(path.corners[path.corners.Length - 1]);

    return output;
  }

  public delegate float SingleLinkMetric<T>(T a, T b);
  public static List<List<int>> SingleLink<T>(List<T> objects,
                                              SingleLinkMetric<T> metric,
                                              float threshold = kDefaultSingleLinkThreshold) {
    List<List<int>> clusters = new List<List<int>>();
    float[,] distances = new float[objects.Count, objects.Count];
    for (int i = 0; i < objects.Count; i++) {
      distances[i, i] = float.MaxValue;
      for (int j = i + 1; j < objects.Count; j++) {
        distances[i, j] = distances[j, i] = metric(objects[i], objects[j]);
      }
    }

    int[] closest = new int[objects.Count];
    for (int i = 0; i < objects.Count; i++) {
      for (int j = 0; j < objects.Count; j++) {
        if (distances[i, j] < distances[i, closest[i]])
          closest[i] = j;
      }
    }

    // TODO: To be continued...
    return clusters;
  }
}