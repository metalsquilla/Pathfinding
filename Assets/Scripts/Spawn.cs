using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawn : MonoBehaviour {
  public GameObject Prefab;
  [Range(0, 99)] public int NumberOfUnits;
  [Range(1, 20)] public float RangeX;
  [Range(1, 20)] public float RangeZ;

  // Use this for initialization
  void Start() {
    Vector3 origin = transform.position;
    for (int i = 0; i < NumberOfUnits; i++) {
      float x = Random.Range(-RangeX, RangeX);
      float z = Random.Range(-RangeZ, RangeZ);
      Vector3 position = origin + new Vector3(x, 0, z);
      Instantiate(Prefab, position, Prefab.transform.rotation);
    }
  }
}
