using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "CameraPresets", menuName = "Data/CameraPresets")]
public class CameraPresets : ScriptableObject {
  [Range(1.0f, 10.0f)] public float CameraMoveSpeed = 1.0f;
  [Range(1.0f, 10.0f)] public float CameraZoomSpeed = 1.0f;
}
