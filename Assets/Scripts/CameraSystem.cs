using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

[RequireComponent(typeof(InputSystem))]
public class CameraSystem : MonoBehaviour {
  public Camera MainCamera = null;

  public CameraPresets Presets = null;

  private InputSystem inputSystem;
  private Vector3 cameraMoveAxisY;
  private Vector3 cameraMoveAxisX;
  private Vector3 cameraZoomAxis;

  // Use this for initialization
  void Start () {
		if (!MainCamera) {
      MainCamera = Camera.main;
    }
    Assert.IsNotNull(MainCamera, "Please set main camera to CameraSystem!");
    Assert.IsNotNull(Presets, "Please assign camera presets to CameraSystem!");

    inputSystem = GetComponent<InputSystem>();
  }

  private void LateUpdate() {
    Transform camera_transform = MainCamera.gameObject.transform;

    cameraMoveAxisY = Vector3.ProjectOnPlane(camera_transform.forward, Vector3.up).normalized;
    cameraMoveAxisX = camera_transform.right;
    cameraZoomAxis = camera_transform.forward;
    float cam_move_x = inputSystem.CameraMoveX * Presets.CameraMoveSpeed;
    float cam_move_y = inputSystem.CameraMoveY * Presets.CameraMoveSpeed;
    float cam_zoom = inputSystem.CameraZoom * Presets.CameraZoomSpeed;

    Vector3 cam_translation = cameraMoveAxisY * cam_move_y + cameraMoveAxisX * cam_move_x + cameraZoomAxis * cam_zoom;
    camera_transform.Translate(cam_translation, Space.World);
  }
}
