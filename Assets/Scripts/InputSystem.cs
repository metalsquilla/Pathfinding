using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CameraSystem))]
public class InputSystem : MonoBehaviour {
  public float CameraMoveX { get; private set; }
  public float CameraMoveY { get; private set; }
  public float CameraZoom { get; private set; }
  
  public Vector3 MouseRayHitPosition { get; private set; }
  public Vector3 MouseRayHitNormal { get; private set; }

  public bool MouseLeftButtonDown { get; private set; }
  public bool MouseRightButtonDown { get; private set; }
  public bool MouseMiddleButtonDown { get; private set; }

  private CameraSystem cameraSystem;

  // Use this for initialization
  void Start () {
    cameraSystem = GetComponent<CameraSystem>();
	}
	
	// Update is called once per frame
	void Update () {
    CameraMoveX = Input.GetAxis("Horizontal");
    CameraMoveY = Input.GetAxis("Vertical");
    CameraZoom = Input.GetAxis("Mouse ScrollWheel");

    MouseLeftButtonDown = Input.GetMouseButtonDown(0);
    MouseRightButtonDown = Input.GetMouseButtonDown(1);
    MouseMiddleButtonDown = Input.GetMouseButtonDown(2);

    Ray ray = cameraSystem.MainCamera.ScreenPointToRay(Input.mousePosition);
    RaycastHit hit;
    if (!Physics.Raycast(ray, out hit)) {
      Debug.LogWarning("TODO: return nearest point on navmesh!");
      MouseRayHitPosition = Vector3.zero;
      MouseRayHitNormal = Vector3.zero;
    }
    else {
      MouseRayHitPosition = hit.point;
      MouseRayHitNormal = hit.normal;
    }
  }
}
