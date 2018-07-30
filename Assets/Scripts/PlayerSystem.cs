using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Assertions;

[RequireComponent(typeof(InputSystem))]
public class PlayerSystem : MonoBehaviour {

  public Transform TargetReticle;

  private List<Squad> squads;
  private InputSystem inputSystem;

  // Use this for initialization
  void Start() {
    squads = new List<Squad>();
    var ground_units = GameObject.FindGameObjectsWithTag("GroundUnit");
    Squad squad = new Squad();
    foreach (var unit in ground_units) {
      squad.Add(unit);
    }
    squads.Add(squad);
    // for (int id = 0; id < ground_units.Length; id += Squad.MaxUnits) {
    //   Squad squad = new Squad();
    //   int end = Mathf.Min(id + Squad.MaxUnits, ground_units.Length);
    //   for (int i = id; i < end; i++)
    //     squad.Add(ground_units[i]);
    //   squads.Add(squad);
    // }
    //var flying_units = GameObject.FindGameObjectsWithTag("FlyingUnit");
    inputSystem = GetComponent<InputSystem>();

    Assert.IsNotNull(TargetReticle, "Bah!");
  }

  // Update is called once per frame
  void Update() {
    if (inputSystem.MouseLeftButtonDown && squads.Count > 0) {
      TargetReticle.position = inputSystem.MouseRayHitPosition + 0.1f * inputSystem.MouseRayHitNormal;
      TargetReticle.rotation = Quaternion.LookRotation(inputSystem.MouseRayHitNormal);
      squads[0].Dispatch(inputSystem.MouseRayHitPosition);
    }
    // if (inputSystem.MouseRightButtonDown && squads.Count > 1) {
    //   TargetReticle.position = inputSystem.MouseRayHitPosition + 0.1f * inputSystem.MouseRayHitNormal;
    //   TargetReticle.rotation = Quaternion.LookRotation(inputSystem.MouseRayHitNormal);
    //   squads[1].Dispatch(inputSystem.MouseRayHitPosition);
    // }
  }

  private void LateUpdate() {
    foreach (Squad squad in squads) {
      squad.MonitorNavigation();
    }
  }

}
