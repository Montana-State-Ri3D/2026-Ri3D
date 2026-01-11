package frc.robot.stateMachines;

import frc.lib.team2930.StateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;

public class ClimbStateMachine extends StateMachine {

  private final SuperStateMachine superStateMachine;
  private final Drive drive;
  private final SuperStructure superStructure;
  private final int totalBars = 3;
  private int currentBar = 1;

  public ClimbStateMachine(
      SuperStateMachine superStateMachine, Drive drive, SuperStructure superStructure) {
    super("ClimbStateMachine");

    this.superStateMachine = superStateMachine;
    this.drive = drive;
    this.superStructure = superStructure;

    setInitialState(stateWithName("Down", () -> down()));
  }

  private StateHandler down() {
    drive.setState(DriveState.Controller);
    superStructure.setState(StructureState.Climb);
    if (superStructure.getElevator().isAtTarget()) {
      currentBar++;
      if (currentBar > totalBars) return stateWithName("End", () -> end());
      return stateWithName("Up", () -> up());
    } else return null;
  }

  private StateHandler up() {
    drive.setState(DriveState.Controller);
    superStructure.setState(StructureState.ClimbIntermediate);
    return superStructure.getElevator().isAtTarget() ? stateWithName("Down", () -> down()) : null;
  }

  private StateHandler end() {
    superStateMachine.setState(SuperState.Idle);
    return stateWithName("Done", setDone());
  }
}
