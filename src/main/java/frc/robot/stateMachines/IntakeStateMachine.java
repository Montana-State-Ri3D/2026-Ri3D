package frc.robot.stateMachines;

import frc.lib.team2930.StateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;

public class IntakeStateMachine extends StateMachine {

  private final SuperStateMachine superStateMachine;
  private final Drive drive;
  private final SuperStructure superStructure;
  private final boolean auto;

  public IntakeStateMachine(
      SuperStateMachine superStateMachine,
      Drive drive,
      SuperStructure superStructure,
      boolean auto) {
    super("IntakeStateMachine");

    this.superStateMachine = superStateMachine;
    this.drive = drive;
    this.superStructure = superStructure;
    this.auto = auto;

    setInitialState(stateWithName("Intake", () -> intake()));
  }

  private StateHandler intake() {
    if (auto) {
      drive.setState(DriveState.PathFollow);
    } else {
      drive.setState(DriveState.Controller);
    }
    ;
    superStructure.setState(StructureState.Intake);
    return superStructure.hasGampiece() ? stateWithName("End", () -> end()) : null;
  }

  private StateHandler end() {
    superStateMachine.setState(SuperState.Stow);
    return stateWithName("Done", setDone());
  }
}
