package frc.robot.stateMachines;

import frc.lib.team2930.StateMachine;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;

public class ScoreStateMachine extends StateMachine {

  private final SuperStateMachine superStateMachine;
  private final Drive drive;
  private final SuperStructure superStructure;
  private final boolean auto;

  public ScoreStateMachine(
      SuperStateMachine superStateMachine,
      Drive drive,
      SuperStructure superStructure,
      boolean auto) {
    super("ScoreStateMachine");

    this.superStateMachine = superStateMachine;
    this.drive = drive;
    this.superStructure = superStructure;
    this.auto = auto;

    setInitialState(stateWithName("ScorePrep", () -> scorePrep()));
  }

  private StateHandler scorePrep() {
    if (auto) {
      drive.setState(DriveState.PathFollow);
    } else {
      drive.setState(DriveState.Controller);
    };
    superStructure.setState(StructureState.ScorePrep);
    return superStructure.getShooter().isAtTarget() ? stateWithName("Score", () -> score()) : null;
  }

  private StateHandler score() {
    if (auto) {
      drive.setState(DriveState.PathFollow);
    } else {
      drive.setState(DriveState.Controller);
    }
    ;
    superStructure.setState(StructureState.Score);
    return null;
  }
}
