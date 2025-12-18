package frc.robot.stateMachines;

import frc.lib.team2930.StateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
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
      SuperStateMachine superStateMachine, Drive drive, SuperStructure superStructure, boolean auto) {
    super("ScoreStateMachine");

    this.superStateMachine = superStateMachine;
    this.drive = drive;
    this.superStructure = superStructure;
    this.auto = auto;

    setInitialState(() -> scorePrep());
  }

  private StateHandler scorePrep() {
    if(auto) {
      drive.setState(DriveState.PathFollow);
    } else { 
      drive.setState(DriveState.Controller);
    };
    superStructure.setState(StructureState.ScorePrep);
    return false ? () -> score() : null; // TODO: add condition for score
  }

  private StateHandler score() {
    if(auto) {
      drive.setState(DriveState.PathFollow);
    } else { 
      drive.setState(DriveState.Controller);
    };
    superStructure.setState(StructureState.Score);
    return superStructure.hasCoral() ? null : () -> end();
  }

  private StateHandler end() {
    superStateMachine.setState(SuperState.Idle);
    return null;
  }
}
