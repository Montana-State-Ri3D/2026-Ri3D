package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.stateMachines.SuperStateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;

public class AutoStateMachine extends StateMachine {

  private final SuperStructure superStructure;
  private final SuperStateMachine stateMachine;
  private final Drive drive;

  private Timer autoScoreTimer = new Timer();
  private TunableNumberGroup group = new TunableNumberGroup("AutoStateMachine");
  private LoggedTunableNumber tunableAutoScoreTime = group.build("autoScoreTime", 5);
  private ChoreoTrajectoryWithName[] trajectories = new ChoreoTrajectoryWithName[3];

  public AutoStateMachine(
      SuperStructure superStructure, SuperStateMachine stateMachine, Drive drive) {
    super("AutoStateMachine");
    this.superStructure = superStructure;
    this.stateMachine = stateMachine;
    this.drive = drive;
    setInitialState(stateWithName("IntakePrep", () -> intakePrep()));
    for (int i = 0; i < 3; i++)
      trajectories[i] = ChoreoTrajectoryWithName.getTrajectory("Path" + (i + 1));
  }

  private StateHandler intakePrep() {
    drive.setTrajectory(trajectories[0]);
    return stateWithName("Intake", () -> intake());
  }

  private StateHandler intake() {
    stateMachine.setState(SuperState.AutoIntake);
    if (drive.isAtTargetPose()) {
      return stateWithName("Score", () -> score());
    }
    return null;
  }

  private StateHandler score() {
    if (!autoScoreTimer.isRunning()) autoScoreTimer.start();
    stateMachine.setState(SuperState.Score);
    if (autoScoreTimer.hasElapsed(tunableAutoScoreTime.get())) {
      drive.setTrajectory(trajectories[2]);
      return stateWithName("ClimbPrep", () -> climbPrep());
    }
    return null;
  }

  private StateHandler climbPrep() {
    stateMachine.setState(SuperState.ClimbPrep);
    if (superStructure.getElevator().isAtTarget()
        && superStructure.getIntake().isExtenderAtTarget()) {
      drive.setTrajectory(trajectories[3]);
      return stateWithName("Climb", () -> climb());
    }
    return null;
  }

  private StateHandler climb() {
    if (drive.isAtTargetPose()) stateMachine.setState(SuperState.Climb);
    return null;
  }
}
