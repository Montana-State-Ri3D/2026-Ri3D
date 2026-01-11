package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.autonomous.AutoDescriptor.AutoAction;
import frc.robot.stateMachines.SuperStateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;

public class AutoStateMachine extends StateMachine {

  private final AutoDescriptor descriptor;
  private final SuperStateMachine stateMachine;
  private final Drive drive;
  private final SuperStructure superStructure;

  private int progress = 0;

  private Timer autoScoreTimer = new Timer();
  private TunableNumberGroup group = new TunableNumberGroup("AutoStateMachine");
  private LoggedTunableNumber tunableAutoScoreTime = group.build("autoScoreTime", 5);

  public AutoStateMachine(
      AutoDescriptor descriptor,
      SuperStateMachine stateMachine,
      Drive drive,
      SuperStructure superStructure) {
    super("AutoStateMachine");
    this.descriptor = descriptor;
    this.stateMachine = stateMachine;
    this.drive = drive;
    this.superStructure = superStructure;
    setInitialState(stateWithName("chooseNextState", () -> chooseNextState()));
  }

  private StateHandler chooseNextState() {
    if (progress == descriptor.actions().size()) return stateWithName("Done", setDone());
    AutoAction desiredAction = descriptor.actions().get(progress);
    SuperState desiredState = desiredAction.desiredState;

    return desiredState.equals(SuperState.Score)
        ? stateWithName("Score", () -> score())
        : stateWithName("IntakePrep", () -> intakePrep());
  }

  private StateHandler score() {
    if(!autoScoreTimer.isRunning()) autoScoreTimer.start();
    stateMachine.setState(SuperState.Score);
    if (autoScoreTimer.hasElapsed(tunableAutoScoreTime.get())) {
      progress++;
      return stateWithName("ChooseNextState", () -> chooseNextState());
    }
    return null;
  }

  private StateHandler intakePrep() {
    String name = "";
    if (progress == 1) {
      name = descriptor.start().name() + "-" + descriptor.actions().get(1).name();
    } else {
      name =
          descriptor.actions().get(progress - 2).name()
              + "-"
              + descriptor.actions().get(progress).name();
    }
    drive.setTrajectory(ChoreoTrajectoryWithName.getTrajectory(name));
    return stateWithName("Intake", () -> intake());
  }

  private StateHandler intake() {
    stateMachine.setState(SuperState.AutoIntake);
    if (drive.isAtTargetPose()) {
      progress++;
      return stateWithName("ChooseNextState", () -> chooseNextState());
    }
    return null;
  }
}
