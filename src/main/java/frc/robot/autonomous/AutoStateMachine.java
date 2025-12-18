package frc.robot.autonomous;

import frc.lib.team2930.StateMachine;
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
    stateMachine.setState(SuperState.Score);
    if (!superStructure.hasGampiece()) {
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
    System.out.println(name);
    drive.setTrajectory(ChoreoTrajectoryWithName.getTrajectory(name));
    return stateWithName("Intake", () -> intake());
  }

  private StateHandler intake() {
    stateMachine.setState(SuperState.AutoIntake);
    if (superStructure.hasGampiece()) {
      progress++;
      return stateWithName("ChooseNextState", () -> chooseNextState());
    }
    return null;
  }
}
