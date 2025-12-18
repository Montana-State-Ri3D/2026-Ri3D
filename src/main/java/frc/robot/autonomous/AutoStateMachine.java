package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.AutoDescriptor.AutoAction;
import frc.robot.stateMachines.SuperStateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
import frc.robot.subsystems.SuperStructure;

public class AutoStateMachine extends StateMachine {

  private final AutoDescriptor descriptor;
  private final SuperStateMachine stateMachine;
  private final SuperStructure superStructure;

  private int progress = 0;

  public AutoStateMachine(
      AutoDescriptor descriptor, SuperStateMachine stateMachine, SuperStructure superStructure) {
    super("AutoStateMachine");
    this.descriptor = descriptor;
    this.stateMachine = stateMachine;
    this.superStructure = superStructure;
    setInitialState(stateWithName("chooseNextState", () -> chooseNextState()));
  }

  private StateHandler chooseNextState() {
    AutoAction desiredAction = descriptor.actions().get(progress);
    SuperState desiredState = desiredAction.desiredState;
    return desiredState.equals(SuperState.AutoScore)
        ? stateWithName("Score", () -> score())
        : stateWithName("Intake", () -> intake());
  }

  private StateHandler score() {
    stateMachine.setState(SuperState.AutoScore);
    if (superStructure.hasGampiece()) {
      progress++;
      return stateWithName("Score", () -> chooseNextState());
    }
    return null;
  }

  private StateHandler intake() {
    // TODO: add intake pathing
    stateMachine.setState(SuperState.Intake);
    if (!superStructure.hasGampiece()) {
      progress++;
      return stateWithName("Intake", () -> chooseNextState());
    }
    return null;
  }

  public Pose2d initPose() {
    return Pose2d.kZero; // TODO: add logic for this.
  }
}
