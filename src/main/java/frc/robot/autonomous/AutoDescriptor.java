package frc.robot.autonomous;

import frc.robot.stateMachines.SuperStateMachine.SuperState;
import java.util.ArrayList;

public record AutoDescriptor(StartingLocation start, ArrayList<AutoAction> actions) {
  public enum AutoAction {
    IA(SuperState.AutoIntake), // Intake Location A
    IB(SuperState.AutoIntake), // Intake Location B
    IC(SuperState.AutoIntake), // Intake Location C
    ID(SuperState.AutoIntake), // Intake Location D
    S(SuperState.AutoIntake); // Score
    public final SuperState desiredState;

    AutoAction(SuperState desiredState) {
      this.desiredState = desiredState;
    }
  }

  public enum StartingLocation {
    S1,
    S2,
    S3
  }
}
