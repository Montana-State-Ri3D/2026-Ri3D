package frc.robot.stateMachines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

public class SuperStateMachine {
  public enum SuperState {
    Default,
    Idle,
    Stow,
    Intake,
    Score,
    ClimbPrep,
    Climb,
    AutoIntake,
    AutoScore
  }

  private SuperState prevState = SuperState.Idle;
  private SuperState state = SuperState.Idle;
  private boolean newState;

  private final Drive drive;
  private final SuperStructure superStructure;

  private StateMachine currentMachine;

  private LoggerGroup group = LoggerGroup.build("SuperStateMachine");
  private LoggerEntry.Text stateLogger = group.buildString("currentState");

  public SuperStateMachine(Drive drive, SuperStructure superStructure) {
    this.drive = drive;
    this.superStructure = superStructure;
  }

  public void periodic() {
    newState = state != prevState;
    switch (state) {
      case Default:
        if (superStructure.hasCoral()) {
          state = SuperState.Stow;
        } else {
          state = SuperState.Idle;
        }
        break;
      case Idle:
        superStructure.setState(StructureState.Idle);
        drive.setState(DriveState.Controller);
        break;
      case Stow:
        superStructure.setState(StructureState.Stow);
        drive.setState(DriveState.Controller);
        break;
      case Intake:
        runStateMachine(() -> new IntakeStateMachine(this, drive, superStructure, false), newState);
        break;
      case Score:
        runStateMachine(() -> new ScoreStateMachine(this, drive, superStructure, false), newState);
        break;
      case ClimbPrep:
        superStructure.setState(StructureState.ClimbPrep);
        drive.setState(DriveState.Controller);
        break;
      case Climb:
        superStructure.setState(StructureState.Climb);
        drive.setState(DriveState.Controller);
        break;
      case AutoIntake:
        runStateMachine(() -> new IntakeStateMachine(this, drive, superStructure, true), newState);
        break;
      case AutoScore:
        runStateMachine(() -> new ScoreStateMachine(this, drive, superStructure, true), newState);
        break;
      default:
        break;
    }
    stateLogger.info(state.name());
    prevState = state;
  }

  private void runStateMachine(Supplier<StateMachine> machineSupplier, boolean newState) {
    if (newState) {
      currentMachine = machineSupplier.get();
      currentMachine.init();
    }
    currentMachine.advance();
  }

  public void setState(SuperState state) {
    this.state = state;
  }

  public SuperState getState() {
    return state;
  }

  public static Command setStateCommand(SuperStateMachine superStateMachine, SuperState state) {
    return Commands.runOnce(() -> superStateMachine.setState(state));
  }
}
