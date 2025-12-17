// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class SuperStructure extends SubsystemBase {

  public enum StructureState {
    Idle,
    Stow,
    Intake,
    ScorePrep,
    Score,
    ClimbPrep,
    Climb
  }

  private StructureState state = StructureState.Idle;

  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;

  private TunableNumberGroup tunableGroup =
      new TunableNumberGroup(SuperStructureConstants.ROOT_TABLE);

  private LoggedTunableNumber stowElevatorHeightInches =
      tunableGroup.build(
          "StowElevatorHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber stowArmAngleDegrees =
      tunableGroup.build("StowArmAngleDegrees", ArmConstants.HOME_POSITION.in(Units.Degree));
  private LoggedTunableNumber intakeVelRPM =
      tunableGroup.build("IntakeVelRPM", SuperStructureConstants.INTAKE_VEL.in(Units.RPM));
  private LoggedTunableNumber climbPrepElevatorHeightInches =
      tunableGroup.build(
          "ClimbPrepElevatorHeightInches",
          SuperStructureConstants.ELEVATOR_CLIMB_PREP_HEIGHT.in(Units.Inches));
  private LoggedTunableNumber climbElevatorHeightInches =
      tunableGroup.build(
          "ClimbElevatorHeightInches",
          SuperStructureConstants.ELEVATOR_CLIMB_HEIGHT.in(Units.Inches));

  private LoggerGroup group = LoggerGroup.build(SuperStructureConstants.ROOT_TABLE);
  private LoggerEntry.Text stateLogger = group.buildString("currentState");

  /** Creates a new SuperStructure. */
  public SuperStructure(Elevator elevator, Arm arm, Intake intake) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
  }

  @Override
  public void periodic() {
    switch (state) {
      case Idle:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(stowArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(0));
        break;
      case Stow:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(stowArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(0));
        break;
      case Intake:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(stowArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(intakeVelRPM.get()));
        break;
      case ScorePrep:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(stowArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(0));
        break;
      case Score:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(stowArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(0));
        break;
      case ClimbPrep:
        elevator.setHeight(Units.Inches.of(climbPrepElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(stowArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(0));
        break;
      case Climb:
        elevator.setHeight(Units.Inches.of(climbElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(stowArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(0));
        break;
      default:
        break;
    }
    stateLogger.info(state.name());
  }

  public void setState(StructureState state) {
    this.state = state;
  }

  public boolean hasCoral() {
    return false; // TODO: add
  }
}
