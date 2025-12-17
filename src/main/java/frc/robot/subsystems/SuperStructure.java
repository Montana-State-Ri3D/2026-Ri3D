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
    Score
  }

  private StructureState state = StructureState.Idle;

  private Elevator elevator;
  private Arm arm;
  private Intake intake;

  private TunableNumberGroup tunableGroup =
      new TunableNumberGroup(SuperStructureConstants.ROOT_TABLE);

  private TunableNumberGroup tunableIdleState = tunableGroup.subgroup(StructureState.Idle.name());
  private LoggedTunableNumber tunableIdleStateElevatorHeightInches =
      tunableIdleState.build(
          "ElevatorHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber tunableIdleStateArmAngleDegrees =
      tunableIdleState.build("ArmAngleDegrees", ArmConstants.HOME_POSITION.in(Units.Degree));
  private LoggedTunableNumber tunableIdleStateIntakeVelRPM =
      tunableIdleState.build("IntakeVelRPM", 0);

  private TunableNumberGroup tunableStowState = tunableGroup.subgroup(StructureState.Stow.name());
  private LoggedTunableNumber tunableStowStateElevatorHeightInches =
      tunableStowState.build(
          "ElevatorHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber tunableStowStateArmAngleDegrees =
      tunableStowState.build("ArmAngleDegrees", ArmConstants.HOME_POSITION.in(Units.Degree));
  private LoggedTunableNumber tunableStowStateIntakeVelRPM =
      tunableStowState.build("IntakeVelRPM", 0);

  private TunableNumberGroup tunableIntakeState =
      tunableGroup.subgroup(StructureState.Intake.name());
  private LoggedTunableNumber tunableIntakeStateElevatorHeightInches =
      tunableIntakeState.build(
          "ElevatorHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber tunableIntakeStateArmAngleDegrees =
      tunableIntakeState.build("ArmAngleDegrees", ArmConstants.HOME_POSITION.in(Units.Degree));
  private LoggedTunableNumber tunableIntakeStateIntakeVelRPM =
      tunableIntakeState.build("IntakeVelRPM", 1000);

  private TunableNumberGroup tunableScorePrepState =
      tunableGroup.subgroup(StructureState.ScorePrep.name());
  private LoggedTunableNumber tunableScorePrepStateElevatorHeightInches =
      tunableScorePrepState.build(
          "ElevatorHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber tunableScorePrepStateArmAngleDegrees =
      tunableScorePrepState.build("ArmAngleDegrees", ArmConstants.HOME_POSITION.in(Units.Degree));
  private LoggedTunableNumber tunableScorePrepStateIntakeVelRPM =
      tunableScorePrepState.build("IntakeVelRPM", 0);

  private TunableNumberGroup tunableScoreState = tunableGroup.subgroup(StructureState.Score.name());
  private LoggedTunableNumber tunableScoreStateElevatorHeightInches =
      tunableScoreState.build(
          "ElevatorHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber tunableScoreStateArmAngleDegrees =
      tunableScoreState.build("ArmAngleDegrees", ArmConstants.HOME_POSITION.in(Units.Degree));
  private LoggedTunableNumber tunableScoreStateIntakeVelRPM =
      tunableScoreState.build("IntakeVelRPM", 0);

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
        elevator.setHeight(Units.Inches.of(tunableIdleStateElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(tunableIdleStateArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(tunableIdleStateIntakeVelRPM.get()));
        break;
      case Stow:
        elevator.setHeight(Units.Inches.of(tunableStowStateElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(tunableStowStateArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(tunableStowStateIntakeVelRPM.get()));
        break;
      case Intake:
        elevator.setHeight(Units.Inches.of(tunableIntakeStateElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(tunableIntakeStateArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(tunableIntakeStateIntakeVelRPM.get()));
        break;
      case ScorePrep:
        elevator.setHeight(Units.Inches.of(tunableScorePrepStateElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(tunableScorePrepStateArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(tunableScorePrepStateIntakeVelRPM.get()));
        break;
      case Score:
        elevator.setHeight(Units.Inches.of(tunableScoreStateElevatorHeightInches.get()));
        arm.setAngle(Units.Degrees.of(tunableScoreStateArmAngleDegrees.get()));
        intake.setVel(Units.RPM.of(tunableScoreStateIntakeVelRPM.get()));
        break;
      default:
        break;
    }
    stateLogger.info(state.name());
  }

  public void setState(StructureState state) {
    this.state = state;
  }
}
