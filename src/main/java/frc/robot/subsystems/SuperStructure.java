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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class SuperStructure extends SubsystemBase {

  public enum StructureState {
    Idle,
    Intake,
    ScorePrep,
    Score,
    ClimbPrep,
    Climb
  }

  private StructureState state = StructureState.Idle;

  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;

  private TunableNumberGroup tunableGroup =
      new TunableNumberGroup(SuperStructureConstants.ROOT_TABLE);

  private LoggedTunableNumber stowElevatorHeightInches =
      tunableGroup.build(
          "StowElevatorHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber intakeFrontVelRPM =
      tunableGroup.build(
          "IntakeFrontVelRPM", SuperStructureConstants.INTAKE_FRONT_VEL.in(Units.RPM));
  private LoggedTunableNumber intakeBackVelRPM =
      tunableGroup.build("IntakeBackVelRPM", SuperStructureConstants.INTAKE_BACK_VEL.in(Units.RPM));
  private LoggedTunableNumber shooterVelRPM =
      tunableGroup.build("ShooterVelRPM", SuperStructureConstants.SHOOTER_VEL.in(Units.RPM));
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
  public SuperStructure(Elevator elevator, Intake intake, Shooter shooter) {
    this.elevator = elevator;
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    switch (state) {
      case Idle:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        shooter.setVel(Units.RPM.of(0));
        break;
      case Intake:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        intake.setFrontVel(Units.RPM.of(intakeFrontVelRPM.get()));
        intake.setBackVel(Units.RPM.of(intakeBackVelRPM.get()));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        shooter.setVel(Units.RPM.of(0));
        break;
      case ScorePrep:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        shooter.setVel(Units.RPM.of(shooterVelRPM.get()));
        break;
      case Score:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        shooter.setVel(Units.RPM.of(shooterVelRPM.get()));
        break;
      case ClimbPrep:
        elevator.setHeight(Units.Inches.of(climbPrepElevatorHeightInches.get()));
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.HOME_POSITION);
        shooter.setVel(Units.RPM.of(0));
        break;
      case Climb:
        elevator.setHeight(Units.Inches.of(climbElevatorHeightInches.get()));
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.HOME_POSITION);
        shooter.setVel(Units.RPM.of(0));
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
