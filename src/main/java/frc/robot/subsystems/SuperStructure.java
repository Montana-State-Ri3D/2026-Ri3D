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
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class SuperStructure extends SubsystemBase {

  public enum StructureState {
    Idle,
    Intake,
    ScorePrep,
    Score,
    ClimbPrep,
    Climb,
    ClimbIntermediate
  }

  private StructureState state = StructureState.Idle;

  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final Hopper hopper;

  private TunableNumberGroup tunableGroup =
      new TunableNumberGroup(SuperStructureConstants.ROOT_TABLE);

  private LoggedTunableNumber stowElevatorHeightInches =
      tunableGroup.build(
          "Elevator/StowHeightInches", ElevatorConstants.HOME_POSITION.in(Units.Inches));
  private LoggedTunableNumber elevatorServoActuatedPos =
      tunableGroup.build(
          "Elevator/Servo/ActuatedPos", SuperStructureConstants.ELEVATOR_SERVO_ACTUATED_POS);
  private LoggedTunableNumber elevatorServoUnactuatedPos =
      tunableGroup.build(
          "Elevator/Servo/UnActuatedPos", SuperStructureConstants.ELEVATOR_SERVO_UNACTUATED_POS);
  private LoggedTunableNumber climbPrepElevatorHeightInches =
      tunableGroup.build(
          "Elevator/ClimbPrepHeightInches",
          SuperStructureConstants.ELEVATOR_CLIMB_PREP_HEIGHT.in(Units.Inches));
  private LoggedTunableNumber climbElevatorHeightInches =
      tunableGroup.build(
          "Elevator/ClimbHeightInches",
          SuperStructureConstants.ELEVATOR_CLIMB_HEIGHT.in(Units.Inches));
  private LoggedTunableNumber climbIntermediateElevatorHeightInches =
      tunableGroup.build(
          "Elevator/IntermediateClimbHeightInches",
          SuperStructureConstants.ELEVATOR_INTERMEDIATE_CLIMB_HEIGHT.in(Units.Inches));
  private LoggedTunableNumber intakeFrontVelRPM =
      tunableGroup.build(
          "Intake/FrontVelRPM", SuperStructureConstants.INTAKE_FRONT_VEL.in(Units.RPM));
  private LoggedTunableNumber intakeBackVelRPM =
      tunableGroup.build(
          "Intake/BackVelRPM", SuperStructureConstants.INTAKE_BACK_VEL.in(Units.RPM));
  private LoggedTunableNumber shooterVelRPM =
      tunableGroup.build("Shooter/VelRPM", SuperStructureConstants.SHOOTER_VEL.in(Units.RPM));
  private LoggedTunableNumber hopperVelRPM =
      tunableGroup.build("Hopper/VelRPM", SuperStructureConstants.SHOOTER_VEL.in(Units.RPM));

  private LoggerGroup group = LoggerGroup.build(SuperStructureConstants.ROOT_TABLE);
  private LoggerEntry.Text stateLogger = group.buildString("currentState");

  /** Creates a new SuperStructure. */
  public SuperStructure(Elevator elevator, Intake intake, Shooter shooter, Hopper hopper) {
    this.elevator = elevator;
    this.intake = intake;
    this.shooter = shooter;
    this.hopper = hopper;
  }

  @Override
  public void periodic() {
    switch (state) {
      case Idle:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        elevator.setServoPositions(elevatorServoUnactuatedPos.get());
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        hopper.setVel(Units.RPM.of(0));
        shooter.setVel(Units.RPM.of(0));
        break;
      case Intake:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        elevator.setServoPositions(elevatorServoUnactuatedPos.get());
        intake.setFrontVel(Units.RPM.of(intakeFrontVelRPM.get()));
        intake.setBackVel(Units.RPM.of(intakeBackVelRPM.get()));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        hopper.setVel(Units.RPM.of(0));
        shooter.setVel(Units.RPM.of(0));
        break;
      case ScorePrep:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        elevator.setServoPositions(elevatorServoUnactuatedPos.get());
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        hopper.setVel(Units.RPM.of(0));
        shooter.setVel(Units.RPM.of(shooterVelRPM.get()));
        break;
      case Score:
        elevator.setHeight(Units.Inches.of(stowElevatorHeightInches.get()));
        elevator.setServoPositions(elevatorServoUnactuatedPos.get());
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.MAX_LENGTH);
        hopper.setVel(Units.RPM.of(hopperVelRPM.get()));
        shooter.setVel(Units.RPM.of(shooterVelRPM.get()));
        break;
      case ClimbPrep:
        elevator.setHeight(Units.Inches.of(climbPrepElevatorHeightInches.get()));
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.HOME_POSITION);
        if (intake.isExtenderAtTarget()) {
          elevator.setServoPositions(elevatorServoActuatedPos.get());
        } else {
          elevator.setServoPositions(elevatorServoUnactuatedPos.get());
        }
        hopper.setVel(Units.RPM.of(0));
        shooter.setVel(Units.RPM.of(0));
        break;
      case Climb:
        elevator.setHeight(Units.Inches.of(climbElevatorHeightInches.get()));
        elevator.setServoPositions(elevatorServoActuatedPos.get());
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.HOME_POSITION);
        hopper.setVel(Units.RPM.of(0));
        shooter.setVel(Units.RPM.of(0));
        break;
      case ClimbIntermediate:
        elevator.setHeight(Units.Inches.of(climbIntermediateElevatorHeightInches.get()));
        elevator.setServoPositions(elevatorServoActuatedPos.get());
        intake.setFrontVel(Units.RPM.of(0));
        intake.setBackVel(Units.RPM.of(0));
        intake.setExtenderPos(IntakeConstants.Extender.HOME_POSITION);
        hopper.setVel(Units.RPM.of(0));
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

  public Elevator getElevator() {
    return elevator;
  }
}
