// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(IntakeConstants.ROOT_TABLE);

  private static final LoggerEntry.Decimal logFrontTargetVel =
      logGroup.buildDecimal("frontRollersTargetVelRPM");
  private static final LoggerEntry.Decimal logBackTargetVel =
      logGroup.buildDecimal("backRollersTargetVelRPM");
  private static final LoggerEntry.Decimal logExtenderTargetPos =
      logGroup.buildDecimal("extenderTargetPosInches");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(IntakeConstants.ROOT_TABLE);
  private static final TunableNumberGroup frontRollerGroup = group.subgroup("FrontRollers");

  private static final LoggedTunableNumber frontRollerkP = frontRollerGroup.build("kP");
  private static final LoggedTunableNumber frontRollerkV = frontRollerGroup.build("kV");

  private static final LoggedTunableNumber frontRollerMaxAccelerationConfig =
      frontRollerGroup.build("MaxAccelerationConfig");

  private final LoggedTunableNumber frontTolerance = group.build("toleranceRPM", 0.1);

  private static final TunableNumberGroup backRollerGroup = group.subgroup("BackRollers");

  private static final LoggedTunableNumber backRollerkP = backRollerGroup.build("kP");
  private static final LoggedTunableNumber backRollerkV = backRollerGroup.build("kV");

  private static final LoggedTunableNumber backRollerMaxAccelerationConfig =
      backRollerGroup.build("MaxAccelerationConfig");

  private static final TunableNumberGroup extenderGroup = group.subgroup("Extender");

  private static final LoggedTunableNumber extenderkP = extenderGroup.build("kP");
  private static final LoggedTunableNumber extenderkD = extenderGroup.build("kD");

  private final LoggedTunableNumber extenderTolerance = extenderGroup.build("toleranceInches", 0.1);

  private Distance extenderTargetPos = Units.Inches.of(0);

  // Motion constants
  // TODO: tune constants
  static {
    if (Constants.currentMode == Mode.SIM) {
      frontRollerkP.initDefault(0.0);
      frontRollerkV.initDefault(0.00208);
      frontRollerMaxAccelerationConfig.initDefault(0.0);

      backRollerkP.initDefault(0.0);
      backRollerkV.initDefault(0.00208);
      backRollerMaxAccelerationConfig.initDefault(0.0);

      extenderkP.initDefault(0.1);
      extenderkD.initDefault(0);
    } else if (Constants.currentMode == Mode.REAL) {
      frontRollerkP.initDefault(0.0);
      frontRollerkV.initDefault(0.0);
      frontRollerMaxAccelerationConfig.initDefault(0.0);

      backRollerkP.initDefault(0.0);
      backRollerkV.initDefault(0.0);
      backRollerMaxAccelerationConfig.initDefault(0.0);

      extenderkP.initDefault(0);
      extenderkD.initDefault(0);
    }
  }

  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  private AngularVelocity targetVel = Units.RPM.zero();
  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new IntakeSubsystem. */
  public Intake(IntakeIO io) {
    this.io = io;

    configFrontRollers();
    configBackRollers();
    configExtender();

    io.setFrontVoltage(0.0);
    io.setBackVoltage(0.0);
    io.setExtenderVoltage(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(IntakeConstants.ROOT_TABLE, inputs);

    logControlMode.info(controlMode);

    // Updating tunable numbers
    var hc = hashCode();
    if (frontRollerkP.hasChanged(hc)
        || frontRollerkV.hasChanged(hc)
        || frontRollerMaxAccelerationConfig.hasChanged(hc)) configFrontRollers();
    if (backRollerkP.hasChanged(hc)
        || backRollerkV.hasChanged(hc)
        || backRollerMaxAccelerationConfig.hasChanged(hc)) configBackRollers();
    if (extenderkP.hasChanged(hc) || extenderkD.hasChanged(hc)) configExtender();
  }

  // Setters

  public void setFrontPercentOut(double percent) {
    io.setFrontVoltage(percent);
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void setBackPercentOut(double percent) {
    io.setBackVoltage(percent);
    // controlMode = ControlMode.OPEN_LOOP;
  }

  public void setExtenderPercentOut(double percent) {
    io.setExtenderVoltage(percent);
    // controlMode = ControlMode.OPEN_LOOP;
  }

  public void setFrontVel(AngularVelocity vel) {
    io.setFrontVel(vel);
    targetVel = vel;
    logFrontTargetVel.info(targetVel.in(Units.RPM));
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public void setBackVel(AngularVelocity vel) {
    io.setBackVel(vel);
    targetVel = vel;
    logBackTargetVel.info(targetVel.in(Units.RPM));
    // controlMode = ControlMode.CLOSED_LOOP;
  }

  public void setExtenderPos(Distance pos) {
    if (pos.lt(Units.Inches.of(0)) || pos.gt(IntakeConstants.Extender.MAX_LENGTH)) {
      System.out.println("\u001B[33mWARNING: Intake Extender Height Clamped\u001B[0m");
      pos =
          Units.Inches.of(
              MathUtil.clamp(
                  pos.in(Units.Inches), 0, IntakeConstants.Extender.MAX_LENGTH.in(Units.Inches)));
    }
    io.setExtenderPos(pos);
    extenderTargetPos = pos;
    logExtenderTargetPos.info(extenderTargetPos.in(Units.Inches));
    // controlMode = ControlMode.CLOSED_LOOP;
  }

  public boolean setIdleMode(IdleMode value) {
    return io.setIdleMode(value);
  }

  private void configFrontRollers() {
    io.configFrontRollers(
        frontRollerkV.get(), frontRollerkP.get(), frontRollerMaxAccelerationConfig.get());
  }

  private void configBackRollers() {
    io.configBackRollers(
        backRollerkV.get(), backRollerkP.get(), backRollerMaxAccelerationConfig.get());
  }

  private void configExtender() {
    io.configExtender(extenderkP.get(), extenderkD.get());
  }

  public void resetExtenderSensorToHomePosition() {
    io.setExtenderSensorPosition(IntakeConstants.Extender.HOME_POSITION);
  }

  // Getters

  public boolean isFrontAtTarget() {
    return isFrontAtTarget(targetVel);
  }

  public boolean isFrontAtTarget(AngularVelocity vel) {
    return Math.abs(vel.in(Units.RPM) - inputs.frontRollersVelocityRPM) <= frontTolerance.get();
  }

  public Voltage getFrontVoltage() {
    return Units.Volts.of(inputs.frontRollersAppliedOutput);
  }

  public LinearVelocity getFrontVelocity() {
    return Units.InchesPerSecond.of(inputs.frontRollersVelocityRPM);
  }

  public boolean isExtenderAtTarget() {
    return isExtenderAtTarget(extenderTargetPos);
  }

  public boolean isExtenderAtTarget(Distance pos) {
    return Math.abs(pos.in(Units.Inches) - inputs.extenderPositionInches)
        <= extenderTolerance.get();
  }
}
