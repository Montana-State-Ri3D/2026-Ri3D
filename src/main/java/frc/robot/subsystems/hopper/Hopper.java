// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterConstants;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(HopperConstants.ROOT_TABLE);

  private static final LoggerEntry.Decimal logTargetAngle = logGroup.buildDecimal("targetVelRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(HopperConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");

  private static final LoggedTunableNumber maxAccelerationConfig =
      group.build("MaxAccelerationConfig");

  private final LoggedTunableNumber tolerance = group.build("toleranceRPM", 200);

  // Motion constants
  // TODO: tune constants
  static {
    if (Constants.currentMode == Mode.SIM) {
      kP.initDefault(0.0);
      kV.initDefault(0.00208);

      maxAccelerationConfig.initDefault(0.0);

    } else if (Constants.currentMode == Mode.REAL) {
      kP.initDefault(0.0);
      kV.initDefault(0.0);

      maxAccelerationConfig.initDefault(0.0);
    }
  }

  private final HopperIO io;
  private final HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();

  private AngularVelocity targetVel = Units.RPM.zero();
  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new ShooterSubsystem. */
  public Hopper(HopperIO io) {
    this.io = io;

    configMotor();

    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(HopperConstants.ROOT_TABLE, inputs);

    logControlMode.info(controlMode);

    // Updating tunable numbers
    var hc = hashCode();
    if (kP.hasChanged(hc) || kV.hasChanged(hc) || maxAccelerationConfig.hasChanged(hc)) {
      configMotor();
    }
  }

  // Setters

  public void setPercentOut(double percent) {
    io.setVoltage(percent);
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void setVel(AngularVelocity vel) {
    io.setVel(vel);
    targetVel = vel;
    logTargetAngle.info(targetVel.in(Units.RPM));
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public boolean setIdleMode(IdleMode value) {
    return io.setIdleMode(value);
  }

  private void configMotor() {
    io.configMotor(kV.get(), kP.get(), maxAccelerationConfig.get());
  }

  // Getters

  public boolean isAtTarget() {
    return isAtTarget(targetVel);
  }

  public boolean isAtTarget(AngularVelocity angle) {
    return Math.abs(angle.in(Units.RPM) - inputs.velocityRPM) <= tolerance.get();
  }

  public Voltage getVoltage() {
    return Units.Volts.of(inputs.appliedOutput);
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(inputs.velocityRPM);
  }
}
