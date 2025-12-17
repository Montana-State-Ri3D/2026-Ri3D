// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(ElevatorConstants.ROOT_TABLE);

  private static final LoggerEntry.Decimal logTargetHeight =
      logGroup.buildDecimal("targetHeightInches");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(ElevatorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber maxVelocityConfig = group.build("MaxVelocityConfig");
  private static final LoggedTunableNumber maxAccelerationConfig =
      group.build("MaxAccelerationConfig");

  private final LoggedTunableNumber tolerance = group.build("toleranceInches", 0.1);

  private final LoggedTunableNumber manualCoefficient =
      group.build("ManualControlCoefficient", 2.0);

  // Motion constants
  // TODO: tune constants
  static {
    if (Constants.currentMode == Mode.SIM) {
      kP.initDefault(0.2);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      maxVelocityConfig.initDefault(0.0);
      maxAccelerationConfig.initDefault(0.0);

    } else if (Constants.currentMode == Mode.REAL) {
      kP.initDefault(0.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      maxVelocityConfig.initDefault(0.0);
      maxAccelerationConfig.initDefault(0.0);
    }
  }

  private final ElevatorIO io;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  private Distance targetHeight = Units.Meters.zero();
  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    configMotor();

    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(ElevatorConstants.ROOT_TABLE, inputs);

    logControlMode.info(controlMode);

    // Updating tunable numbers
    var hc = hashCode();
    if (kP.hasChanged(hc)
        || kD.hasChanged(hc)
        || kG.hasChanged(hc)
        || maxVelocityConfig.hasChanged(hc)
        || maxAccelerationConfig.hasChanged(hc)) {
      configMotor();
    }
  }

  // Setters

  public void setPercentOut(double percent) {
    io.setVoltage(percent);
    controlMode = ControlMode.OPEN_LOOP;
  }

  /**
   * Sets height and elevator acceleration
   *
   * @param height
   * @param accel
   */
  public void setHeight(Distance height) {
    if (height.lt(Units.Inches.of(0)) || height.gt(ElevatorConstants.MAX_HEIGHT)) {
      System.out.println("\u001B[33mWARNING: Elevator Height Clamped\u001B[0m");
      height =
          Units.Inches.of(
              MathUtil.clamp(
                  height.in(Units.Inches), 0, ElevatorConstants.MAX_HEIGHT.in(Units.Inches)));
    }
    io.setHeight(height);
    targetHeight = height;
    logTargetHeight.info(targetHeight.in(Units.Inches));
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public void resetSensorToHomePosition() {
    io.setSensorPosition(Constants.ElevatorConstants.HOME_POSITION);
  }

  public boolean setIdleMode(IdleMode value) {
    return io.setIdleMode(value);
  }

  private void configMotor() {
    io.configMotor(
        kP.get(), kD.get(), kG.get(), maxVelocityConfig.get(), maxAccelerationConfig.get());
  }

  public void setElevatorManualControl(double percent) {
    setPercentOut(manualCoefficient.get() * percent + kG.get());
  }

  // Getters

  public boolean isAtTarget() {
    return isAtTarget(targetHeight);
  }

  public boolean isAtTarget(Distance height) {
    return Math.abs(height.in(Units.Inches) - inputs.heightInches) <= tolerance.get();
  }

  public Distance getHeight() {
    return Units.Inches.of(inputs.heightInches);
  }

  public Voltage getVoltage() {
    return Units.Volts.of(inputs.appliedOutput);
  }

  public LinearVelocity getVelocity() {
    return Units.InchesPerSecond.of(inputs.velocityInchesPerSecond);
  }
}
