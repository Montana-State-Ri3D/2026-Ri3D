// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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

public class Arm extends SubsystemBase {
  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(ElevatorConstants.ROOT_TABLE);

  private static final LoggerEntry.Decimal logTargetAngle = logGroup.buildDecimal("targetHeight");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(ElevatorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber targetVelocityConfig = group.build("MaxVelocityConfig");
  private static final LoggedTunableNumber targetAccelerationConfig =
      group.build("TargetAccelerationConfig");

  private final LoggedTunableNumber tolerance = group.build("toleranceInches", 0.1);

  // Motion constants
  // TODO: tune constants
  static {
    if (Constants.currentMode == Mode.SIM) {
      kP.initDefault(0.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      targetVelocityConfig.initDefault(0.0);
      targetAccelerationConfig.initDefault(0.0);

    } else if (Constants.currentMode == Mode.REAL) {
      kP.initDefault(0.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      targetVelocityConfig.initDefault(0.0);
      targetAccelerationConfig.initDefault(0.0);
    }
  }

  private final ArmIO io;
  private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

  private Angle targetAngle = Units.Degree.zero();
  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new ArmSubsystem. */
  public Arm(ArmIO io) {
    this.io = io;

    configMotor();

    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Arm", inputs);

    logControlMode.info(controlMode);

    // Updating tunable numbers
    var hc = hashCode();
    if (kP.hasChanged(hc)
        || kD.hasChanged(hc)
        || kG.hasChanged(hc)
        || targetVelocityConfig.hasChanged(hc)
        || targetAccelerationConfig.hasChanged(hc)) {
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
  public void setAngle(Angle angle) {
    io.setAngle(angle);
    targetAngle = angle;
    logTargetAngle.info(targetAngle.in(Units.Degree));
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public void resetSensorToHomePosition() {
    io.setSensorPosition(Constants.ArmConstants.HOME_POSITION);
  }

  public boolean setIdleMode(IdleMode value) {
    return io.setIdleMode(value);
  }

  public void configMotor() {
    io.configMotor(
        kP.get(), kD.get(), kG.get(), targetVelocityConfig.get(), targetAccelerationConfig.get());
  }

  public void setArmManualControl(double percent) {
    setPercentOut(2 * percent + kG.get() * Math.cos(getAngle().in(Units.Radians)));
  }

  // Getters

  public boolean isAtTarget() {
    return isAtTarget(targetAngle);
  }

  public boolean isAtTarget(Angle angle) {
    return Math.abs(angle.in(Units.Degree) - inputs.angleDegrees) <= tolerance.get();
  }

  public Angle getAngle() {
    return Units.Degree.of(inputs.angleDegrees);
  }

  public Voltage getVoltage() {
    return Units.Volts.of(inputs.appliedOutput);
  }

  public LinearVelocity getVelocity() {
    return Units.InchesPerSecond.of(inputs.velocityDegreesPerSecond);
  }
}
