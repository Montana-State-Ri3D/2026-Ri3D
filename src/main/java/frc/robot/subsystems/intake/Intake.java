// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class Intake extends SubsystemBase {
  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(ElevatorConstants.ROOT_TABLE);

  private static final LoggerEntry.Decimal logTargetAngle = logGroup.buildDecimal("targetVelRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(ElevatorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");

  private static final LoggedTunableNumber maxAccelerationConfig =
      group.build("MaxAccelerationConfig");

  private final LoggedTunableNumber tolerance = group.build("toleranceRPM", 0.1);

  // Motion constants
  // TODO: tune constants
  static {
    if (Constants.currentMode == Mode.SIM) {
      kP.initDefault(0.0);
      kV.initDefault(0.0);

      maxAccelerationConfig.initDefault(0.0);

    } else if (Constants.currentMode == Mode.REAL) {
      kP.initDefault(0.0);
      kV.initDefault(0.0);

      maxAccelerationConfig.initDefault(0.0);
    }
  }

  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  private AngularVelocity targetVel = Units.RPM.zero();
  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new IntakeSubsystem. */
  public Intake(IntakeIO io) {
    this.io = io;

    configMotor();

    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Intake", inputs);

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

  /**
   * Sets height and elevator acceleration
   *
   * @param height
   * @param accel
   */
  public void setRPM(AngularVelocity vel) {
    io.setVel(vel);
    targetVel = vel;
    logTargetAngle.info(targetVel.in(Units.RPM));
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public boolean setIdleMode(IdleMode value) {
    return io.setIdleMode(value);
  }

  public void configMotor() {
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

  public LinearVelocity getVelocity() {
    return Units.InchesPerSecond.of(inputs.velocityRPM);
  }
}
