// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private LoggedTunableNumber p = new LoggedTunableNumber("Pivot/p", 0.02);
  private LoggedTunableNumber i = new LoggedTunableNumber("Pivot/i", 0);
  private LoggedTunableNumber d = new LoggedTunableNumber("Pivot/d", 0);
  private LoggedTunableNumber f = new LoggedTunableNumber("Pivot/f", 0);

  /** Creates a new Pivot. */
  public Pivot(PivotIO io) {
    this.io = io;
    configMotor();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    int hc = hashCode();
    if (p.hasChanged(hc) || i.hasChanged(hc) || d.hasChanged(hc) || f.hasChanged(hc)) configMotor();
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setAngle(Rotation2d angle) {
    Logger.recordOutput("Pivot/TargetAngleDegrees", angle.getDegrees());
    io.setAngle(angle);
  }

  public void configMotor() {
    io.configMotor(p.get(), i.get(), d.get(), f.get());
  }

  public void zeroMotor() {
    io.resetSensorPosition(Rotation2d.kZero);
  }
}
