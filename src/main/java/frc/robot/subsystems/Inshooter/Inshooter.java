// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Inshooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;

public class Inshooter extends SubsystemBase {

  private InshooterIO io;
  private InshooterIO.Inputs inputs;

  private LoggedTunableNumber p = new LoggedTunableNumber("Inshooter/p", 0);
  private LoggedTunableNumber i = new LoggedTunableNumber("Inshooter/i", 0);
  private LoggedTunableNumber d = new LoggedTunableNumber("Inshooter/d", 0);
  private LoggedTunableNumber f = new LoggedTunableNumber("Inshooter/f", 0);

  /** Creates a new Inshooter. */
  public Inshooter(InshooterIO io) {
    this.io = io;
    configMotor();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    int hc = hashCode();
    if(p.hasChanged(hc) || i.hasChanged(hc) || d.hasChanged(hc) || f.hasChanged(hc)) configMotor();
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(double velocityRPM) {
    Logger.recordOutput("Inshooter/TargetVelocityRPM", velocityRPM);
    io.setVelocity(velocityRPM);
  }

  public void configMotor() {
    io.configMotor(p.get(), i.get(), d.get(), f.get());
  }
}
