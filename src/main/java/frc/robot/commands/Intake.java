// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.Inshooter.Inshooter;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  private Inshooter inshooter;
  private Pivot pivot;

  private LoggedTunableNumber intakeVel = new LoggedTunableNumber("Intake/IntakingVelRPM", -1000);

  private Supplier<Rotation2d> inactivePivotAngle;

  /** Creates a new Intake. */
  public Intake(Inshooter inshooter, Pivot pivot, Supplier<Rotation2d> inactivePivotAngle) {
    this.inshooter = inshooter;
    this.pivot = pivot;
    this.inactivePivotAngle = inactivePivotAngle;

    addRequirements(inshooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inshooter.setVelocity(intakeVel.get());
    pivot.setAngle(inactivePivotAngle.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    inshooter.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
