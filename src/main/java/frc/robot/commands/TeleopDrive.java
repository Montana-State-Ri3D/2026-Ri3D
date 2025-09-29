// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {

  private Drive drive;
  private CommandXboxController controller;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity speedX = DriveConstants.MAX_LINEAR_SPEED.times(controller.getLeftY());
    LinearVelocity speedY = DriveConstants.MAX_LINEAR_SPEED.times(-controller.getLeftX());
    if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        speedX = speedX.unaryMinus();
        speedY = speedY.unaryMinus();
    }
    drive.driveRobotCentric(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speedX,
            speedY,
            DriveConstants.MAX_ANGULAR_SPEED.times(controller.getRightX()),
            drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
