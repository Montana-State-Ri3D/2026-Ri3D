// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.LoggedTunableNumber;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Inshooter.Inshooter;
import frc.robot.subsystems.Inshooter.InshooterIO;
import frc.robot.subsystems.Inshooter.InshooterIOReal;
import frc.robot.subsystems.drive.DemoDrive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Vision vision;
  private final Inshooter inshooter;
  private final Pivot pivot;

  private final LoggedTunableNumber tunableInshooterVoltage =
      new LoggedTunableNumber("Inshooter/TunableInshooterVoltage", 0);
  private final LoggedTunableNumber tunableInshooterVelocity =
      new LoggedTunableNumber("Inshooter/TunableInshooterVelocity", 0);
  private final LoggedTunableNumber tunablePivotVoltage =
      new LoggedTunableNumber("Pivot/TunablePivotVoltage", 3);
  private final LoggedTunableNumber tunablePivotAngleDegrees =
      new LoggedTunableNumber("Pivot/TunablePivotAngleDegrees", -100);

  private LoggedTunableNumber inactivePivotAngle = new LoggedTunableNumber("Shoot/PassiveAngle", 0);

  private final DemoDrive drive = new DemoDrive(); // Demo drive subsystem, sim only
  // private final CommandGenericHID keyboard = new CommandGenericHID(0); // Keyboard 0 on port 0
  private final CommandXboxController controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        // vision =
        //     new Vision(
        //         demoDrive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
        inshooter = new Inshooter(new InshooterIOReal());
        pivot = new Pivot(new PivotIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        inshooter = new Inshooter(new InshooterIO() {});
        pivot = new Pivot(new PivotIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        inshooter = new Inshooter(new InshooterIO() {});
        pivot = new Pivot(new PivotIO() {});
        break;
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controller
        .rightBumper()
        .whileTrue(
            new Intake(inshooter, pivot, () -> Rotation2d.fromDegrees(inactivePivotAngle.get())));
    controller
        .rightTrigger()
        .whileTrue(
            new Shoot(inshooter, pivot, () -> Rotation2d.fromDegrees(inactivePivotAngle.get())));
    controller
        .a()
        .whileTrue(
            Commands.run(() -> inshooter.setVoltage(tunableInshooterVoltage.get()))
                .finallyDo(() -> inshooter.setVoltage(0)));
    controller
        .b()
        .whileTrue(
            Commands.run(() -> inshooter.setVelocity(tunableInshooterVelocity.get()))
                .finallyDo(() -> inshooter.setVoltage(0)));
    controller
        .y()
        .whileTrue(
            Commands.run(() -> pivot.setVoltage(tunablePivotVoltage.get()))
                .finallyDo(() -> pivot.setVoltage(0)));
    controller
        .x()
        .whileTrue(
            Commands.run(
                    () -> pivot.setAngle(Rotation2d.fromDegrees(tunablePivotAngleDegrees.get())))
                .finallyDo(() -> pivot.setVoltage(0)));
    controller.povDown().onTrue(Commands.runOnce(pivot::zeroMotor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
