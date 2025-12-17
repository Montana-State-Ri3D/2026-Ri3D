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

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.robot.stateMachines.SuperStateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveModules;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Drive drive;
  private final SuperStructure superStructure;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  // private final Vision vision;

  private final SuperStateMachine superStateMachine;

  private final CommandXboxController controller =
      new CommandXboxController(0); // Driver Controller

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL: // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveModules(true), new GyroIOPigeon2(), controller);
        elevator = new Elevator(new ElevatorIOReal());
        arm = new Arm(new ArmIOReal());
        intake = new Intake(new IntakeIOReal());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOLimelight(camera0Name, drive::getRotation),
        //         new VisionIOLimelight(camera1Name, drive::getRotation));
        break;

      case SIM: // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveModules(false), new GyroIO() {}, controller);
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        intake = new Intake(new IntakeIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default: // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        drive = new Drive(new DriveModules(false), new GyroIO() {}, controller);
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeIO() {});
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    superStructure = new SuperStructure(elevator, arm, intake);

    superStateMachine = new SuperStateMachine(drive, superStructure);

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
    // Reset robot rotation
    controller.start().onTrue(Commands.runOnce(() -> drive.setRotation(new Rotation2d())));

    // Score
    controller
        .rightTrigger()
        .onTrue(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Score)).onFalse(
          SuperStateMachine.setStateCommand(
              superStateMachine, SuperState.Default));

    // Intake
    controller
        .rightBumper()
        .onTrue(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Intake))
        .onFalse(
          SuperStateMachine.setStateCommand(
            superStateMachine, SuperState.Default));

    // Climb
    controller
        .leftBumper()
        .onTrue(SuperStateMachine.setStateCommand(superStateMachine, SuperState.ClimbPrep))
        .onFalse(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Climb));

    SmartDashboard.putData(
        "Brake Mode",
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.setIdleMode(IdleMode.kBrake);
              arm.setIdleMode(IdleMode.kBrake);
              intake.setIdleMode(IdleMode.kBrake);
            }));

    SmartDashboard.putData(
        "Coast Mode",
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.setIdleMode(IdleMode.kCoast);
              arm.setIdleMode(IdleMode.kCoast);
              intake.setIdleMode(IdleMode.kCoast);
            }));

    SmartDashboard.putData(
        "Zero Robot",
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.resetSensorToHomePosition();
              arm.resetSensorToHomePosition();
            }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public void robotPeriodic() {
    superStateMachine.periodic();
  }

  public void onDisabled() {
    drive.stop();
  }
}
