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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.autonomous.AutoManager;
import frc.robot.stateMachines.SuperStateMachine;
import frc.robot.stateMachines.SuperStateMachine.SuperState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveModules;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.usb_vision.*;
import java.util.function.Supplier;

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
  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;
  private final USBVision vision;
  // private final Vision vision;
  private final TunableNumberGroup tempTunables = new TunableNumberGroup("TempTunables");

  private final SuperStateMachine superStateMachine;

  private final CommandXboxController controller =
      new CommandXboxController(0); // Driver Controller

  private SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<Supplier<Command>>();
  private Command autoCommand = Commands.none();
  private final AutoManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL: // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveModules(true), new GyroIOPigeon2(), controller);
        elevator = new Elevator(new ElevatorIO() {});
        intake = new Intake(new IntakeIOReal() {});
        shooter = new Shooter(new ShooterIOReal() {});
        hopper = new Hopper(new HopperIOReal() {});
        vision = new USBVision(new USBVisionIO() {}, drive::addVisionMeasurement);
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOLimelight(camera0Name, drive::getRotation),
        //         new VisionIOLimelight(camera1Name, drive::getRotation));
        break;

      case SIM: // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveModules(false), new GyroIO() {}, controller);
        elevator = new Elevator(new ElevatorIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        hopper = new Hopper(new HopperIOSim());
        vision = new USBVision(new USBVisionIO() {}, drive::addVisionMeasurement);
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
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        hopper = new Hopper(new HopperIO() {});
        vision = new USBVision(new USBVisionIO() {}, drive::addVisionMeasurement);
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    superStructure = new SuperStructure(elevator, intake, shooter, hopper, drive::getPose);

    superStateMachine = new SuperStateMachine(drive, superStructure);

    autoManager = new AutoManager(superStateMachine, drive, superStructure);

    // Configure the button bindings
    configureButtonBindings();

    configureAutoChooser();
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
        .onTrue(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Score))
        .onFalse(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Idle));

    // Intake
    controller
        .rightBumper()
        .onTrue(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Intake))
        .onFalse(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Idle));

    // Reverse
    controller
        .leftTrigger()
        .onTrue(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Reverse))
        .onFalse(SuperStateMachine.setStateCommand(superStateMachine, SuperState.Idle));

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
              intake.setIdleMode(IdleMode.kBrake);
              shooter.setIdleMode(IdleMode.kBrake);
            }));

    SmartDashboard.putData(
        "Coast Mode",
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.setIdleMode(IdleMode.kCoast);
              intake.setIdleMode(IdleMode.kCoast);
              shooter.setIdleMode(IdleMode.kCoast);
            }));

    SmartDashboard.putData(
        "Zero Robot",
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.resetSensorToHomePosition();
              intake.resetExtenderSensorToHomePosition();
            }));
  }

  private void configureAutoChooser() {
    autoChooser = autoManager.getAutos();
    autoChooser.onChange((commandSupplier) -> autoCommand = commandSupplier.get());
    SmartDashboard.putData("AutoChooser", autoChooser);
    autoCommand = autoChooser.getSelected().get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }

  public void robotPeriodic() {
    superStateMachine.periodic();
  }

  public void onDisabled() {
    drive.stop();
  }
}
