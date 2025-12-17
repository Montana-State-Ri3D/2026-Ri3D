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

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Voltage MAX_VOLTAGE = Units.Volts.of(12);
  public static final double defaultPeriod = 0.02;

  public static class DriveConstants {
    public static final String ROOT_TABLE = "Drive";

    public static final int NUM_MODULES = 4;

    public static final LinearVelocity MAX_LINEAR_SPEED =
        Units.MetersPerSecond.of(4); // TODO: find actual value

    public static final AngularVelocity MAX_ANGULAR_SPEED =
        Units.RadiansPerSecond.of(7); // TODO: find actual value

    public static final double GEAR_RATIO = 1.0 / 9.0; // how many wheel rot per motor rotation

    public static final Distance WHEEL_RAD = Units.Inch.of(3);

    public static final Distance WHEEL_CIRC = WHEEL_RAD.times(Math.PI * 2);

    public static final double MOI = 0.00001;

    public static final SparkMaxConfig MOTOR_CONFIG(int id) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.absoluteEncoder.inverted(MOTOR_INVERTS[id]);
      config.inverted(MOTOR_INVERTS[id]);
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(40);
      config.encoder.positionConversionFactor(GEAR_RATIO);
      config.encoder.velocityConversionFactor(GEAR_RATIO);
      return config;
    }

    // Location of the wheels relative to the robot center TODO: measure imperically before driving
    // robot!
    public static final Translation2d[] WHEEL_OFFSETS = {
      new Translation2d(0.321, 0.2921),
      new Translation2d(0.321, -0.2921),
      new Translation2d(-0.321, 0.2921),
      new Translation2d(-0.321, -0.2921)
    };

    // Creating my kinematics object using the wheel locations.
    public static final MecanumDriveKinematics KINEMATICS =
        new MecanumDriveKinematics(
            DriveConstants.WHEEL_OFFSETS[0],
            DriveConstants.WHEEL_OFFSETS[1],
            DriveConstants.WHEEL_OFFSETS[2],
            DriveConstants.WHEEL_OFFSETS[3]);

    public static final boolean[] MOTOR_INVERTS = {true, false, true, false};
  }

  public class ElevatorConstants {
    public static final String ROOT_TABLE = "Elevator";
    public static final Distance HOME_POSITION = Units.Meter.of(0); // TODO: determine
    public static final double GEAR_RATIO = 1; // TODO: determine (pulley / motor)
    public static final boolean INVERT = false; // TODO: determine
    public static final boolean FOLLOWER_INVERT = false; // TODO: determine
    public static final Distance PULLEY_RADIUS = Units.Inch.of(2); // TODO: determine
    public static final double INCHES_TO_MOTOR_ROT =
        1.0 / (PULLEY_RADIUS.in(Units.Inches) * 2 * Math.PI * GEAR_RATIO);
    public static final double MOI = 0.0001;

    public static final Distance MAX_HEIGHT = Units.Inches.of(30); // TODO: determine

    public static final SparkFlexConfig MOTOR_CONFIG() {
      SparkFlexConfig config = new SparkFlexConfig();
      config.absoluteEncoder.inverted(INVERT);
      config.inverted(INVERT);
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(40);
      config.encoder.positionConversionFactor(1.0 / INCHES_TO_MOTOR_ROT);
      config.encoder.velocityConversionFactor(1.0 / INCHES_TO_MOTOR_ROT);
      config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
      return config;
    }
  }

  public class ArmConstants {
    public static final String ROOT_TABLE = "Arm";
    public static final Angle HOME_POSITION = Units.Degree.of(0); // TODO: determine
    public static final double GEAR_RATIO = 1; // TODO: determine (pulley / motor)
    public static final boolean INVERT = false; // TODO: determine
    public static final double MOI = 0.0001;
    public static final Angle MIN_ANGLE = Units.Degrees.of(0); // TODO: determine
    public static final Angle MAX_ANGLE = Units.Degrees.of(180); // TODO: determine

    public static final SparkFlexConfig MOTOR_CONFIG() {
      SparkFlexConfig config = new SparkFlexConfig();
      config.absoluteEncoder.inverted(INVERT);
      config.inverted(INVERT);
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(40);
      config.encoder.positionConversionFactor(GEAR_RATIO);
      config.encoder.velocityConversionFactor(GEAR_RATIO);
      config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
      return config;
    }
  }

  public class IntakeConstants {
    public static final String ROOT_TABLE = "Intake";
    public static final double GEAR_RATIO = 1; // TODO: determine (pulley / motor)
    public static final boolean INVERT = false; // TODO: determine
    public static final double MOI = 0.0001;

    public static final SparkFlexConfig MOTOR_CONFIG() {
      SparkFlexConfig config = new SparkFlexConfig();
      config.absoluteEncoder.inverted(INVERT);
      config.inverted(INVERT);
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(40);
      config.encoder.positionConversionFactor(GEAR_RATIO);
      config.encoder.velocityConversionFactor(GEAR_RATIO);
      return config;
    }
  }

  public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public class SuperStructureConstants {
    public static final String ROOT_TABLE = "SuperStructure";

    public static final AngularVelocity INTAKE_VEL = Units.RPM.of(1000);
    public static final Distance ELEVATOR_CLIMB_PREP_HEIGHT = Units.Inches.of(30);
    public static final Distance ELEVATOR_CLIMB_HEIGHT = Units.Inches.of(15);
  }

  public class CanIDs {
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 2;
    public static final int BACK_LEFT_DRIVE_CAN_ID = 3;
    public static final int BACK_RIGHT_DRIVE_CAN_ID = 4;
    public static final int[] DRIVE_CAN_IDS = {
      FRONT_LEFT_DRIVE_CAN_ID,
      FRONT_RIGHT_DRIVE_CAN_ID,
      BACK_LEFT_DRIVE_CAN_ID,
      BACK_RIGHT_DRIVE_CAN_ID
    };
    public static final int PIGEON_CAN_ID = 5;
    public static final int ELEVATOR_LEAD_CAN_ID = 6;
    public static final int ELEVATOR_FOLLOW_CAN_ID = 7;
    public static final int ARM_CAN_ID = 8;
    public static final int INTAKE_CAN_ID = 9;
  }
}
