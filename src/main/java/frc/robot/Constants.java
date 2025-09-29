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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.Units;
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

  public static class DriveConstants {
    public static final int NUM_MODULES = 4;

    public static final LinearVelocity MAX_LINEAR_SPEED =
        Units.MetersPerSecond.of(4); // TODO: find actual value

    public static final AngularVelocity MAX_ANGULAR_SPEED = Units.RadiansPerSecond.of(7); // TODO: find actual value

    public static final double GEAR_RATIO = 16;

    public static final Distance WHEEL_RAD = Units.Inch.of(3);

    public static final Distance WHEEL_CIRC = WHEEL_RAD.times(Math.PI * 2);

    public static final SparkMaxConfig MOTOR_CONFIG(int id) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.absoluteEncoder.inverted(Constants.DriveConstants.MOTOR_INVERTS[id]);
      config.inverted(Constants.DriveConstants.MOTOR_INVERTS[id]);
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(40);
      return config;
    }

    // Location of the wheels relative to the robot center TODO: measure imperically before driving robot!
    public static final Translation2d[] WHEEL_OFFSETS = {
      new Translation2d(0.381, 0.381),
      new Translation2d(0.381, -0.381),
      new Translation2d(-0.381, 0.381),
      new Translation2d(-0.381, -0.381)
    };

    // Creating my kinematics object using the wheel locations.
    public static final MecanumDriveKinematics KINEMATICS =
        new MecanumDriveKinematics(
            DriveConstants.WHEEL_OFFSETS[0],
            DriveConstants.WHEEL_OFFSETS[1],
            DriveConstants.WHEEL_OFFSETS[2],
            DriveConstants.WHEEL_OFFSETS[3]);

    public static final boolean[] MOTOR_INVERTS = {false, false, true, true};
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

  public class CanIDs {
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 2;
    public static final int BACK_LEFT_DRIVE_CAN_ID = 3;
    public static final int BACK_RIGHT_DRIVE_CAN_ID = 4;
    public static final int[] DRIVE_CAN_IDS = {FRONT_LEFT_DRIVE_CAN_ID, FRONT_RIGHT_DRIVE_CAN_ID, BACK_LEFT_DRIVE_CAN_ID, BACK_RIGHT_DRIVE_CAN_ID};
    public static final int PIGEON_CAN_ID = 5;
  }
}
