// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.usb_vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class USBVision extends SubsystemBase {
  // Logging

  // private static final LoggerGroup logGroup = LoggerGroup.build(VisionConstants.ROOT_TABLE);

  // private static final LoggerEntry.Bool logOnTarget = logGroup.buildBoolean("onTarget");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(VisionConstants.ROOT_TABLE);

  private final LoggedTunableNumber angleTolerance = group.build("angleTolerance", 0.1);

  private final USBVisionIO io;
  private final USBVisionInputsAutoLogged inputs = new USBVisionInputsAutoLogged();
  private final VisionConsumer consumer;

  /** Creates a new Vision Subsystem. */
  public USBVision(USBVisionIO io, VisionConsumer consumer) {
    this.io = io;
    this.consumer = consumer;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(VisionConstants.ROOT_TABLE, inputs);

    for (var observation : inputs.poseObservations) {
      double stdDevFactor =
          Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
      double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
      double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

      consumer.accept(
          observation.pose().toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }

    // Updating tunable numbers
    var hc = hashCode();
    if (angleTolerance.hasChanged(hc)) {
      setAngleTolerance();
    }
  }

  // Setters

  public void setAngleTolerance() {
    io.setAngleTolerance(angleTolerance.get());
  }

  // Getters

  // public boolean isOnTarget() {
  //   return inputs.rotToTarget.getZ <= angleTolerance.get() * (Math.PI/180.0);
  // }

  // public Rotation3d getRotateToTarget() {
  //   return inputs.rotatePose;
  // }

  // public Pose3d getDistToTarget() {
  //   return inputs.targetPose;
  // }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
