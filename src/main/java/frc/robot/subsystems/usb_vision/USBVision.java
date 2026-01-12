// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.usb_vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class USBVision extends SubsystemBase {
  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(VisionConstants.ROOT_TABLE);

  private static final LoggerEntry.DecimalArray logToTarget =
      logGroup.buildDecimalArray("OffsetFromTarget");
  private static final LoggerEntry.Decimal logDistX = logGroup.buildDecimal("DistX");
  private static final LoggerEntry.Decimal logDistY = logGroup.buildDecimal("DistY");
  private static final LoggerEntry.Decimal logDistZ = logGroup.buildDecimal("DistZ");

  private static final LoggerEntry.Decimal logRotX = logGroup.buildDecimal("RotX");
  private static final LoggerEntry.Decimal logRotY = logGroup.buildDecimal("RotY");
  private static final LoggerEntry.Decimal logRotZ = logGroup.buildDecimal("RotZ");
  // private static final LoggerEntry.Bool logOnTarget = logGroup.buildBoolean("onTarget");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(VisionConstants.ROOT_TABLE);

  private final LoggedTunableNumber angleTolerance = group.build("angleTolerance", 0.1);

  private final USBVisionIO io;
  private final USBVisionInputsAutoLogged inputs = new USBVisionInputsAutoLogged();

  // private boolean onTarget = false;
  // private Angle angleToTarget = Units.Radians.zero();

  /** Creates a new Vision Subsystem. */
  public USBVision(USBVisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(VisionConstants.ROOT_TABLE, inputs);

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

  public Rotation3d getRotateToTarget() {
    return inputs.rotatePose;
  }

  public Transform3d getDistToTarget() {
    return inputs.targetPose;
  }
}
