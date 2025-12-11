package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;

public class DriveIOSpark implements DriveIO {
  private DriveModuleSpark[] modules = new DriveModuleSpark[DriveConstants.NUM_MODULES];

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  public DriveIOSpark() {
    for (int i = 0; i < DriveConstants.NUM_MODULES; i++) {
      modules[i] = new DriveModuleSpark(i);
    }
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.positions =
        new MecanumDriveWheelPositions(
            modules[0].getRelativePosition(),
            modules[1].getRelativePosition(),
            modules[2].getRelativePosition(),
            modules[3].getRelativePosition());
    inputs.realSpeeds =
        DriveConstants.KINEMATICS.toChassisSpeeds(
            new MecanumDriveWheelSpeeds(
                modules[0].getVel(),
                modules[1].getVel(),
                modules[2].getVel(),
                modules[3].getVel()));
    inputs.desiredSpeeds = desiredSpeeds;
  }

  @Override
  public void setVelocity(ChassisSpeeds speeds) {
    // Convert to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds =
        constrainSpeedsToMaxSpeed(DriveConstants.KINEMATICS.toWheelSpeeds(speeds));
    // Store desired speed
    desiredSpeeds = DriveConstants.KINEMATICS.toChassisSpeeds(wheelSpeeds);
    // Set the individual wheel speeds
    modules[0].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.frontLeftMetersPerSecond));
    modules[1].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.frontRightMetersPerSecond));
    modules[2].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.rearLeftMetersPerSecond));
    modules[3].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.rearRightMetersPerSecond));
  }

  private MecanumDriveWheelSpeeds constrainSpeedsToMaxSpeed(MecanumDriveWheelSpeeds speeds) {
    double max = 0;
    if (Math.abs(speeds.frontLeftMetersPerSecond) > max)
      max = Math.abs(speeds.frontLeftMetersPerSecond);
    if (Math.abs(speeds.frontRightMetersPerSecond) > max)
      max = Math.abs(speeds.frontRightMetersPerSecond);
    if (Math.abs(speeds.rearLeftMetersPerSecond) > max)
      max = Math.abs(speeds.rearLeftMetersPerSecond);
    if (Math.abs(speeds.rearRightMetersPerSecond) > max)
      max = Math.abs(speeds.rearRightMetersPerSecond);
    double multiplier = 1;
    if (max > DriveConstants.MAX_LINEAR_SPEED.in(Units.MetersPerSecond)) {
      multiplier = DriveConstants.MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / max;
    }
    return speeds.times(multiplier);
  }

  @Override
  public void setVoltage(Voltage volts) {
    for (DriveModuleSpark module : modules) {
      module.setVoltage(volts);
    }
  }

  @Override
  public void setVoltage(Voltage[] volts) {
    for (int i = 0; i < DriveConstants.NUM_MODULES; i++) {
      modules[i].setVoltage(volts[i]);
    }
  }

  @Override
  public void updateConstants(double p, double i, double d, double ff) {
    for (DriveModuleSpark module : modules) {
      module.updateMotorConfig(p, i, d, ff);
    }
  }
}
