package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.AutoLog;

public class DriveModules {

  @AutoLog
  public static class DriveIOInputs {
    public ChassisSpeeds realSpeeds = new ChassisSpeeds();
    public ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    public MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions();
  }

  private DriveModuleIO[] modules = new DriveModuleIO[DriveConstants.NUM_MODULES];

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  public DriveModules(boolean real) {
    for (int i = 0; i < DriveConstants.NUM_MODULES; i++) {
      modules[i] = real ? new DriveModuleIOSpark(i) : new DriveModuleIOSim(i);
    }
  }

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
                modules[0].getVelocity(),
                modules[1].getVelocity(),
                modules[2].getVelocity(),
                modules[3].getVelocity()));
    inputs.desiredSpeeds = desiredSpeeds;
    for (DriveModuleIO module : modules) {
      module.updateInputs();
    }
  }

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

  public void setVoltage(Voltage volts) {
    for (DriveModuleIO module : modules) {
      module.setVoltage(volts);
    }
  }

  public void setVoltage(Voltage[] volts) {
    for (int i = 0; i < DriveConstants.NUM_MODULES; i++) {
      modules[i].setVoltage(volts[i]);
    }
  }

  public void updateConstants(double v, double p) {
    for (DriveModuleIO module : modules) {
      module.updateMotorConfig(v, p);
    }
  }
}
