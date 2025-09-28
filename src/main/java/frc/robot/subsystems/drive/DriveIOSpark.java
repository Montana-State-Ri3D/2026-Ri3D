package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;

public class DriveIOSpark implements DriveIO {
  private DriveModuleSpark[] modules;

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
            modules[0].getAbsolutePosition(),
            modules[1].getAbsolutePosition(),
            modules[2].getAbsolutePosition(),
            modules[3].getAbsolutePosition());
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
    desiredSpeeds = speeds;
    // Convert to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = DriveConstants.KINEMATICS.toWheelSpeeds(speeds);
    // Get the individual wheel speeds
    modules[0].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.frontLeftMetersPerSecond));
    modules[1].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.frontRightMetersPerSecond));
    modules[2].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.rearLeftMetersPerSecond));
    modules[3].setVelocity(Units.MetersPerSecond.of(wheelSpeeds.rearRightMetersPerSecond));
  }

  @Override
  public void setVoltage(Voltage volts) {
    for (DriveModuleSpark module : modules) {
      module.setVoltage(volts);
    }
  }
}
