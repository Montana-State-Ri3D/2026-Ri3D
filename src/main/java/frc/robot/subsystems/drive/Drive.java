package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final MecanumDrivePoseEstimator poseEstimator =
      new MecanumDrivePoseEstimator(
          DriveConstants.KINEMATICS,
          Rotation2d.kZero,
          new MecanumDriveWheelPositions(),
          Pose2d.kZero);

  // TODO: tune PID
  private LoggedTunableNumber tunableP = new LoggedTunableNumber("Drive/pidf/p", 0);
  private LoggedTunableNumber tunableI = new LoggedTunableNumber("Drive/pidf/i", 0);
  private LoggedTunableNumber tunableD = new LoggedTunableNumber("Drive/pidf/d", 0);
  private LoggedTunableNumber tunableFF = new LoggedTunableNumber("Drive/pidf/ff", 0);

  public Drive(DriveIO io, GyroIO gyroIO) {
    this.io = io;
    this.gyroIO = gyroIO;
    updateConstants();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", inputs);
    poseEstimator.update(gyroInputs.yawPosition, inputs.positions);

    int hc = hashCode();
    if (tunableP.hasChanged(hc)
        || tunableI.hasChanged(hc)
        || tunableD.hasChanged(hc)
        || tunableFF.hasChanged(hc)) updateConstants();
  }

  private void updateConstants() {
    io.updateConstants(tunableP.get(), tunableI.get(), tunableD.get(), tunableFF.get());
  }

  public void driveRobotCentric(ChassisSpeeds speeds) {
    io.setVelocity(speeds);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the latest estimated rotation from the pose estimator. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @AutoLogOutput(key = "Drive/EstimatedPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public void setRotation(Rotation2d rot) {
    poseEstimator.resetRotation(rot);
  }

  public void stop() {
    io.setVoltage(Units.Volts.of(0));
  }

  public void setModuleVoltages(Voltage[] volts) {
    io.setVoltage(volts);
  }
}
