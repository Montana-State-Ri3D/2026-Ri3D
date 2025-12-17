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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final DriveModules modules;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final MecanumDrivePoseEstimator poseEstimator =
      new MecanumDrivePoseEstimator(
          DriveConstants.KINEMATICS,
          Rotation2d.kZero,
          new MecanumDriveWheelPositions(),
          Pose2d.kZero);

  private static final TunableNumberGroup group = new TunableNumberGroup(DriveConstants.ROOT_TABLE);

  // TODO: tune PID

  private static LoggedTunableNumber tunableV = group.build("kV");
  private static LoggedTunableNumber tunableP = group.build("kP");

  private Rotation2d simYawAngle = Rotation2d.kZero;

  static {
    if (RobotBase.isReal()) {
      tunableV.initDefault(0.009);
      tunableP.initDefault(0);
    } else {
      tunableV.initDefault(0.009);
      tunableP.initDefault(0);
    }
  }

  public Drive(DriveModules modules, GyroIO gyroIO) {
    this.modules = modules;
    this.gyroIO = gyroIO;
    updateConstants();
  }

  @Override
  public void periodic() {
    modules.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs(DriveConstants.ROOT_TABLE, inputs);
    Logger.processInputs(DriveConstants.ROOT_TABLE + "/Gyro", gyroInputs);
    if (RobotBase.isSimulation())
      simYawAngle =
          simYawAngle.plus(
              Rotation2d.fromRadians(
                  inputs.realSpeeds.omegaRadiansPerSecond * Constants.defaultPeriod));
    poseEstimator.update(
        RobotBase.isReal() ? gyroInputs.yawPosition : simYawAngle, inputs.positions);

    int hc = hashCode();
    if (tunableP.hasChanged(hc) || tunableV.hasChanged(hc)) updateConstants();
  }

  private void updateConstants() {
    modules.updateConstants(tunableV.get(), tunableP.get());
  }

  public void driveRobotCentric(ChassisSpeeds speeds) {
    modules.setVelocity(speeds);
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
    modules.setVoltage(Units.Volts.of(0));
  }

  public void setModuleVoltages(Voltage[] volts) {
    modules.setVoltage(volts);
  }
}
