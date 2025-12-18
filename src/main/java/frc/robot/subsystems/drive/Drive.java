package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.autonomous.ChoreoTrajectoryWithName;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  public enum DriveState {
    Controller,
    PathFollow
  }

  private DriveState state = DriveState.Controller;

  private final DriveModules modules;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final CommandXboxController controller;

  private final MecanumDrivePoseEstimator poseEstimator =
      new MecanumDrivePoseEstimator(
          DriveConstants.KINEMATICS,
          Rotation2d.kZero,
          new MecanumDriveWheelPositions(),
          Pose2d.kZero);

  private static final TunableNumberGroup group = new TunableNumberGroup(DriveConstants.ROOT_TABLE);

  private static LoggedTunableNumber tunableV = group.build("kV");
  private static LoggedTunableNumber tunableP = group.build("kP");

  static { // TODO: tune PID
    if (Constants.currentMode == Mode.REAL) {
      tunableV.initDefault(0.009);
      tunableP.initDefault(0);
    } else {
      tunableV.initDefault(0.082);
      tunableP.initDefault(0);
    }
  }

  private final TunableNumberGroup linear = group.subgroup("Linear");
  private final LoggedTunableNumber linearKP = linear.build("P", 0.1);
  private final LoggedTunableNumber linearKD = linear.build("D", 0.0);
  private final TunableNumberGroup angular = group.subgroup("Angular");
  private final LoggedTunableNumber angularKP = angular.build("P", 0.1);
  private final LoggedTunableNumber angularKD = angular.build("D", 0.0);

  private Rotation2d simYawAngle = Rotation2d.kZero;

  private final double DEADBAND = 0.1;

  private ChoreoHelper choreoHelper;

  public Drive(DriveModules modules, GyroIO gyroIO, CommandXboxController controller) {
    this.modules = modules;
    this.gyroIO = gyroIO;
    this.controller = controller;
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

    switch (state) {
      case Controller:
        driveController();
        break;
      case PathFollow:
        if (choreoHelper != null) {
          driveRobotCentric(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  choreoHelper
                      .calculateChassisSpeeds(getPose(), System.currentTimeMillis() / 1000.0)
                      .chassisSpeeds(),
                  simYawAngle));
        } else {
          driveRobotCentric(new ChassisSpeeds());
        }
        break;
      default:
        break;
    }
  }

  public void setTrajectory(ChoreoTrajectoryWithName traj) {
    choreoHelper =
        new ChoreoHelper(
            System.currentTimeMillis() / 1000.0,
            getPose(),
            traj,
            DriveConstants.WHEEL_OFFSETS[0].getX() / 2.0,
            new PIDController(linearKP.get(), 0, linearKD.get()),
            new PIDController(linearKP.get(), 0, linearKD.get()),
            new PIDController(angularKP.get(), 0, angularKD.get()));
  }

  private void driveController() {
    double angMagnitude = MathUtil.applyDeadband(-controller.getRightX(), DEADBAND);
    LinearVelocity speedX =
        DriveConstants.MAX_LINEAR_SPEED.times(
            MathUtil.applyDeadband(-controller.getLeftY(), DEADBAND));
    LinearVelocity speedY =
        DriveConstants.MAX_LINEAR_SPEED.times(
            MathUtil.applyDeadband(-controller.getLeftX(), DEADBAND));
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      speedX = speedX.unaryMinus();
      speedY = speedY.unaryMinus();
    }
    driveRobotCentric(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speedX,
            speedY,
            Constants.DriveConstants.MAX_ANGULAR_SPEED.times(
                Math.copySign(angMagnitude * angMagnitude, angMagnitude)),
            getRotation()));
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

  public void setState(DriveState state) {
    this.state = state;
  }
}
