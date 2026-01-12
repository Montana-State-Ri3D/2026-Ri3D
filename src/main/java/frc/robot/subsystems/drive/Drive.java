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
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.DriveToPose;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.autonomous.ChoreoTrajectoryWithName;
import frc.robot.subsystems.drive.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  public enum DriveState {
    Controller,
    PathFollow,
    DriveToPose,
    RotateToAngle
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

  private static final LoggerGroup loggerGroup = LoggerGroup.build(DriveConstants.ROOT_TABLE);
  private static final LoggerEntry.Text stateLogger = loggerGroup.buildString("currentState");
  private static final LoggerGroup rotateToAngleLoggerGroup = loggerGroup.subgroup("RotateToAngle");
  private static final LoggerEntry.Decimal rotationLogger =
      rotateToAngleLoggerGroup.buildDecimal("CurrentRotation");
  private static final LoggerEntry.Decimal targetRotationLogger =
      rotateToAngleLoggerGroup.buildDecimal("TargetRotation");

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

  private Pose2d targetPose = Pose2d.kZero;

  private DriveToPose driveToPose;

  private boolean newState = false;
  private DriveState prevState = DriveState.Controller;

  private final TunableNumberGroup rotateToAngleGroup = group.subgroup("RotateToAngle");
  private final LoggedTunableNumber rotToAngKP = rotateToAngleGroup.build("P", 0.3);
  private final LoggedTunableNumber rotToAngKD = rotateToAngleGroup.build("D", 0.0);
  private final LoggedTunableNumber rotToAngTolerance = rotateToAngleGroup.build("toleranceDeg", 4);
  private final PIDController rotateToAngleController =
      new PIDController(rotToAngKP.get(), 0, rotToAngKD.get());
  private Rotation2d targetAngle = Rotation2d.kZero;

  private boolean atTargetPose;
  private boolean atTargetAngle;

  public Drive(DriveModules modules, GyroIO gyroIO, CommandXboxController controller) {
    this.modules = modules;
    this.gyroIO = gyroIO;
    this.controller = controller;
    updateConstants();
    driveToPose = new DriveToPose(this, () -> targetPose);
    rotateToAngleController.enableContinuousInput(-Math.PI, Math.PI);
    rotateToAngleController.setTolerance(Math.toRadians(rotToAngTolerance.get()));
  }

  @Override
  public void periodic() {
    modules.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs(DriveConstants.ROOT_TABLE, inputs);
    Logger.processInputs(DriveConstants.ROOT_TABLE + "/Gyro", gyroInputs);
    Logger.recordOutput(DriveConstants.ROOT_TABLE + "/TargetPose", targetPose);
    Logger.recordOutput(
        DriveConstants.ROOT_TABLE + "/TargetAngleDegrees", targetAngle.getDegrees());
    if (RobotBase.isSimulation())
      simYawAngle =
          simYawAngle.plus(
              Rotation2d.fromRadians(
                  inputs.realSpeeds.omegaRadiansPerSecond * Constants.defaultPeriod));
    poseEstimator.update(
        RobotBase.isReal() ? gyroInputs.yawPosition : simYawAngle, inputs.positions);

    int hc = hashCode();
    if (tunableP.hasChanged(hc) || tunableV.hasChanged(hc)) updateConstants();

    newState = !prevState.equals(state);
    switch (state) {
      case Controller:
        atTargetAngle = false;
        driveController(null);
        break;
      case PathFollow:
        atTargetAngle = false;
        if (choreoHelper != null) {
          ChassisSpeedsWithPathEnd result =
              choreoHelper.calculateChassisSpeeds(getPose(), System.currentTimeMillis() / 1000.0);
          atTargetPose = result.atEndOfPath();
          driveRobotCentric(result.chassisSpeeds());
        } else {
          atTargetPose = false;
          driveRobotCentric(new ChassisSpeeds());
        }
        break;
      case DriveToPose:
        atTargetAngle = false;
        if (newState) driveToPose.init();
        driveToPose.run();
        break;
      case RotateToAngle:
        if (rotToAngKP.hasChanged(hashCode()) || rotToAngKD.hasChanged(hashCode()))
          rotateToAngleController.setPID(rotToAngKP.get(), 0, rotToAngKD.get());
        if (rotToAngTolerance.hasChanged(hc))
          rotateToAngleController.setTolerance(Math.toRadians(rotToAngTolerance.get()));
        double currentRot = getRotation().getRadians();
        double targetRot = targetAngle.getRadians();
        rotationLogger.info(currentRot);
        targetRotationLogger.info(targetRot);
        driveController(rotateToAngleController.calculate(currentRot, targetRot));
        atTargetAngle = rotateToAngleController.atSetpoint();
        break;
      default:
        break;
    }
    stateLogger.info(state.name());

    prevState = state;
  }

  public void setTrajectory(ChoreoTrajectoryWithName traj) {
    atTargetPose = false;
    choreoHelper =
        new ChoreoHelper(
            System.currentTimeMillis() / 1000.0,
            getPose(),
            traj,
            DriveConstants.WHEEL_OFFSETS[0].getX() / 2.0,
            new PIDController(linearKP.get(), 0, linearKD.get()),
            new PIDController(linearKP.get(), 0, linearKD.get()),
            new PIDController(angularKP.get(), 0, angularKD.get()));
    choreoHelper.setFinalTargetError(0.06);
    choreoHelper.setFinalOffsetError(0.05);
    choreoHelper.setFinalHeadingError(0.05);
  }

  public void setTargetPose(Pose2d target) {
    targetPose = target;
  }

  public void setTargetAngle(Rotation2d target) {
    targetAngle = target;
  }

  private void driveController(Double omegaOverride) {
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
            omegaOverride == null
                ? Constants.DriveConstants.MAX_ANGULAR_SPEED.times(
                    Math.copySign(angMagnitude * angMagnitude, angMagnitude))
                : Constants.DriveConstants.MAX_ANGULAR_SPEED.times(omegaOverride),
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

  public ChassisSpeeds fieldVelocity() {
    return inputs.realSpeeds;
  }

  @AutoLogOutput(key = DriveConstants.ROOT_TABLE + "/EstimatedPose")
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

  public boolean isAtTargetPose() {
    return atTargetPose;
  }

  public boolean isAtTargetAngle() {
    return atTargetAngle;
  }
}
