package frc.robot.subsystems.intake;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

  private final SparkFlex frontRollers =
      new SparkFlex(Constants.CanIDs.INTAKE_FRONT_ROLLERS_CAN_ID, MotorType.kBrushless);
  private final SparkFlex backRollers =
      new SparkFlex(Constants.CanIDs.INTAKE_BACK_ROLLERS_CAN_ID, MotorType.kBrushless);
  private final SparkMax extender =
      new SparkMax(Constants.CanIDs.INTAKE_EXTENDER_CAN_ID, MotorType.kBrushless);

  private SparkFlexConfig frontConfig = IntakeConstants.FrontRollers.MOTOR_CONFIG();
  private SparkFlexConfig backConfig = IntakeConstants.BackRollers.MOTOR_CONFIG();
  private SparkMaxConfig extenderConfig = IntakeConstants.Extender.MOTOR_CONFIG();

  private final RelativeEncoder frontRollersEncoder = frontRollers.getEncoder();
  private final RelativeEncoder backRollersEncoder = backRollers.getEncoder();
  private final RelativeEncoder extenderEncoder = extender.getEncoder();

  // private final VL6180 timeOfFlight = new VL6180(Port.kOnboard);

  public IntakeIOReal() {
    // timeOfFlight.startContinuous((int) (Constants.defaultPeriod * 1000));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.frontRollersVelocityRPM = frontRollersEncoder.getVelocity();
    inputs.frontRollersAppliedOutput = frontRollers.getAppliedOutput();
    inputs.frontRollersCurrentAmps = frontRollers.getOutputCurrent();
    inputs.frontRollersTempCelsius = frontRollers.getMotorTemperature();
    inputs.backRollersVelocityRPM = backRollersEncoder.getVelocity();
    inputs.backRollersAppliedOutput = backRollers.getAppliedOutput();
    inputs.backRollersCurrentAmps = backRollers.getOutputCurrent();
    inputs.backRollersTempCelsius = frontRollers.getMotorTemperature();
    inputs.extenderPositionInches = extenderEncoder.getPosition();
    inputs.extenderVelocityInchesPerSecond = extenderEncoder.getVelocity() * 60.0;
    inputs.extenderAppliedOutput = extender.getAppliedOutput();
    inputs.extenderCurrentAmps = extender.getOutputCurrent();
    inputs.extenderTempCelsius = extender.getMotorTemperature();
  }

  @Override
  public void setFrontVoltage(double volts) {
    frontRollers.setVoltage(volts);
  }

  @Override
  public void setBackVoltage(double volts) {
    backRollers.setVoltage(volts);
  }

  @Override
  public void setExtenderVoltage(double volts) {
    extender.setVoltage(volts);
  }

  @Override
  public void setFrontVel(AngularVelocity vel) {
    frontRollers
        .getClosedLoopController()
        .setReference(
            vel.in(Units.RPM) / IntakeConstants.FrontRollers.GEAR_RATIO,
            ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setBackVel(AngularVelocity vel) {
    backRollers
        .getClosedLoopController()
        .setReference(
            vel.in(Units.RPM) / IntakeConstants.BackRollers.GEAR_RATIO,
            ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setExtenderPos(Distance length) {
    extender.getClosedLoopController().setReference(length.in(Units.Inches), ControlType.kPosition);
  }

  @Override
  public void configFrontRollers(double kV, double kP, double maxAcceleration) {
    frontConfig.closedLoop.pidf(kP, 0, 0, kV);
    frontConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    frontRollers.configure(
        frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void configBackRollers(double kV, double kP, double maxAcceleration) {
    backConfig.closedLoop.pidf(kP, 0, 0, kV);
    backConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    backRollers.configure(
        backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void configExtender(double kP, double kD) {
    extenderConfig.closedLoop.pid(kP, 0, kD);
    extender.configure(
        extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setExtenderSensorPosition(Distance position) {
    extenderEncoder.setPosition(position.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    frontConfig.idleMode(value);
    backConfig.idleMode(value);
    extenderConfig.idleMode(IdleMode.kCoast);
    return frontRollers.configure(
                frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            == REVLibError.kOk
        && backRollers.configure(
                backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            == REVLibError.kOk
        && extender.configure(
                extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            == REVLibError.kOk;
  }
}
