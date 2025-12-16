package frc.robot.subsystems.arm;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOReal implements ArmIO {

  private final SparkFlex motor =
      new SparkFlex(Constants.CanIDs.ARM_CAN_ID, MotorType.kBrushless);
  private SparkFlexConfig config = Constants.ElevatorConstants.MOTOR_CONFIG();

  private final RelativeEncoder encoder = motor.getEncoder();

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.angleDegrees = Units.Rotation.of(encoder.getPosition()).in(Units.Degree);
    inputs.velocityDegreesPerSecond =
        Units.RPM.of(encoder.getVelocity()).in(Units.DegreesPerSecond);
    inputs.appliedOutput = motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setAngle(Angle angle) {
    motor
        .getClosedLoopController()
        .setReference(
            angle.in(Units.Rotation) / ArmConstants.GEAR_RATIO,
            ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setSensorPosition(Angle angle) {
    encoder.setPosition(angle.in(Units.Rotation) / ArmConstants.GEAR_RATIO);
  }

  @Override
  public void configMotor(
      double kP, double kD, double kG, double maxVelocity, double maxAcceleration) {
    config.closedLoop.pidf(
        kP, 0, kD, kG * Math.cos(Units.Rotation.of(encoder.getPosition()).in(Units.Radian)));
    config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    config.closedLoop.maxMotion.maxVelocity(maxVelocity);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    config.idleMode(value);
    return motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        == REVLibError.kOk;
  }
}
