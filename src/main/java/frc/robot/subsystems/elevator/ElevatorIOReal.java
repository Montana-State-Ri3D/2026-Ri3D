package frc.robot.subsystems.elevator;

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
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

  private final SparkFlex leadMotor =
      new SparkFlex(Constants.CanIDs.ELEVATOR_LEAD_CAN_ID, MotorType.kBrushless);
  private final SparkFlex followerMotor =
      new SparkFlex(Constants.CanIDs.ELEVATOR_FOLLOW_CAN_ID, MotorType.kBrushless);
  private SparkFlexConfig config = Constants.ElevatorConstants.MOTOR_CONFIG();

  private final RelativeEncoder encoder = leadMotor.getEncoder();

  public ElevatorIOReal() {

    SparkFlexConfig config = new SparkFlexConfig();
    config.follow(leadMotor, Constants.ElevatorConstants.FOLLOWER_INVERT);
    followerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.heightInches = encoder.getPosition();
    inputs.velocityInchesPerSecond = encoder.getVelocity() * 60.0;
    inputs.appliedOutput = leadMotor.getAppliedOutput();
    inputs.currentAmps = leadMotor.getOutputCurrent();
    inputs.tempCelsius = leadMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    leadMotor.setVoltage(volts);
  }

  @Override
  public void setHeight(Distance height) {
    leadMotor
        .getClosedLoopController()
        .setReference(
            height.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT,
            ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setSensorPosition(Distance position) {
    encoder.setPosition(position.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT);
  }

  @Override
  public void configMotor(
      double kP, double kD, double kG, double maxVelocity, double maxAcceleration) {
    config.closedLoop.pidf(kP, 0, kD, kG);
    config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    config.closedLoop.maxMotion.maxVelocity(maxVelocity);
    leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    config.idleMode(value);
    return leadMotor.configure(
            config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        == REVLibError.kOk;
  }
}
