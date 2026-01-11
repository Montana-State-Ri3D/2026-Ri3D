package frc.robot.subsystems.shooter;

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
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

  private final SparkFlex leadMotor =
      new SparkFlex(Constants.CanIDs.SHOOTER_LEAD_CAN_ID, MotorType.kBrushless);
  private final SparkFlex followMotor =
      new SparkFlex(Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID, MotorType.kBrushless);
  private SparkFlexConfig config = Constants.ElevatorConstants.MOTOR_CONFIG();

  private final RelativeEncoder encoder = leadMotor.getEncoder();

  private final SparkFlexConfig followConfig = new SparkFlexConfig();

  // private final VL6180 timeOfFlight = new VL6180(Port.kOnboard);

  public ShooterIOReal() {
    followConfig.follow(leadMotor, ShooterConstants.FOLLOW_INVERT);
    followMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedOutput = leadMotor.getAppliedOutput();
    inputs.leadCurrentAmps = leadMotor.getOutputCurrent();
    inputs.leadTempCelsius = leadMotor.getMotorTemperature();
    inputs.followCurrentAmps = followMotor.getOutputCurrent();
    inputs.followTempCelsius = followMotor.getMotorTemperature();
    // inputs.tofDistanceInches = timeOfFlight.getDistance().in(Units.Inches);
  }

  @Override
  public void setVoltage(double volts) {
    leadMotor.setVoltage(volts);
  }

  @Override
  public void setVel(AngularVelocity angle) {
    leadMotor
        .getClosedLoopController()
        .setReference(
            angle.in(Units.RPM) / ShooterConstants.GEAR_RATIO,
            ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void configMotor(double kV, double kP, double maxAcceleration) {
    config.closedLoop.pidf(kP, 0, 0, kV);
    config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    config.idleMode(value);
    followConfig.idleMode(value);
    return leadMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            == REVLibError.kOk
        && followMotor.configure(
                followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            == REVLibError.kOk;
  }
}
