package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj.I2C.Port;
import frc.lib.teamBSR.VL6180;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class IntakeIOReal implements IntakeIO {

  private final SparkFlex motor =
      new SparkFlex(Constants.CanIDs.INTAKE_CAN_ID, MotorType.kBrushless);
  private SparkFlexConfig config = Constants.ElevatorConstants.MOTOR_CONFIG();

  private final RelativeEncoder encoder = motor.getEncoder();

  private final VL6180 timeOfFlight = new VL6180(Port.kOnboard);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedOutput = motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
    inputs.tofDistanceInches = timeOfFlight.getDistance().in(Units.Inches);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVel(AngularVelocity angle) {
    motor
        .getClosedLoopController()
        .setReference(
            angle.in(Units.RPM) / ArmConstants.GEAR_RATIO, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void configMotor(double kV, double kP, double maxAcceleration) {
    config.closedLoop.pidf(kP, 0, 0, kV);
    config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    config.idleMode(value);
    return motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        == REVLibError.kOk;
  }
}
