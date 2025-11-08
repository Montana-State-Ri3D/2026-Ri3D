package frc.robot.subsystems.Inshooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class InshooterIOReal implements InshooterIO {

  private SparkFlex motor = new SparkFlex(0, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkClosedLoopController closedLoop;

  public InshooterIOReal() {
    closedLoop = motor.getClosedLoopController();
    encoder = motor.getExternalEncoder();
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.voltage = motor.getBusVoltage();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.current = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRPM) {
    closedLoop.setReference(velocityRPM, ControlType.kVelocity);
  }

  @Override
  public void configMotor(double p, double i, double d, double f) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.pidf(p, i, d, f);
    config.inverted(false);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
