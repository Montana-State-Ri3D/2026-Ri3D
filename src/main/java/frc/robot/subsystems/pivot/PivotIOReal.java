package frc.robot.subsystems.pivot;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.PivotConstants;

public class PivotIOReal implements PivotIO {

  private SparkMax motor = new SparkMax(CanIDs.PIVOT_ID, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkClosedLoopController closedLoop;

  public PivotIOReal() {
    closedLoop = motor.getClosedLoopController();
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(encoder.getPosition() / PivotConstants.GEAR_RATIO);
    inputs.current = motor.getOutputCurrent();
    inputs.volts = motor.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    closedLoop.setReference(
        angle.getRotations() * PivotConstants.GEAR_RATIO, ControlType.kPosition);
  }

  @Override
  public void configMotor(double p, double i, double d, double f) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.closedLoop.pidf(p, i, d, f);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {
    REVLibError err = encoder.setPosition(angle.getRotations());
    System.out.println(err.toString());
  }
}
