package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class IntakeInputs {
    public double velocityRPM;
    public double appliedOutput;
    public double currentAmps;
    public double tempCelsius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVel(AngularVelocity angle) {}

  public default void configMotor(double kV, double kP, double maxAcceleration) {}

  public default boolean setIdleMode(IdleMode value) {
    return false;
  }
}
