package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class ElevatorInputs {
    public double heightInches;
    public double velocityInchesPerSecond;
    public double appliedOutput;
    public double currentAmps;
    public double tempCelsius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setHeight(Distance height) {}

  public default void configMotor(
      double kP, double kD, double kG, double maxVelocity, double maxAcceleration) {}

  public default void setSensorPosition(Distance position) {}

  public default boolean setIdleMode(IdleMode value) {
    return false;
  }
}
