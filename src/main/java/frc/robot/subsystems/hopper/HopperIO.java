package frc.robot.subsystems.hopper;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class HopperInputs {
    public double velocityRPM;
    public double appliedOutput;
    public double currentAmps;
    public double tempCelsius;
    public double tofDistanceInches;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HopperInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVel(AngularVelocity angle) {}

  public default void configMotor(double kV, double kP, double maxAcceleration) {}

  public default boolean setIdleMode(IdleMode value) {
    return false;
  }
}
