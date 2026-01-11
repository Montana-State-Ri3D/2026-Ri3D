package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class ShooterInputs {
    public double velocityRPM;
    public double appliedOutput;
    public double leadCurrentAmps;
    public double leadTempCelsius;
    public double tofDistanceInches;
    public double followCurrentAmps;
    public double followTempCelsius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVel(AngularVelocity angle) {}

  public default void configMotor(double kV, double kP, double maxAcceleration) {}

  public default boolean setIdleMode(IdleMode value) {
    return false;
  }
}
