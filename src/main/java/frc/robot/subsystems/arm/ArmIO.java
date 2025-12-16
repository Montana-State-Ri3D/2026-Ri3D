package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class ArmInputs {
    public double angleDegrees;
    public double velocityDegreesPerSecond;
    public double appliedOutput;
    public double currentAmps;
    public double tempCelsius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setAngle(Angle angle) {}

  public default void configMotor(
      double kP, double kD, double kG, double maxVelocity, double maxAcceleration) {}

  public default void setSensorPosition(Angle angle) {}

  public default boolean setIdleMode(IdleMode value) {
    return false;
  }
}
