package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class IntakeInputs {
    public double frontRollersVelocityRPM;
    public double frontRollersAppliedOutput;
    public double frontRollersCurrentAmps;
    public double frontRollersTempCelsius;
    public double backRollersVelocityRPM;
    public double backRollersAppliedOutput;
    public double backRollersCurrentAmps;
    public double backRollersTempCelsius;
    public double extenderPositionInches;
    public double extenderVelocityInchesPerSecond;
    public double extenderAppliedOutput;
    public double extenderCurrentAmps;
    public double extenderTempCelsius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeInputs inputs) {}

  public default void setFrontVoltage(double volts) {}

  public default void setBackVoltage(double volts) {}

  public default void setExtenderVoltage(double volts) {}

  public default void setFrontVel(AngularVelocity angle) {}

  public default void setBackVel(AngularVelocity angle) {}

  public default void setExtenderPos(Distance length) {}

  public default void configFrontRollers(double kV, double kP, double maxAcceleration) {}

  public default void configBackRollers(double kV, double kP, double maxAcceleration) {}

  public default void configExtender(double kP, double kD) {}

  public default void setExtenderSensorPosition(Distance position) {}

  public default boolean setIdleMode(IdleMode value) {
    return false;
  }
}
