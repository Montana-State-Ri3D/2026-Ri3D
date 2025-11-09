package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double volts;
    public Rotation2d angle = Rotation2d.kZero;
    public double current;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setAngle(Rotation2d angle) {}

  public default void configMotor(double p, double i, double d, double f) {}

  public default void resetSensorPosition(Rotation2d angle) {}
}
