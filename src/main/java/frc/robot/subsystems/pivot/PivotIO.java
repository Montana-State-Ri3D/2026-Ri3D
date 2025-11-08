package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public class Inputs {
    double volts;
    Rotation2d angle;
    double current;
  }

  public default void updateInputs(Inputs inputs){}

  public default void setVoltage(double volts){}

  public default void setAngle(Rotation2d angle){}

  public default void configMotor(double p, double i, double d, double f){}
}
