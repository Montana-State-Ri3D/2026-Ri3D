package frc.robot.subsystems.Inshooter;

import org.littletonrobotics.junction.AutoLog;

public interface InshooterIO {

  @AutoLog
  public class Inputs {
    double voltage;
    double velocityRPM;
    double current;
  }

  public default void updateInputs(Inputs inputs){}

  public default void setVoltage(double volts){}

  public default void setVelocity(double velocityRPM){}

  public default void configMotor(double p, double i, double d, double f){}
}
