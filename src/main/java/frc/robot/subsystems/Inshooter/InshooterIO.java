package frc.robot.subsystems.Inshooter;

import org.littletonrobotics.junction.AutoLog;

public interface InshooterIO {

  @AutoLog
  public static class InshooterIOInputs {
    public double voltage;
    public double velocityRPM;
    public double current;
  }

  public default void updateInputs(InshooterIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRPM) {}

  public default void configMotor(double p, double i, double d, double f) {}
}
