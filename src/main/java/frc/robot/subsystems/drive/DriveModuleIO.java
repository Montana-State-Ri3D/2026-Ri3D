package frc.robot.subsystems.drive;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface DriveModuleIO {
  @AutoLog
  public static class DriveModuleSparkInputs {
    public double voltage;
    public double relativePosMeters;
    public double relativePosRotations;
    public double velMetersPerSecond;
    public double desiredVelMetersPerSecond;
    public double current;
  }

  public default void updateInputs() {}

  public default void setVoltage(Voltage volts) {}

  public default void setVelocity(LinearVelocity vel) {}

  public default Distance getRelativePosition() {
    return Units.Meters.zero();
  }

  public default LinearVelocity getVelocity() {
    return Units.MetersPerSecond.zero();
  }

  public default void updateMotorConfig(double kV, double kP) {}
}
