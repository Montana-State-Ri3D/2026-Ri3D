package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public ChassisSpeeds realSpeeds;
    public ChassisSpeeds desiredSpeeds;
    public MecanumDriveWheelPositions positions;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(ChassisSpeeds speeds) {}

  /** Set direct voltage of all motors. */
  public default void setVoltage(Voltage volts) {}

  /** Set direct voltage of specific motors. */
  public default void setVoltage(Voltage[] volts) {}

  /** Set motor PIDFs. */
  public default void updateConstants(double p, double i, double d, double ff) {}
}
