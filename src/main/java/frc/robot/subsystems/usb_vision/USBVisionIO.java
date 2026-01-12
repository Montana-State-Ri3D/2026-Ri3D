package frc.robot.subsystems.usb_vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface USBVisionIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  class USBVisionInputs {
    public Transform3d targetPose;
    public double targetX;
    public double targetY;
    public double targetZ;

    public Rotation3d rotatePose;
    public double rotateX;
    public double rotateY;
    public double rotateZ;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(USBVisionInputs inputs) {}

  public default void setAngleTolerance(double angle) {}
}
