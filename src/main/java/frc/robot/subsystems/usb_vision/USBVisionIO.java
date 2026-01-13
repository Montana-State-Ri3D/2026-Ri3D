package frc.robot.subsystems.usb_vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface USBVisionIO {

  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public class USBVisionInputs {
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];

    // public Pose3d robotVisionPose;

    // public Pose3d targetPose;
    // public double targetX;
    // public double targetY;
    // public double targetZ;

    // public Rotation3d rotatePose;
    // public double rotateX;
    // public double rotateY;
    // public double rotateZ;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(USBVisionInputs inputs) {}

  public default void setAngleTolerance(double angle) {}
}
