package frc.robot.subsystems.usb_vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class USBVisionIOReal implements USBVisionIO {

  private Thread visionThread = new Thread(this::apriltagVisionThreadProc);

  private double tolerance = 0.0;

  public USBVisionIOReal() {
    visionThread.setDaemon(true);
    visionThread.start();
  }

  @Override
  public void updateInputs(USBVisionInputs inputs) {
    // Save pose observations to inputs object
    List<PoseObservation> poseObservations = DataSync.getObservations();
    if (poseObservations != null) {
      inputs.poseObservations = new PoseObservation[poseObservations.size()];
      for (int i = 0; i < poseObservations.size(); i++) {
        inputs.poseObservations[i] = poseObservations.get(i);
      }
    }

    // Pose3d robotPose = DataSync.getRobotPose();
    // Pose3d tagPose = DataSync.getTagPose();
    // Rotation3d rot = DataSync.getRot();
    // inputs.robotVisionPose = robotPose;
    // inputs.targetPose = tagPose;
    // if (tagPose != null) {
    //   inputs.targetX = tagPose.getX();
    //   inputs.targetY = tagPose.getY();
    //   inputs.targetZ = tagPose.getZ();
    // }
    // inputs.rotatePose = rot;
    // if (rot != null) {
    //   inputs.rotateX = rot.getX();
    //   inputs.rotateY = rot.getY();
    //   inputs.rotateZ = rot.getZ();
    // }
  }

  @Override
  public void setAngleTolerance(double angle) {
    tolerance = angle;
  }

  public static class DataSync {
    private static List<PoseObservation> m_poseObservations;
    private static Pose3d m_robotPose;
    private static Pose3d m_tagPose;
    private static Rotation3d m_rot;

    public static synchronized void setObservations(List<PoseObservation> poseObservations) {
      m_poseObservations = poseObservations;
    }

    public static synchronized void setRobotPose(Pose3d pose) {
      m_robotPose = pose;
    }

    public static synchronized void setTagPose(Pose3d pose) {
      m_tagPose = pose;
    }

    public static synchronized void setRot(Rotation3d rot) {
      m_rot = rot;
    }

    public static synchronized List<PoseObservation> getObservations() {
      return m_poseObservations;
    }

    public static synchronized Pose3d getRobotPose() {
      return m_robotPose;
    }

    public static synchronized Pose3d getTagPose() {
      return m_tagPose;
    }

    public static synchronized Rotation3d getRot() {
      return m_rot;
    }
  }

  void apriltagVisionThreadProc() {
    var detector = new AprilTagDetector();
    // look for tag36h11, correct 1 error bit (hamming distance 1)
    // hamming 1 allocates 781KB, 2 allocates 27.4 MB, 3 allocates 932 MB
    // max of 1 recommended for RoboRIO 1, while hamming 2 is feasible on the RoboRIO 2
    detector.addFamily("tag36h11", 1);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    var poseEstConfig =
        new AprilTagPoseEstimator.Config(
            0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // Instantiate once
    ArrayList<Long> tags = new ArrayList<>();
    var outlineColor = new Scalar(0, 255, 0);
    var crossColor = new Scalar(0, 0, 255);

    // We'll output to NT
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
    // IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);
      double timestamp = Timer.getFPGATimestamp();

      List<PoseObservation> poseObservations = new LinkedList<>();

      // have not seen any tags yet
      tags.clear();

      for (AprilTagDetection detection : detections) {
        // remember we saw this tag
        tags.add((long) detection.getId());

        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
            mat,
            Integer.toString(detection.getId()),
            new Point(cx + ll, cy),
            Imgproc.FONT_HERSHEY_SIMPLEX,
            1,
            crossColor,
            3);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(detection.getId());
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = estimator.estimate(detection);
          Transform3d coordinateShiftCamToTarget =
              new Transform3d(
                  cameraToTarget.getX(),
                  cameraToTarget.getZ(),
                  cameraToTarget.getY(),
                  new Rotation3d(
                      cameraToTarget.getRotation().getX(),
                      cameraToTarget.getRotation().getZ(),
                      cameraToTarget.getRotation().getY()));
          Transform3d fieldToCamera = fieldToTarget.plus(coordinateShiftCamToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          Rotation3d rot = cameraToTarget.getRotation();

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  timestamp, // Timestamp
                  robotPose, // 3D pose estimate
                  detection.getDecisionMargin(), // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm())); // Average tag distance

          tagsTable
              .getEntry("pose_" + detection.getId())
              .setDoubleArray(
                  new double[] {
                    robotPose.getX(), robotPose.getY(), robotPose.getZ(),
                    robotPose.getRotation().getX(), robotPose.getRotation().getY(),
                        robotPose.getRotation().getZ()
                  });
        }
      }

      // put list of tags onto dashboard
      // pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());
      DataSync.setObservations(poseObservations);

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }
    // pubTags.close();
    detector.close();
  }
}
