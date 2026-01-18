package frc.robot.stateMachines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StructureState;
import frc.robot.subsystems.drive.Drive;

public class ScoreStateMachine extends StateMachine {

  private final Drive drive;
  private final SuperStructure superStructure;

  private final LoggerGroup conditions = LoggerGroup.build("ScoreStateMachine/ShootConditions");
  private final LoggerEntry.Bool shooterAtRPM = conditions.buildBoolean("ShooterAtRPM");
  private final LoggerEntry.Bool robotAtAngle = conditions.buildBoolean("RobotAtAngle");
  Timer time = new Timer();

  public ScoreStateMachine(Drive drive, SuperStructure superStructure) {
    super("ScoreStateMachine");

    this.drive = drive;
    this.superStructure = superStructure;

    setInitialState(stateWithName("ScorePrep", () -> scorePrep()));
  }

  private StateHandler scorePrep() {
    if (!time.isRunning()) time.start();
    drive.setTargetAngle(calcDesiredRobotAngle());
    // drive.setState(DriveState.RotateToAngle);
    // superStructure.setState(StructureState.ScorePrep);
    superStructure.getShooter().setVel(Units.RPM.of(4000));
    boolean atRPM = superStructure.getShooter().isAtTarget();
    boolean atAngle = drive.isAtTargetAngle();
    shooterAtRPM.info(atRPM);
    robotAtAngle.info(atAngle);
    return time.hasElapsed(1)
        // && atAngle
        ? stateWithName("Score", () -> score())
        : null;
  }

  private StateHandler score() {
    drive.setTargetAngle(calcDesiredRobotAngle());
    superStructure.setState(StructureState.Score);
    return null;
  }

  private Rotation2d calcDesiredRobotAngle() {
    Pose2d shooterPose =
        drive
            .getPose()
            .plus(
                new Transform2d(
                    ShooterConstants.HORIZONTAL_OFFSET, Units.Inches.of(0), Rotation2d.kZero));
    return GeometryUtil.getHeading(
        shooterPose.getTranslation(), FieldConstants.HUB_TRANSLATION_BLUE);
  }
}
