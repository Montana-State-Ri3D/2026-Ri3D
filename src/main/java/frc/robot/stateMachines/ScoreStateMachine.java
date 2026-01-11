package frc.robot.stateMachines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.StateMachine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;

public class ScoreStateMachine extends StateMachine {

  private final Drive drive;
  private final SuperStructure superStructure;

  public ScoreStateMachine(Drive drive, SuperStructure superStructure) {
    super("ScoreStateMachine");

    this.drive = drive;
    this.superStructure = superStructure;

    setInitialState(stateWithName("ScorePrep", () -> scorePrep()));
  }

  private StateHandler scorePrep() {
    drive.setRotation(calcDesiredRobotAngle());
    drive.setState(DriveState.RotateToAngle);
    superStructure.setState(StructureState.ScorePrep);
    return superStructure.getShooter().isAtTarget() && drive.isAtTargetPose()
        ? stateWithName("Score", () -> score())
        : null;
  }

  private StateHandler score() {
    drive.setState(DriveState.RotateToAngle);
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
