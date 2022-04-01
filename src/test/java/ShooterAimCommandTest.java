import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.shooter.ShooterAimCommand;
import org.junit.Test;

public class ShooterAimCommandTest {
  public static final double DELTA = 1e-1; // acceptable deviation range

  @Test
  public void testMovingWithVision1() {
    Rotation2d angle = Rotation2d.fromDegrees(26.667731675599);
    double range = 2.4954558701768;
    Rotation2d turretAngle = new Rotation2d();
    Pose2d robotPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(0.0, 0.0, 0.0);

    double movingGoalAngle =
        ShooterAimCommand.calculateMovingGoalRange(angle, range, turretAngle, robotPose, velocity);
    assertEquals(2.4954558701768, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithVision2() {
    Rotation2d angle = Rotation2d.fromDegrees(26.667731675599);
    double range = 2.4954558701768;
    Rotation2d turretAngle = new Rotation2d();
    Pose2d robotPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(-1.0, 1.0, 0.0);

    double movingGoalAngle =
        ShooterAimCommand.calculateMovingGoalRange(angle, range, turretAngle, robotPose, velocity);
    assertEquals(3.4824273143886, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithVision3() {
    Rotation2d angle = Rotation2d.fromDegrees(57.6756658484);
    double range = 2.0945882650297;
    Rotation2d turretAngle = new Rotation2d();
    Pose2d robotPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
    ChassisSpeeds velocity = new ChassisSpeeds(-1.0, -1.5, 0.0);

    double movingGoalAngle =
        ShooterAimCommand.calculateMovingGoalRange(angle, range, turretAngle, robotPose, velocity);
    assertEquals(3.0398067372779, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithoutVision1() {
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 3.0), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(0.0, 0.0, 0.0);

    double movingGoalAngle =
        ShooterAimCommand.calculateMovingGoalRange(robotPose, velocity);
    assertEquals(2.4954558701768, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithoutVision2() {
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 3.0), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(1.0, 1.0, 0.0);

    double movingGoalAngle =
        ShooterAimCommand.calculateMovingGoalRange(robotPose, velocity);
    assertEquals(0.98858484714262, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithoutVision3() {
    Pose2d robotPose = new Pose2d(new Translation2d(10.0, 5.0), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(-2.0, 1.0, 0.0);

    double movingGoalAngle =
        ShooterAimCommand.calculateMovingGoalRange(robotPose, velocity);
    assertEquals(2.2516216378424, movingGoalAngle, DELTA);
  }
}
