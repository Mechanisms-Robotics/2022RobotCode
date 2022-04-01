import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.turret.TurretAimCommand;
import org.junit.Test;

public class TurretAimCommandTest {
  public static final double DELTA = 1e-1; // acceptable deviation range

  @Test
  public void testMovingWithVision1() {
    Rotation2d angle = Rotation2d.fromDegrees(26.667731675599);
    double range = 2.4954558701768;
    Rotation2d turretAngle = new Rotation2d();
    Pose2d robotPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(0.0, 0.0, 0.0);

    double movingGoalAngle =
        TurretAimCommand.calculateMovingGoalAngle(angle, range, turretAngle, robotPose, velocity);
    assertEquals(26.667731675599, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithVision2() {
    Rotation2d angle = Rotation2d.fromDegrees(26.667731675599);
    double range = 2.4954558701768;
    Rotation2d turretAngle = new Rotation2d();
    Pose2d robotPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(1.0, 0.0, 0.0);

    double movingGoalAngle =
        TurretAimCommand.calculateMovingGoalAngle(angle, range, turretAngle, robotPose, velocity);
    assertEquals(48.81407483429, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithVision3() {
    Rotation2d angle = Rotation2d.fromDegrees(-32.324334152);
    double range = 2.0945882650297;
    Rotation2d turretAngle = Rotation2d.fromDegrees(0.0);
    Pose2d robotPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(270.0));
    ChassisSpeeds velocity = new ChassisSpeeds(0.5, 0.5, 0.0);

    double movingGoalAngle =
        TurretAimCommand.calculateMovingGoalAngle(angle, range, turretAngle, robotPose, velocity);
    assertEquals(-11.677502601, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithoutVision1() {
    Rotation2d turretAngle = Rotation2d.fromDegrees(0.0);
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 3.0), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(0.0, 0.0, 0.0);

    double movingGoalAngle =
        TurretAimCommand.calculateMovingGoalAngle(turretAngle, robotPose, velocity);
    assertEquals(26.667731675599, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithoutVision2() {
    Rotation2d turretAngle = Rotation2d.fromDegrees(0.0);
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 3.0), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds velocity = new ChassisSpeeds(1.0, 1.0, 0.0);

    double movingGoalAngle =
        TurretAimCommand.calculateMovingGoalAngle(turretAngle, robotPose, velocity);
    assertEquals(-7.5563440564395, movingGoalAngle, DELTA);
  }

  @Test
  public void testMovingWithoutVision3() {
    Rotation2d turretAngle = Rotation2d.fromDegrees(0.0);
    Pose2d robotPose = new Pose2d(new Translation2d(10.0, 5.0), Rotation2d.fromDegrees(0.0));
    ChassisSpeeds velocity = new ChassisSpeeds(-1.0, 1.0, 0.0);

    double movingGoalAngle =
        TurretAimCommand.calculateMovingGoalAngle(turretAngle, robotPose, velocity);
    assertEquals(-13.719345264, movingGoalAngle, DELTA);
  }
}
