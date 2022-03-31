import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.turret.TurretAimCommand;
import org.junit.Test;

public class TurretAimCommandTest {
  public static final double DELTA = 1e-2; // acceptable deviation range

  @Test
  public void testAngle1() {
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 3.0), Rotation2d.fromDegrees(90.0));

    Rotation2d currentTurretAngle = new Rotation2d();

    double angle = TurretAimCommand.calculateGoalAngle(robotPose, currentTurretAngle);
    assertEquals(26.6677317, angle, DELTA);
  }

  @Test
  public void testAngle2() {
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 5.0), Rotation2d.fromDegrees(90.0));

    Rotation2d currentTurretAngle = new Rotation2d();

    double angle = TurretAimCommand.calculateGoalAngle(robotPose, currentTurretAngle);
    assertEquals(-21.5351258, angle, DELTA);
  }

  @Test
  public void testAngle3() {
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 5.0), Rotation2d.fromDegrees(115.0));

    Rotation2d currentTurretAngle = new Rotation2d();

    double angle = TurretAimCommand.calculateGoalAngle(robotPose, currentTurretAngle);
    assertEquals(-46.5351258, angle, DELTA);
  }

  @Test
  public void testAngle4() {
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 5.0), Rotation2d.fromDegrees(80.0));

    Rotation2d currentTurretAngle = new Rotation2d();

    double angle = TurretAimCommand.calculateGoalAngle(robotPose, currentTurretAngle);
    assertEquals(-11.5351258, angle, DELTA);
  }

  @Test
  public void testAngle5() {
    Pose2d robotPose = new Pose2d(new Translation2d(6.0, 5.0), Rotation2d.fromDegrees(0.0));

    Rotation2d currentTurretAngle = new Rotation2d();

    double angle = TurretAimCommand.calculateGoalAngle(robotPose, currentTurretAngle);
    assertEquals(68.4648742, angle, DELTA);
  }

  @Test
  public void testAngle6() {
    Pose2d robotPose = new Pose2d(new Translation2d(10.0, 3.0), Rotation2d.fromDegrees(0.0));

    Rotation2d currentTurretAngle = new Rotation2d();

    double angle = TurretAimCommand.calculateGoalAngle(robotPose, currentTurretAngle);
    assertEquals(-122.34, angle, DELTA);
  }

  @Test
  public void testAngle7() {
    Pose2d robotPose = new Pose2d(new Translation2d(10.0, 5.0), Rotation2d.fromDegrees(0.0));

    Rotation2d currentTurretAngle = new Rotation2d();

    double angle = TurretAimCommand.calculateGoalAngle(robotPose, currentTurretAngle);
    assertEquals(63.564577062185286, angle, DELTA);
  }
}
