package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** This command adjusts the shooter RPM, hood angle, and turret angle relative to a target. */
public class AimCommand extends CommandBase {
  // Subsystems
  private final Turret turret;
  private final Hood hood;
  private final Limelight limelight;
  private final Swerve swerve;

  // Supplier of the boolean whether to go into fender shot mode
  private final Supplier<Boolean> fenderShotButton;

  // The position of the hood for the fender shot
  private static final double FENDER_HOOD_POSITION = -0.25;

  private Rotation2d lastVisionTargetAngle = new Rotation2d();
  private Pose2d swerevPoseAtLastVisionUpdate = new Pose2d();
  private Rotation2d targetAngle = new Rotation2d(); // Relative to field

  // Coordinator System Transforms
  // TODO: Calculate Translation offset of the turret
  private static final Transform2d ROBOT_TO_TURRET =
    new Transform2d(
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(-90.0)));
  private static final Transform2d TURRET_TO_ROBOT = ROBOT_TO_TURRET.inverse();

  /**
   * Constructs an AimCommand
   *
   * @param turret Instance of Turret
   */
  public AimCommand(
      Swerve swerve,
      Turret turret,
      Hood hood,
      Limelight limelight,
      Supplier<Boolean> fenderShotButton) {
    this.turret = turret;
    this.hood = hood;
    this.swerve = swerve;

    this.limelight = limelight;

    this.fenderShotButton = fenderShotButton;

    // Add the shooter, hood, turret, and goalTracker as requirements
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (!fenderShotButton.get()) {
      var target = limelight.getCurrentTarget();
      calculateTargetAngle(target);
      if (target.hasTarget) {
        hood.aim(target.range);
        turret.aim(target.targetAngle);
      }
    } else {
      turret.goToZero();
      hood.setHoodRawPosition(FENDER_HOOD_POSITION);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }


  private void calculateTargetAngle(Limelight.TargetData data) {
    if (data.hasTarget) {
      final Rotation2d currentTargetAngle = new Rotation2d(turret.getAngle()).rotateBy(new Rotation2d(data.targetAngle));
      this.lastVisionTargetAngle = currentTargetAngle.rotateBy(TURRET_TO_ROBOT.getRotation());
      this.targetAngle = this.lastVisionTargetAngle;
      this.swerevPoseAtLastVisionUpdate = swerve.getPose();
    } else {
      final Pose2d currentSwervePose = swerve.getPose();
      final Transform2d lastVisionToCurrent = new Transform2d(
        this.swerevPoseAtLastVisionUpdate,
        currentSwervePose
      );
      this.targetAngle = this.targetAngle.rotateBy(lastVisionToCurrent.getRotation());
    }
  }
}
