package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private Rotation2d fieldVisionTargetAngle = new Rotation2d();
  private Pose2d swerevPoseAtLastVisionUpdate = new Pose2d();
  private Rotation2d targetAngle = new Rotation2d(); // Relative to field
  private double lastTargetRange = 0.0;

  // Coordinator System Transforms
  // TODO: Calculate Translation offset of the turret
  private static final Transform2d ROBOT_TO_TURRET =
    new Transform2d(
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(90.0)));
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
//      SmartDashboard.putNumber("Last Vision Target Angle", this.lastVisionTargetAngle.getDegrees());
//      SmartDashboard.putNumber("Target Angle", this.targetAngle.getDegrees());
      if (target.hasTarget) {
        hood.aim(target.range);
        turret.aim(target.targetAngle, swerve.getSpeeds(), target.range);
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
      final Rotation2d currentTargetAngle = new Rotation2d(turret.getAngle()).rotateBy(Rotation2d.fromDegrees(data.targetAngle));
      this.lastVisionTargetAngle = currentTargetAngle.rotateBy(TURRET_TO_ROBOT.getRotation()).rotateBy(swerve.getHeading());
      this.targetAngle = this.lastVisionTargetAngle;
      this.lastTargetRange = data.range;
      this.swerevPoseAtLastVisionUpdate = swerve.getPose();
    } else {
      final Pose2d goalPose = new Pose2d(new Translation2d(this.lastTargetRange, this.lastVisionTargetAngle), new Rotation2d());
      final Pose2d currentSwervePose = swerve.getPose();

      final Transform2d lastTransform = new Transform2d(
          new Pose2d(this.swerevPoseAtLastVisionUpdate.getTranslation(), new Rotation2d()),
          goalPose
      );

      final Transform2d currentTransform = new Transform2d(
          new Pose2d(currentSwervePose.getTranslation(), new Rotation2d()),
          goalPose
      );

      // acos((a^2 + b^2 - c^2) / 2ab) = theta
      double a = Math.abs(this.swerevPoseAtLastVisionUpdate.getTranslation().getDistance(goalPose.getTranslation()));
      double b = Math.abs(this.swerevPoseAtLastVisionUpdate.getTranslation().getDistance(currentSwervePose.getTranslation()));
      double c = Math.abs(currentSwervePose.getTranslation().getDistance(goalPose.getTranslation()));
      Rotation2d deltaAngle = new Rotation2d(Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b)));

      SmartDashboard.putNumber("Last to Current Rotation", deltaAngle.getDegrees());
      this.targetAngle = this.lastVisionTargetAngle.rotateBy(deltaAngle);
    }
    SmartDashboard.putNumber("Current Target Angle", targetAngle.getDegrees());
  }
}
