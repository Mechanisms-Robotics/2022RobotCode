package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

import java.util.function.Supplier;

/** This command runs the shooter, accelerator, and feeder to shoot a ball */
public class AutoShootCommand extends CommandBase {
  private final Shooter shooter;
  private final Accelerator accelerator;
  private final Feeder feeder;


  // FOR READ ONLY
  private final Turret turret;
  private final Limelight limelight;
  private final Swerve swerve;

  private boolean spunUp = false;

  // The maximum range to shoot from
  private static final double MAX_RANGE = 1.2; // meters
  private static final double MAX_VELOCITY = 1.0; // m/s
  private static final double MAX_ANGULAR_VELOCITY = 0.1; // rads/s

  /**
   * Constructs a ShootCommand
   *
   * @param shooter Instance of Shooter
   * @param accelerator Instance of Accelerator
   * @param feeder Instance of Feeder
   */
  public AutoShootCommand(
      Shooter shooter,
      Accelerator accelerator,
      Feeder feeder,
      Turret turret,
      Limelight limelight,
      Swerve swerve
      ) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    this.feeder = feeder;
    this.turret = turret;
    this.limelight = limelight;
    this.swerve = swerve;

    // Add the shooter, accelerator, and feeder as a requirement
    addRequirements(shooter, accelerator, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var target = limelight.getCurrentTarget();
    if (target.hasTarget && target.range <= MAX_RANGE) {
      shooter.shoot(target.range);
      accelerator.shoot();

      if (spunUp) {
        final boolean validSwerveSpeed = Math.abs(swerve.getVelocity()) <= MAX_VELOCITY;
        final boolean validSwerveRotationSpeed = Math.abs(swerve.getAngularVelocity()) <= MAX_ANGULAR_VELOCITY;
        if (validSwerveSpeed && validSwerveRotationSpeed && turret.isAimed()) {
          feeder.shoot();
        } else {
          feeder.stop();
        }
      } else {
        spunUp = shooter.atSpeed();
        feeder.stop();
      }
    } else {
      shooter.stop();
      accelerator.stop();
      feeder.stop();
      spunUp = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    accelerator.stop();
    shooter.stop();
  }
}
