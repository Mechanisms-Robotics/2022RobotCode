package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/** This command runs the shooter, accelerator, and feeder to shoot a ball */
public class AutoShootCommand extends CommandBase {
  private final Shooter shooter;
  private final Accelerator accelerator;
  private final Feeder feeder;

  // Suppliers of data from the GoalTracker
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetRangeSupplier;
  private final Supplier<Double> turretAimErrorSupplier;
  private final Supplier<Double> swerveVelocitySupplier;

  // Timer for spinup
  private final Timer spinupTimer = new Timer();

  // The amount of time it takes to spinup
  private static final double SPINUP_TIME = 1.0; // seconds

  // The maximum range to shoot from
  private static final double MAX_RANGE = 1.5; // meters
  private static final double MAX_VELOCITY = 1.0; // m/s
  private static final double MAX_TURRET_ERROR = Math.toRadians(3.0); // degrees -> rads

  // Whether we are spinning up or not
  private boolean spinningUp = false;

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
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetRangeSupplier,
      Supplier<Double> turretAimErrorSupplier,
      Supplier<Double> swerveVelocitySupplier) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    this.feeder = feeder;

    this.hasTargetSupplier = hasTargetSupplier;
    this.targetRangeSupplier = targetRangeSupplier;
    this.turretAimErrorSupplier = turretAimErrorSupplier;
    this.swerveVelocitySupplier = swerveVelocitySupplier;

    // Add the shooter, accelerator, and feeder as a requirement
    addRequirements(shooter, accelerator, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (hasTargetSupplier.get()) {
      if (targetRangeSupplier.get() <= MAX_RANGE) {
        shooter.shoot(targetRangeSupplier.get());
        accelerator.shoot();

        if (!spinningUp) {
          spinupTimer.reset();
          spinupTimer.start();

          spinningUp = true;
        } else if (spinupTimer.hasElapsed(SPINUP_TIME)
            && Math.abs(turretAimErrorSupplier.get()) <= MAX_TURRET_ERROR
            && swerveVelocitySupplier.get() <= MAX_VELOCITY) {
          feeder.shoot();
        } else {
          feeder.stop();
        }
      } else {
        spinningUp = false;

        feeder.stop();
        accelerator.stop();
        shooter.stop();
      }
    } else {
      spinningUp = false;

      feeder.stop();
      accelerator.stop();
      shooter.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    accelerator.stop();
    shooter.stop();
  }
}
