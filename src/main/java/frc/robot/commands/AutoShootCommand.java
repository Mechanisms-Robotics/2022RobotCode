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

  // Timer for spinup
  private final Timer spinupTimer = new Timer();

  // The amount of time it takes to spinup
  private static final double SPINUP_TIME = 1.0; // seconds

  // The maximum range to shoot from
  private static final double MAX_RANGE = 3.0; // meters

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
      Supplier<Double> targetRangeSupplier) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    this.feeder = feeder;

    this.hasTargetSupplier = hasTargetSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

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
        } else if (spinupTimer.hasElapsed(SPINUP_TIME)) {
          feeder.shoot();
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
