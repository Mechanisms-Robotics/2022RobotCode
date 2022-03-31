package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import java.util.function.Supplier;

/** Feeds the balls into the Accelerator to be shot */
public class FeederShootCommand extends CommandBase {

  // Instance of Feeder
  private final Feeder feeder;

  // Supplier of shooterAtSpeed
  private final Supplier<Boolean> shooterAtSpeedSupplier;

  /**
   * Constructs a FeederShootCommand
   *
   * @param feeder An instance of Feeder
   */
  public FeederShootCommand(Feeder feeder, Supplier<Boolean> shooterAtSpeedSupplier) {
    // Set feeder
    this.feeder = feeder;

    // Set shooterAtSpeedSupplier
    this.shooterAtSpeedSupplier = shooterAtSpeedSupplier;

    // Add the feeder a requirement
    addRequirements(feeder);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    if (shooterAtSpeedSupplier.get()) {
      // Run the feeder
      feeder.shoot();
    } else {
      // Stop the feeder
      feeder.stop();
    }
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the feeder
    feeder.stop();
  }
}
