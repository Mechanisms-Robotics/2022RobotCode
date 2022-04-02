package frc.robot.commands.accelerator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import java.util.function.Supplier;

/** Runs the accelerator to speed up balls and feed them into the shooter */
public class AcceleratorShootCommand extends CommandBase {

  // Instance of Accelerator
  private final Accelerator accelerator;

  // Supplier of shooterRPM
  private final Supplier<Double> shooterRPMSupplier;

  // Percent of shooter RPM to run accelerator at
  private static final double ACCELERATOR_RPM_SCALAR = 0.4;

  /**
   * Constructs an AcceleratorShootCommand
   *
   * @param accelerator An instance of Accelerator
   */
  public AcceleratorShootCommand(Accelerator accelerator, Supplier<Double> shooterRPMSupplier) {
    // Set accelerator
    this.accelerator = accelerator;

    // Set shooter RPM supplier
    this.shooterRPMSupplier = shooterRPMSupplier;

    // Add the accelerator a requirement
    addRequirements(accelerator);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Run the accelerator
    accelerator.shoot(shooterRPMSupplier.get() * ACCELERATOR_RPM_SCALAR);
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the accelerator
    accelerator.stop();
  }
}
