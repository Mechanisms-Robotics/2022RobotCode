package frc.robot.commands.accelerator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;

/**
 * Runs the accelerator to speed up balls and feed them into the shooter
 */
public class AcceleratorShootCommand extends CommandBase {

    // Instance of Accelerator
    private final Accelerator accelerator;

    /**
     * Constructs an AcceleratorShootCommand
     *
     * @param accelerator An instance of Accelerator
     */
    public AcceleratorShootCommand(Accelerator accelerator) {
        // Set accelerator
        this.accelerator = accelerator;

        // Add the accelerator a requirement
        addRequirements(accelerator);
    }

    /**
     * Runs when the command is first started
     */
    @Override
    public void initialize() {
        // Run the accelerator
        accelerator.shoot();
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