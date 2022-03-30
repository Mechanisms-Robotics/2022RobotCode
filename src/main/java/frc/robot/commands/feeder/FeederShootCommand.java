package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Feeder;

/**
 * Feeds the balls into the Accelerator to be shot
 */
public class FeederShootCommand extends CommandBase {

    // Instance of Feeder
    private final Feeder feeder;

    /**
     * Constructs a FeederShootCommand
     *
     * @param feeder An instance of Feeder
     */
    public FeederShootCommand(Feeder feeder) {
        // Set feeder
        this.feeder = feeder;

        // Add the feeder a requirement
        addRequirements(feeder);
    }

    /**
     * Runs when the command is first started
     */
    @Override
    public void initialize() {
        // Run the feeder
        feeder.shoot();
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