package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Climber;

/**
 * Runs the climber down to pull the robot up
 */
public class ClimberClimbCommand extends CommandBase {

    // Instance of climber
    private final Climber climber;

    // Position to go to lift the bot
    private static final int CLIMBED_POSITION = 100000; // ticks

    /**
     * Constructs a ClimberClimbCommand
     *
     * @param climber An instance of Climber
     */
    public ClimberClimbCommand(Climber climber) {
        // Set climber
        this.climber = climber;

        // Add the climber as a requirement
        addRequirements(climber);
    }

    /**
     * Runs when the command is first started
     */
    @Override
    public void initialize() {
        // Start running the climber down
        climber.down();
    }

    /**
     * Returns whether the command is finished or not
     *
     * @return Whether the command is finished or not
     */
    @Override
    public boolean isFinished() {
        // If the climber has reached CLIMBED_POSITION return true
        return climber.isBelow(CLIMBED_POSITION);
    }

    /**
     * Runs when the command ends
     *
     * @param interrupted Whether the command was interrupted or not
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the climber
        climber.stop();
    }
}