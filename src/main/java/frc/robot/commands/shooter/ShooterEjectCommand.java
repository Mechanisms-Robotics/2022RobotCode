package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/**
 * Ejects a ball out the shooter
 */
public class ShooterEjectCommand extends CommandBase {

    // Instance of Shooter
    private final Shooter shooter;

    /**
     * Constructs a ShooterEjectCommand
     *
     * @param shooter An instance of Shooter
     */
    public ShooterEjectCommand(Shooter shooter) {
        // Set shooter
        this.shooter = shooter;

        // Add the shooter as a requirement
        addRequirements(shooter);
    }

    /**
     * Runs when the command is first started
     */
    @Override
    public void initialize() {
        // Run the shooter at the eject RPM
        shooter.eject();
    }

    /**
     * Runs when the command ends
     *
     * @param interrupted Whether the command was interrupted or not
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the shooter
        shooter.stop();
    }
}