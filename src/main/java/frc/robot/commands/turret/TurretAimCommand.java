package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/**
 * Aims the turret at the goal
 */
public class TurretAimCommand extends CommandBase {

    // Instance of Turret
    private final Turret turret;

    // Suppliers for hasTarget and targetAngle
    private Supplier<Boolean> hasTargetSupplier;
    private Supplier<Double> targetAngleSupplier;

    /**
     * Constructs a TurretAimCommand
     *
     * @param turret An instance of Turret
     */
    public TurretAimCommand(Turret turret, Supplier<Boolean> hasTargetSupplier, Supplier<Double> targetAngleSupplier) {
        // Set turret
        this.turret = turret;

        // Set hasTarget and targetAngle suppliers
        this.hasTargetSupplier = hasTargetSupplier;
        this.targetAngleSupplier = targetAngleSupplier;

        // Add the turret as a requirement
        addRequirements(turret);
    }

    /**
     * Runs periodically while the command is running
     */
    @Override
    public void execute() {
        // Check if we have a target
        if (hasTargetSupplier.get()) {
            // If we have a vision target aim the turret at it
            turret.aim(targetAngleSupplier.get());
        }
    }
}