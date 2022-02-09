package frc.robot.commands.aiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretSetCommand extends CommandBase {

    private final Turret turret;
    private final double degrees;

    public TurretSetCommand(Turret turret, double degrees) {
        this.turret = turret;
        this.degrees = degrees;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setPosition(degrees);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

}
