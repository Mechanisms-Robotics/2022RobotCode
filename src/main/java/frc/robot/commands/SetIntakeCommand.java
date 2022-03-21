package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntakeCommand extends CommandBase {

    public enum IntakeMode {
        DEPLOY,
        RETRACT
    }

    private final IntakeMode mode;
    private final Intake intake;

    public SetIntakeCommand(Intake intake, IntakeMode mode) {
        this.intake = intake;
        this.mode = mode;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (mode == IntakeMode.DEPLOY) {
            intake.deploy();
        } else {
            intake.retract();
        }
    }

}
