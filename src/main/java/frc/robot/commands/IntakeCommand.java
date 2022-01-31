package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.deploy();
    }

    @Override
    public void execute() {
        intake.setOpenLoop(-0.4);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intake.retract();
    }
}
