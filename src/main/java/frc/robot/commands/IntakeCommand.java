package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

import java.util.function.Supplier;

public class IntakeCommand extends CommandBase {

    private final Intake intake;
    private final Supplier<Boolean> toggleIntake;
    private boolean isIntaking = true;
    private boolean prevToggleIntake = false;

    public IntakeCommand(Supplier<Boolean> toggleIntake, Intake intake) {
        this.intake = intake;
        this.toggleIntake = toggleIntake;
    }

    @Override
    public void initialize() {
        //intake.deploy();
    }

    @Override
    public void execute() {
//        if (toggleIntake.get() && !prevToggleIntake) {
//            isIntaking = !isIntaking;
//            prevToggleIntake = true;
//        } else if (!toggleIntake.get() && prevToggleIntake) {
//            prevToggleIntake = false;
//        }
//
//        if (isIntaking) {
//            intake.setOpenLoop(Constants.intakeSpeed);
//        } else {
//            intake.stop();
//        }
        intake.setOpenLoop(Constants.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        //intake.retract();
    }
}
