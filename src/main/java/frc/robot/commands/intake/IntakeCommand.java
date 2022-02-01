package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

  private final Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setOpenLoop(Constants.intakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
