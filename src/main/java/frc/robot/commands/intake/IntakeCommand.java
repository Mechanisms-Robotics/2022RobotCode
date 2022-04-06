package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** Runs the intake */
public class IntakeCommand extends CommandBase {

  // Instance of intake
  private final Intake intake;

  /**
   * Constructs an IntakeCommand
   *
   * @param intake An instance of Intake
   */
  public IntakeCommand(Intake intake) {
    // Set intake
    this.intake = intake;

    // Add the intake as a requirement
    addRequirements(intake);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Run the intake wheels
    intake.intake();
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the intake wheels
    intake.stop();
  }
}
