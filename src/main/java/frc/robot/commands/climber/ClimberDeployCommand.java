package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/** Deploys the climber */
public class ClimberDeployCommand extends CommandBase {

  // Instance of climber
  private final Climber climber;

  // Position to go to deploy the climber
  private static final int DEPLOY_POSITION = 200000; // ticks

  /**
   * Constructs a ClimberDeployCommand
   *
   * @param climber An instance of Climber
   */
  public ClimberDeployCommand(Climber climber) {
    // Set climber
    this.climber = climber;

    // Add the climber as a requirement
    addRequirements(climber);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Start running the climber up
    climber.up();
  }

  /**
   * Returns whether the command is finished or not
   *
   * @return Whether the command is finished or not
   */
  @Override
  public boolean isFinished() {
    // If the climber has reached DEPLOY_POSITION return true
    return climber.isAbove(DEPLOY_POSITION);
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
