package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/** Releases the intake with the climber */
public class ClimberReleaseCommand extends CommandBase {

  // Instance of climber
  private final Climber climber;

  // The timer to wait for the intake to release
  private final Timer releaseTimer = new Timer();

  // Positions to go to to release the intake and stow the climber
  private static final int RELEASE_POSITION = 20523; // ticks
  private static final int STOW_POSITION = 100; // ticks

  // The amount of time to wait for the intake to release
  private static final double RELEASE_TIME = 1.0; // seconds

  // Whether the intake has been released
  private boolean released = false;

  /**
   * Constructs a ClimberReleaseCommand
   *
   * @param climber An instance of Climber
   */
  public ClimberReleaseCommand(Climber climber) {
    // Set climber
    this.climber = climber;

    // Stop and reset releaseTimer
    releaseTimer.stop();
    releaseTimer.reset();

    // Add the climber as a requirement
    addRequirements(climber);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Start running the climber up
    climber.up();
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // If the intake is not released and we have reached RELEASE_POSITION
    if (!released && climber.isAbove(RELEASE_POSITION)) {
      // Stop the climber
      climber.stop();

      // Reset and start releaseTimer
      releaseTimer.reset();
      releaseTimer.start();

      // Set released to true
      released = true;
    }

    // If the releaseTimer has elapsed RELEASE_TIME
    if (releaseTimer.hasElapsed(RELEASE_TIME)) {
      // Start running the climber down
      climber.down();
    }
  }

  /**
   * Returns whether the command is finished or not
   *
   * @return Whether the command is finished or not
   */
  @Override
  public boolean isFinished() {
    // If the intake has been released and we are at STOW_POSITION return true
    return released && climber.isBelow(STOW_POSITION);
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
