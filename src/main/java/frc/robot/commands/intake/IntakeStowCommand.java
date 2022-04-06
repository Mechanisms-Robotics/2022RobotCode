package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** Stows the intake */
public class IntakeStowCommand extends CommandBase {

  // Instance of intake
  private final Intake intake;

  // This timer is used to wait for the motor to start moving before trying to detect stalling
  private final Timer stallTimer = new Timer();

  // The amount of time to wait before trying to detect stalling
  private static final double STALL_WAIT_TIME = 0.5; // seconds
  // The velocity that if dropped below, indicates stalling
  private static final double STALL_VELOCITY = 1000; // ticks/100ms

  /**
   * Constructs an IntakeStowCommand
   *
   * @param intake An instance of Intake
   */
  public IntakeStowCommand(Intake intake) {
    // Set intake
    this.intake = intake;

    // Stop and reset the stall timer
    stallTimer.stop();
    stallTimer.reset();

    // Add the intake as a requirement
    addRequirements(intake);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Check if the intake is deployed
    if (intake.isDeployed()) {
      // If the intake is deployed, stop the wheels and start retracting it
      intake.stop();
      intake.retract();

      // Reset and start the stall timer
      stallTimer.reset();
      stallTimer.start();
    } else {
      // If the intake is already stowed set the intake state to stowed
      intake.setStowed(true);
    }
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if STALL_WAIT_TIME has elapsed
    if (stallTimer.hasElapsed(STALL_WAIT_TIME)) {
      // If the velocity has dropped below STALL_VELOCITY the motor has started stalling
      if (intake.getRetractVelocity() <= STALL_VELOCITY) {
        // Set the intake state to stowed
        intake.setStowed(true);
      }
    }
  }

  /**
   * Returns whether the command is finished or not
   *
   * @return Whether the command is finished or not
   */
  @Override
  public boolean isFinished() {
    // Return true if the intake is stowed
    return intake.isStowed();
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the intake retract motor
    intake.stopRetraction();
    intake.brake();
  }
}
