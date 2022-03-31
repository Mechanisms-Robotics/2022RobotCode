package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

/** Automatically intakes balls into the feeder as they come in */
public class FeederIntakeCommand extends CommandBase {

  // Instance of Feeder
  private final Feeder feeder;

  // Whether a ball has been detected at the entry sensor or not
  private boolean ballDetected = false;

  /**
   * Constructs a FeederIntakeCommand
   *
   * @param feeder An instance of Feeder
   */
  public FeederIntakeCommand(Feeder feeder) {
    // Set feeder
    this.feeder = feeder;

    // Add the feeder a requirement
    addRequirements(feeder);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // If the entry sensor is triggered set ballDetected to true
    if (feeder.getEntrySensor()) {
      ballDetected = true;
    }

    if (ballDetected && !feeder.getLowSensor()) {
      // If we've detected a ball and there isn't one in front of the low sensor

      // Run the feeder
      feeder.intake();
    } else if (ballDetected && !feeder.getEntrySensor() && feeder.getLowSensor()) {
      // If we detected a ball and there is now one in front of the low sensor but not in front of
      // the entry sensor

      // Stop the feeder
      feeder.stop();

      // Set ballDetected to false
      ballDetected = false;
    } else if (ballDetected && feeder.getLowSensor() && !feeder.getHighSensor()) {
      // If we have detected a ball and there is one in front of the low sensor but not in front of
      // the high sensor

      // Run the feeder
      feeder.intake();
    } else if (ballDetected && feeder.getHighSensor()) {
      // If we have detected a ball but there is now one in front of the high sensor

      // Stop the feeder
      feeder.stop();

      // Set ballDetected to false
      ballDetected = false;
    }
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the feeder
    feeder.stop();
  }
}
