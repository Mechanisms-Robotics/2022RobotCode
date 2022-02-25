package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

/** This class contains all the code responsible for tracking the goal using the Limelight. */
public class GoalTracker extends SubsystemBase {
  // Instance of limelight
  private final PhotonCamera limelight;

  // Constants
  private static final double TARGET_HEIGHT = 2.64; // meters
  private static final double CAMERA_HEIGHT = 1.024; // meters
  private static final double CAMERA_PITCH = Math.toRadians(60.0); // radians

  /** Constructs a GoalTracker */
  public GoalTracker() {
    // Instantiate the limelight and limelight network table
    this.limelight = new PhotonCamera("limelight");

    // Turn off the limelight's LEDs
    turnOnLEDs();
  }

  /**
   * Checks if the limelight has a target
   *
   * @return Whether the limelight has a target or not
   */
  public boolean hasTarget() {
    // Turn on the limelight's LEDs
    turnOnLEDs();

    // Check if the limelight has a target and output it to the SmartDashboard
    boolean hasTarget = this.limelight.getLatestResult().hasTargets();
    SmartDashboard.putBoolean("hasTarget", hasTarget);

    // Return whether the limelight has a target or not
    return hasTarget;
  }

  /**
   * Gets the angle to the target
   *
   * @return Angle to the target
   */
  public double getTargetAngle() {
    // Turn on the limelight's LEDs
    turnOnLEDs();

    // Check if we have a target
    if (hasTarget()) {
      // Get the best target and get the yaw to it
      PhotonTrackedTarget trackedTarget = this.limelight.getLatestResult().getBestTarget();
      double targetYaw = -trackedTarget.getYaw();

      // Output targetYaw to SmartDashboard
      SmartDashboard.putNumber("targetYaw", targetYaw);

      // Return the yaw to the target
      return targetYaw;
    } else {
      // Output zero yaw to the SmartDashboard
      SmartDashboard.putNumber("targetYaw", 0.0);

      // Return zero
      return 0.0;
    }
  }

  /**
   * Gets the range to the target
   *
   * @return The range to the target
   */
  public double getTargetRange() {
    // Turn on the limelight's LEDs
    turnOnLEDs();

    // Check if we have a target
    if (hasTarget()) {
      // Get the best target and calculate the range to it
      PhotonTrackedTarget trackedTarget = this.limelight.getLatestResult().getBestTarget();
      double targetRange =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT,
              TARGET_HEIGHT,
              CAMERA_PITCH,
              Units.degreesToRadians(trackedTarget.getPitch()));

      // Output targetRange to SmartDashboard
      SmartDashboard.putNumber("targetRange", targetRange);

      // Return the range to the target
      return targetRange;
    } else {
      // Output zero range to the SmartDashboard
      SmartDashboard.putNumber("targetRange", 0.0);

      // Return zero
      return 0.0;
    }
  }

  /** Turns on the limelight's LEDs */
  public void turnOnLEDs() {
    this.limelight.setLED(VisionLEDMode.kOn);
  }

  /** Turns off the limelight's LEDs */
  public void turnOffLEDs() {
    this.limelight.setLED(VisionLEDMode.kOff);
  }
}
