package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code responsible for the behaviour of the Hood subsystem */
public class Hood extends SubsystemBase {
  // Hood servo
  private final Servo hoodServo = new Servo(0);

  // Current position of the servo
  private double currentPos = 0.0;

  /** Constructs a Hood */
  public Hood() {
    // Set the bounds of the servo
    hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  /**
   * Aims the hood given a range to the target
   *
   * @param range Range to target
   */
  public void aimHood(double range) {
    // Aim the hood based on an interpolating tree map
  }

  /**
   * Sets the hood to a raw position
   *
   * @param rawPosition Raw position for the hood to go to
   */
  public void setHoodRawPosition(double rawPosition) {
    // Clamp the raw position between the servo bounds
    rawPosition = MathUtil.clamp(rawPosition, -1.0, 1.0);

    // Set speed is the speed of the pwm pulse not the speed of the servo
    // PWM speed commands the servo position.
    hoodServo.setSpeed(rawPosition);

    // Update current position
    currentPos = rawPosition;
  }

  /**
   * Gets the current position of the servo
   *
   * @return Current position of the servo
   */
  public double getCurrentPos() {
    return currentPos;
  }
}
