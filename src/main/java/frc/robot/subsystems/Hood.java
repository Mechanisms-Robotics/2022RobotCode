package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

/** This class contains all the code responsible for the behaviour of the Hood subsystem */
public class Hood extends SubsystemBase {
  // Hood servo
  private final Servo hoodServo = new Servo(0);

  // Current position of the servo
  private double currentPos = 0.0;

  // Range interpolating tree map
  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      RANGE_TO_POSITION = new InterpolatingTreeMap<>();


  /** Constructs a Hood */
  public Hood() {
    // Set the bounds of the servo
    hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    // Configure hood range interpolating tree map (meters, hood position)
    // measured from fender to inside of bumper
    RANGE_TO_POSITION.put(new InterpolatingDouble(0.0), new InterpolatingDouble(-0.375));
    RANGE_TO_POSITION.put(new InterpolatingDouble(0.05), new InterpolatingDouble(-0.25));
    RANGE_TO_POSITION.put(new InterpolatingDouble(0.3), new InterpolatingDouble(0.25));
    RANGE_TO_POSITION.put(new InterpolatingDouble(0.6), new InterpolatingDouble(0.4));
    RANGE_TO_POSITION.put(new InterpolatingDouble(1.1), new InterpolatingDouble(0.65));
    RANGE_TO_POSITION.put(new InterpolatingDouble(1.25), new InterpolatingDouble(0.7));
    RANGE_TO_POSITION.put(new InterpolatingDouble(1.5), new InterpolatingDouble(1.0));
    // RANGE_TO_POSITION.put(new InterpolatingDouble(1.72), new InterpolatingDouble(1.0));
    // RANGE_TO_POSITION.put(new InterpolatingDouble(2.82), new InterpolatingDouble(1.0));
    // RANGE_TO_POSITION.put(new InterpolatingDouble(3.0), new InterpolatingDouble(1.0));
    RANGE_TO_POSITION.put(new InterpolatingDouble(20.0), new InterpolatingDouble(1.0));
  }

  /**
   * Aims the hood given a range to the target
   *
   * @param range Range to target
   */
  public void aim(double range) {
    // Calculate interpolated hood position based on range and call setHoodRawPosition
    setHoodRawPosition(RANGE_TO_POSITION.getInterpolated(new InterpolatingDouble(range)).value);
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
