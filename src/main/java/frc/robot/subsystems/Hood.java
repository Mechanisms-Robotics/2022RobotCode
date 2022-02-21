package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final Servo hoodServo = new Servo(0);
  private double currentPos = 0.0;

  public Hood() {
    hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  public void aimHood(double range) {
    // Aim the hood based on an interpolating tree map
  }

  public void setHoodRawPosition(double rawPosition) {
    rawPosition = MathUtil.clamp(rawPosition, -1.0, 1.0);

    // Set speed is the speed of the pwm pulse not the speed of the servo
    // PWM speed commands the servo position.
    hoodServo.setSpeed(rawPosition);
    currentPos = rawPosition;
  }

  public double getCurrentPos() {
    return currentPos;
  }
}
