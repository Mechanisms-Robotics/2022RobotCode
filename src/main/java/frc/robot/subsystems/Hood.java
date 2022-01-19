package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private static final int HOOD_SERVO_PWM_PORT = 0;
  private static final double HOOD_STEP_AMOUNT = 0.01;

  private final Servo hoodServo = new Servo(HOOD_SERVO_PWM_PORT);
  private double currentPos = 0.0;

  public Hood() {
    hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  public void setHoodRawPosition(double rawPosition) {
    MathUtil.clamp(rawPosition, -1.0, 1.0);

    // Set speed is the speed of the pwm pulse not the speed of the servo
    // PWM speed commands the servo position.
    hoodServo.setSpeed(rawPosition);
    currentPos = rawPosition;
  }

  public void increaseHood() {
    double currentPos = hoodServo.getSpeed();
    double wantedPos = currentPos + HOOD_STEP_AMOUNT;
    setHoodRawPosition(wantedPos);
  }

  public void decreaseHood() {
    double currentPos = hoodServo.getSpeed();
    double wantedPos = currentPos - HOOD_STEP_AMOUNT;
    setHoodRawPosition(wantedPos);
  }

  public double getCurrentPos() {
    return currentPos;
  }
}
