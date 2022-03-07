package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code responsible for the behaviour of the intake class */
public class Climber extends SubsystemBase {
  // Climber speeds
  private static final double UP_SPEED = 0.5; // percent
  private static final double DOWN_SPEED = -0.25; // percent

  // Climber motor configuration
  private static final TalonFXConfiguration CLIMBER_MOTOR_CONFIG = new TalonFXConfiguration();

  // Configure the climber current limit
  static {
    final var climberCurrentLimit = new SupplyCurrentLimitConfiguration();
    climberCurrentLimit.currentLimit = 15; // Amps
    climberCurrentLimit.triggerThresholdCurrent = 18; // Amps
    climberCurrentLimit.triggerThresholdTime = 0.25; // sec
    climberCurrentLimit.enable = true;
    CLIMBER_MOTOR_CONFIG.supplyCurrLimit = climberCurrentLimit;
  }

  // Climber motor
  private final WPI_TalonFX climberMotorLeft = new WPI_TalonFX(1);
  private final WPI_TalonFX climberMotorRight = new WPI_TalonFX(2);

  /** Constructs a Climber */
  public Climber() {
    // Configure climber motors
    climberMotorLeft.configAllSettings(CLIMBER_MOTOR_CONFIG, startupCanTimeout);
    climberMotorLeft.setInverted(TalonFXInvertType.Clockwise);
    climberMotorLeft.setNeutralMode(NeutralMode.Brake);

    climberMotorRight.configAllSettings(CLIMBER_MOTOR_CONFIG, startupCanTimeout);
    climberMotorRight.follow(climberMotorLeft);
    climberMotorRight.setInverted(TalonFXInvertType.CounterClockwise);
    climberMotorRight.setNeutralMode(NeutralMode.Brake);

    // CAN bus utilization optimization
    climberMotorLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    climberMotorLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    climberMotorRight.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    climberMotorRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  /**
   * Set the climber motors to run at a desired percentage
   *
   * @param percentOutput The percentage to run the climber motors at
   */
  private void setOpenLoop(double percentOutput) {
    climberMotorLeft.set(ControlMode.PercentOutput, percentOutput);
    climberMotorRight.set(ControlMode.Follower, 70);
  }

  /** Runs the climber at UP_SPEED */
  public void up() {
    setOpenLoop(UP_SPEED);
  }

  /** Runs the climber at DOWN_SPEED */
  public void down() {
    setOpenLoop(DOWN_SPEED);
  }

  /** Stops the climber */
  public void stop() {
    setOpenLoop(0.0);
  }
}