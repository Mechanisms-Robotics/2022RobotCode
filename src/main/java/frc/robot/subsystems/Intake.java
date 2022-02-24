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
public class Intake extends SubsystemBase {

  // Intake speeds
  private static final double INTAKE_SPEED = 0.40; // percent
  private static final double OUTTAKE_SPEED = -0.25; // percent

  // Intake motor configuration
  private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();

  // Configure the intake current limit
  static {
    final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();
    intakeCurrentLimit.currentLimit = 15; // Amps
    intakeCurrentLimit.triggerThresholdCurrent = 18; // Amps
    intakeCurrentLimit.triggerThresholdTime = 0.25; // sec
    intakeCurrentLimit.enable = true;
    INTAKE_MOTOR_CONFIG.supplyCurrLimit = intakeCurrentLimit;
  }

  // Intake motor
  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20);

  /** Constructs an Intake */
  public Intake() {
    // Configure intake motor
    intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIG, startupCanTimeout);
    intakeMotor.setInverted(TalonFXInvertType.Clockwise);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    // CAN bus utilization optimization
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  /**
   * Set the intake motor to run at a desired percentage
   *
   * @param percentOutput The percentage to run the intake motor at
   */
  private void setOpenLoop(double percentOutput) {
    intakeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /** Runs the intake at INTAKE_SPEED */
  public void intake() {
    setOpenLoop(INTAKE_SPEED);
  }

  /** Runs the intake at OUTTAKE_SPEED */
  public void outtake() {
    setOpenLoop(OUTTAKE_SPEED);
  }

  /** Stops the intake */
  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
