package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Units;

import java.util.concurrent.locks.ReentrantLock;

/** This class contains all the code responsible for the behaviour of the intake class */
public class Intake extends SubsystemBase {

  // Intake speeds
  private static final double INTAKE_SPEED = 0.5; // percent
  private static final double OUTTAKE_SPEED = -0.25; // percent

  private static final int INTAKE_RETRACTED_POSITION = 0; // TODO: get values
  private static final int INTAKE_DEPLOYED_POSITION = 0;

  private static final int INTAKE_RETRACTED_GEAR_RATIO = 88; // TODO: validate gear ratio

  // Intake motor configuration
  private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final TalonFXConfiguration INTAKE_RETRACT_MOTOR_CONFIG;

  // Configure the intake current limit
  static {
    final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();
    intakeCurrentLimit.currentLimit = 15; // Amps
    intakeCurrentLimit.triggerThresholdCurrent = 18; // Amps
    intakeCurrentLimit.triggerThresholdTime = 0.25; // sec
    intakeCurrentLimit.enable = true;
    INTAKE_MOTOR_CONFIG.supplyCurrLimit = intakeCurrentLimit;
    INTAKE_RETRACT_MOTOR_CONFIG = INTAKE_MOTOR_CONFIG;

    final SlotConfiguration intakeRetractMMPID = new SlotConfiguration();
    intakeRetractMMPID.kP = 0.1;
    INTAKE_RETRACT_MOTOR_CONFIG.slot1 = intakeRetractMMPID;
    INTAKE_RETRACT_MOTOR_CONFIG.motionCruiseVelocity = 0.0; // TODO: Find values
    INTAKE_RETRACT_MOTOR_CONFIG.motionAcceleration = 0.0;
    INTAKE_RETRACT_MOTOR_CONFIG.motionCurveStrength = 0;
  }

  private boolean deployed = false;

  // Intake motor
  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20);
  private final WPI_TalonFX intakeRetractMotor = new WPI_TalonFX(21); // TODO: rename motor

  /** Constructs an Intake */
  public Intake() {
    // Configure intake motor
    intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIG, startupCanTimeout);
    intakeRetractMotor.configAllSettings(INTAKE_MOTOR_CONFIG, startupCanTimeout);
    intakeMotor.setInverted(TalonFXInvertType.Clockwise);
    intakeRetractMotor.setInverted(TalonFXInvertType.Clockwise);
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeRetractMotor.setNeutralMode(NeutralMode.Coast);

    // CAN bus utilization optimization
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    intakeRetractMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    intakeRetractMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  /**
   * Set the intake motor to run at a desired percentage
   *
   * @param percentOutput The percentage to run the intake motor at
   */
  private void setOpenLoop(double percentOutput) {
    intakeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void deploy() {
    if (!deployed) {
      intakeRetractMotor.set(ControlMode.MotionMagic, INTAKE_DEPLOYED_POSITION);
    }
  }

  public void retract() {
    if (deployed) {
      intakeRetractMotor.set(ControlMode.MotionMagic, INTAKE_RETRACTED_POSITION);
    }
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
