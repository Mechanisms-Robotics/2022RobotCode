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
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code responsible for the behaviour of the intake class */
public class Intake extends SubsystemBase {

  // Intake speeds
  private static final double INTAKE_SPEED = 0.5; // percent
  private static final double OUTTAKE_SPEED = -0.25; // percent

  private static final int INTAKE_ALLOWABLE_ERROR = 3000; // ticks

  // Intake motor configuration
  private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final TalonFXConfiguration INTAKE_RETRACT_MOTOR_CONFIG =
      new TalonFXConfiguration();

  // Configure the intake current limit
  static {
    final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();
    intakeCurrentLimit.currentLimit = 15; // Amps
    intakeCurrentLimit.triggerThresholdCurrent = 18; // Amps
    intakeCurrentLimit.triggerThresholdTime = 0.25; // sec
    intakeCurrentLimit.enable = true;

    INTAKE_MOTOR_CONFIG.supplyCurrLimit = intakeCurrentLimit;
    INTAKE_MOTOR_CONFIG.reverseSoftLimitEnable = false;
    INTAKE_MOTOR_CONFIG.forwardSoftLimitEnable = false;

    final SlotConfiguration intakeRetractPID = new SlotConfiguration();
    intakeRetractPID.kP = 0.0;
    intakeRetractPID.allowableClosedloopError = INTAKE_ALLOWABLE_ERROR;

    INTAKE_RETRACT_MOTOR_CONFIG.slot1 = intakeRetractPID;
    INTAKE_RETRACT_MOTOR_CONFIG.reverseSoftLimitEnable = false;
    INTAKE_RETRACT_MOTOR_CONFIG.forwardSoftLimitEnable = false;

    INTAKE_RETRACT_MOTOR_CONFIG.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    INTAKE_RETRACT_MOTOR_CONFIG.velocityMeasurementWindow = 4;
  }

  // Intake motor
  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20);
  private final WPI_TalonFX intakeRetractMotor = new WPI_TalonFX(21);

  /** Constructs an Intake */
  public Intake() {
    // Configure intake motor
    intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIG, startupCanTimeout);
    intakeMotor.setInverted(TalonFXInvertType.Clockwise);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeRetractMotor.configAllSettings(INTAKE_RETRACT_MOTOR_CONFIG, startupCanTimeout);
    intakeRetractMotor.setInverted(TalonFXInvertType.Clockwise);
    intakeRetractMotor.setNeutralMode(NeutralMode.Brake);

    // CAN bus utilization optimization
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

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
    // TODO: Change to closed loop
    intakeRetractMotor.set(ControlMode.PercentOutput, -0.25);
    //    intakeRetractMotor.set(ControlMode.Position, INTAKE_DEPLOYED_POSITION);
  }

  public void retract() {
    // TODO: Change to closed loop
    intakeRetractMotor.set(ControlMode.PercentOutput, 0.35);
    //    intakeRetractMotor.set(ControlMode.Position, INTAKE_RETRACTED_POSITION);
  }

  public double getVelocity() {
    return intakeRetractMotor.getSelectedSensorVelocity();
  }

  /** Runs the intake at INTAKE_SPEED */
  public void intake() {
    setOpenLoop(INTAKE_SPEED);
  }

  /** Runs the intake at OUTTAKE_SPEED */
  public void outtake() {
    setOpenLoop(OUTTAKE_SPEED);
  }

  public void coast() {
    intakeRetractMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    intakeRetractMotor.setNeutralMode(NeutralMode.Brake);
  }

  /** Stops the intake */
  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /** Stops the intake */
  public void stopRetractMotor() {
    intakeRetractMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
