package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code responsible for the behaviour of the Feeder subsystem */
public class Feeder extends SubsystemBase {

  // Feeder motor configuration
  private static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = new TalonFXConfiguration();

  // Feeder speeds
  private static final double SHOOT_SPEED = 0.75; // percent
  private static final double INTAKE_SPEED = 0.25; // percent
  private static final double OUTTAKE_SPEED = -1.0; // percent
  private static final double BACKUP_SPEED = -0.15; // percent

  // Configure the feeder current limit
  static {
    // TODO: Determine how much current the feeder draws nominally and
    final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();
    feederCurrentLimit.currentLimit = 30; // Amps
    feederCurrentLimit.triggerThresholdCurrent = 35; // Amps
    feederCurrentLimit.triggerThresholdTime = 0.25; // sec
    feederCurrentLimit.enable = true;
    FEEDER_MOTOR_CONFIG.supplyCurrLimit = feederCurrentLimit;
  }

  // Feeder motor
  private final WPI_TalonFX feederMotor = new WPI_TalonFX(30);

  // Proximity sensor
  private final DigitalInput entrySensor = new DigitalInput(0);
  private final DigitalInput lowSensor = new DigitalInput(1);
  private final DigitalInput highSensor = new DigitalInput(2);

  /** Constructs a Feeder */
  public Feeder() {
    // Configure feeder motor
    feederMotor.configAllSettings(FEEDER_MOTOR_CONFIG, startupCanTimeout);
    feederMotor.setInverted(TalonFXInvertType.Clockwise);
    feederMotor.setNeutralMode(NeutralMode.Coast);
    feederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    feederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Entry Sensor", !entrySensor.get());
    SmartDashboard.putBoolean("Low Sensor", !lowSensor.get());
    SmartDashboard.putBoolean("High Sensor", !highSensor.get());
  }

  /**
   * Sets the feeder motor to run at a desired percentage
   *
   * @param percentOutput The percentage to run the feeder motor at
   */
  private void setOpenLoop(double percentOutput) {
    feederMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /** Runs the feeder at INTAKE_SPEED */
  public void intake() {
    setOpenLoop(INTAKE_SPEED);
  }

  /** Runs the feeder at OUTTAKE_SPEED */
  public void outtake() {
    setOpenLoop(OUTTAKE_SPEED);
  }

  /** Runs the feeder at SHOOT_SPEED */
  public void shoot() {
    setOpenLoop(SHOOT_SPEED);
  }

  /** Runs the feeder at BACKUP_SPEED */
  public void backup() {
    setOpenLoop(BACKUP_SPEED);
  }

  /**
   * Returns whether the entry sensor is triggered or not
   *
   * @return Whether the entry sensor is triggered or not
   */
  public boolean getEntrySensor() {
    return !entrySensor.get();
  }

  /**
   * Returns whether the low sensor is triggered or not
   *
   * @return Whether the low sensor is triggered or not
   */
  public boolean getLowSensor() {
    return !lowSensor.get();
  }

  /**
   * Returns whether the high sensor is triggered or not
   *
   * @return Whether the high sensor is triggered or not
   */
  public boolean getHighSensor() {
    return !highSensor.get();
  }

  /** Stops the feeder */
  public void stop() {
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
