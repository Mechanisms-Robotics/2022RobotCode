package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Units;

/** This contains all the code responsible for the behaviour of the Accelerator subsystem. */
public class Accelerator extends SubsystemBase {

  // Accelerator motor configuration
  private static final TalonFXConfiguration ACCELERATOR_MOTOR_CONFIG = new TalonFXConfiguration();

  // Accelerator speeds
  private static final double SHOOT_SPEED = 750; // RPM
  private static final double BACKUP_SPEED = -0.5; // percent
  private static final double PREP_SPEED = -0.10;
  private static final double OUTTAKE_SPEED = -0.5; // percent
  private static final double IDLE_SPEED = -0.10; // percent

  private static final double GEAR_RATIO = 2.0; // 2:1

  private static final double PREP_SPINUP_TIME = 0.5;
  private static final int PREP_FINISHED_SPEED = Units.RPMToFalcon(50, GEAR_RATIO);
  private boolean preping = false;
  private final Timer prepTimer = new Timer();

  // Configure the accelerator current limits
  static {
    // TODO: Determine how much current the accelerator draws nominally and
    final var acceleratorCurrentLimit = new SupplyCurrentLimitConfiguration();
    acceleratorCurrentLimit.currentLimit = 30; // Amps
    acceleratorCurrentLimit.triggerThresholdCurrent = 35; // Amps
    acceleratorCurrentLimit.triggerThresholdTime = 0.5; // sec
    acceleratorCurrentLimit.enable = true;
    ACCELERATOR_MOTOR_CONFIG.supplyCurrLimit = acceleratorCurrentLimit;

    final var acceleratorPID = new SlotConfiguration();
    acceleratorPID.kP = 0.05;
    acceleratorPID.kF = 0.05;
    ACCELERATOR_MOTOR_CONFIG.slot0 = acceleratorPID;
  }

  // Accelerator master and follower motor
  private final WPI_TalonFX acceleratorMotor = new WPI_TalonFX(40);
  private final WPI_TalonFX acceleratorFollowerMotor = new WPI_TalonFX(41);

  /** Constructs an Accelerator */
  public Accelerator() {
    // Configures accelerator motor
    acceleratorMotor.configAllSettings(ACCELERATOR_MOTOR_CONFIG, Constants.startupCanTimeout);
    acceleratorMotor.setInverted(TalonFXInvertType.Clockwise);
    acceleratorMotor.setNeutralMode(NeutralMode.Brake);
    acceleratorMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    acceleratorMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    // Configures accelerator follower motor
    acceleratorFollowerMotor.configAllSettings(
        ACCELERATOR_MOTOR_CONFIG, Constants.startupCanTimeout);
    // acceleratorFollowerMotor.follow(acceleratorMotor);
    acceleratorFollowerMotor.setInverted(TalonFXInvertType.CounterClockwise);
    acceleratorFollowerMotor.setNeutralMode(NeutralMode.Brake);

    // CAN Bus Usage Optimisation.
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Accelerator RPM",
        Units.falconToRPM(acceleratorMotor.getSelectedSensorVelocity(), GEAR_RATIO));
  }

  /** Runs the accelerator at a specified rpm */
  public void shoot(double rpm) {
    preping = false;
    acceleratorMotor.set(ControlMode.Velocity, Units.RPMToFalcon(rpm, GEAR_RATIO));

    acceleratorFollowerMotor.set(ControlMode.Velocity, Units.RPMToFalcon(rpm, GEAR_RATIO));
  }

  /** Runs the accelerator at SHOOT_SPEED */
  public void shoot() {
    preping = false;
    acceleratorMotor.set(ControlMode.Velocity, Units.RPMToFalcon(SHOOT_SPEED, GEAR_RATIO));

    acceleratorFollowerMotor.set(ControlMode.Velocity, Units.RPMToFalcon(SHOOT_SPEED, GEAR_RATIO));
  }

  /** Runs the accelerator at BACKUP_SPEED */
  public void backup() {
    preping = false;
    setOpenLoop(BACKUP_SPEED);
  }

  /** Runs the accelerator at OUTTAKE_SPEED */
  public void outtake() {
    preping = false;
    setOpenLoop(OUTTAKE_SPEED);
  }

  /** Runs the accelerator at IDLE_SPEED */
  public void idle() {
    preping = false;
    setOpenLoop(IDLE_SPEED);
  }

  public void prep() {
    prepTimer.start();
    prepTimer.reset();
    preping = true;
    setOpenLoop(PREP_SPEED);
  }

  public boolean isPreped() {
    if (preping) {
      if (prepTimer.hasElapsed(PREP_SPINUP_TIME)) {
        return Math.abs(acceleratorMotor.getSelectedSensorVelocity()) <= PREP_FINISHED_SPEED;
      }
    }
    return false;
  }

  /**
   * Sets the accelerator motors to run at a desired percentage
   *
   * @param percentOutput The percentage to run the motors at
   */
  private void setOpenLoop(double percentOutput) {
    acceleratorMotor.set(ControlMode.PercentOutput, percentOutput);
    acceleratorFollowerMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /** Stops the accelerator motors */
  public void stop() {
    preping = false;
    acceleratorMotor.set(ControlMode.PercentOutput, 0.0);
    acceleratorFollowerMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
