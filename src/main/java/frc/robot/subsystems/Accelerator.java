package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Units;

public class Accelerator extends SubsystemBase {

  private static final TalonFXConfiguration ACCELERATOR_MOTOR_CONFIG = new TalonFXConfiguration();

  private static final double SHOOT_SPEED = 500; // 500
  private static final double BACKUP_SPEED = -0.5;
  private static final double OUTTAKE_SPEED = -0.5;
  private static final double IDLE_SPEED = -0.10;

  private static final double GEAR_RATIO = 2.0 / 1.0; // 2:1

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

  private final WPI_TalonFX acceleratorMotor = new WPI_TalonFX(40);
  private final WPI_TalonFX acceleratorFollowerMotor = new WPI_TalonFX(41);

  public Accelerator() {
    acceleratorMotor.configAllSettings(ACCELERATOR_MOTOR_CONFIG, Constants.startupCanTimeout);
    acceleratorMotor.setInverted(TalonFXInvertType.Clockwise);
    acceleratorMotor.setNeutralMode(NeutralMode.Brake);
    acceleratorMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    acceleratorMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    acceleratorFollowerMotor.configAllSettings(
        ACCELERATOR_MOTOR_CONFIG, Constants.startupCanTimeout);
    //acceleratorFollowerMotor.follow(acceleratorMotor);
    acceleratorFollowerMotor.setInverted(TalonFXInvertType.CounterClockwise);
    acceleratorFollowerMotor.setNeutralMode(NeutralMode.Brake);

    // CAN Bus Usage Optimisation.
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  public void shoot() {
    acceleratorMotor.set(
        ControlMode.Velocity,
        Units.RPMToFalcon(SHOOT_SPEED, GEAR_RATIO));

    acceleratorFollowerMotor.set(
        ControlMode.Velocity,
        Units.RPMToFalcon(SHOOT_SPEED, GEAR_RATIO)
    );
  }

  public void backup() {
    setOpenLoop(BACKUP_SPEED);
  }

  public void outtake() {
    setOpenLoop(OUTTAKE_SPEED);
  }

  public void idle() {
    setOpenLoop(IDLE_SPEED);
  }

  private void setOpenLoop(double percentOutput) {
    acceleratorMotor.set(ControlMode.PercentOutput, percentOutput);
    acceleratorFollowerMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    acceleratorMotor.set(ControlMode.PercentOutput, 0.0);
    acceleratorFollowerMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
