package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Accelerator extends SubsystemBase {

  private static final TalonFXConfiguration ACCELERATOR_MOTOR_CONFIG = new TalonFXConfiguration();

  private static final double SHOOT_SPEED = 0.60;
  private static final double BACKUP_SPEED = -0.25;
  private static final double IDLE_SPEED = -0.10;

  static {
    // TODO: Determine how much current the accelerator draws nominally and
    final var acceleratorCurrentLimit = new SupplyCurrentLimitConfiguration();
    acceleratorCurrentLimit.currentLimit = 10; // Amps
    acceleratorCurrentLimit.triggerThresholdCurrent = 15; // Amps
    acceleratorCurrentLimit.triggerThresholdTime = 0.5; // sec
    acceleratorCurrentLimit.enable = true;
    ACCELERATOR_MOTOR_CONFIG.supplyCurrLimit = acceleratorCurrentLimit;
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
    acceleratorFollowerMotor.follow(acceleratorMotor);
    acceleratorFollowerMotor.setInverted(InvertType.OpposeMaster);
    acceleratorFollowerMotor.setNeutralMode(NeutralMode.Brake);

    // CAN Bus Usage Optimisation.
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  public void spinup() {
    setOpenLoop(SHOOT_SPEED);
  }

  public void backup() {
    setOpenLoop(BACKUP_SPEED);
  }

  public void idle() {
    setOpenLoop(IDLE_SPEED);
  }

  private void setOpenLoop(double percentOutput) {
    acceleratorMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    acceleratorMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
