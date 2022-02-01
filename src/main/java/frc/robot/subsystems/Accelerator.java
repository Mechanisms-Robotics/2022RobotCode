package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Accelerator extends SubsystemBase {

  // TODO: Update IDs, if possible
  private static final int ACCELERATOR_MOTOR_ID = 40;
  private static final int ACCELERATOR_FOLLOWER_MOTOR_ID = 41;

  private static final TalonFXConfiguration ACCELERATOR_MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    // TODO: Determine how much current the accelerator draws nominally and
    final var acceleratorCurrentLimit = new SupplyCurrentLimitConfiguration();
    acceleratorCurrentLimit.currentLimit = 10; // Amps
    acceleratorCurrentLimit.triggerThresholdCurrent = 15; // Amps
    acceleratorCurrentLimit.triggerThresholdTime = 0.5; // sec
    acceleratorCurrentLimit.enable = true;
    ACCELERATOR_MOTOR_CONFIG.supplyCurrLimit = acceleratorCurrentLimit;
  }

  private final WPI_TalonFX acceleratorMotor = new WPI_TalonFX(ACCELERATOR_MOTOR_ID);
  private final WPI_TalonFX acceleratorFollowerMotor =
      new WPI_TalonFX(ACCELERATOR_FOLLOWER_MOTOR_ID);

  public Accelerator() {
    acceleratorMotor.configAllSettings(ACCELERATOR_MOTOR_CONFIG, startupCanTimeout);
    acceleratorMotor.setInverted(TalonFXInvertType.Clockwise);
    acceleratorMotor.setNeutralMode(NeutralMode.Coast);

    acceleratorFollowerMotor.configAllSettings(ACCELERATOR_MOTOR_CONFIG, startupCanTimeout);
    acceleratorFollowerMotor.follow(acceleratorMotor);
    acceleratorFollowerMotor.setInverted(InvertType.OpposeMaster);
    acceleratorFollowerMotor.setNeutralMode(NeutralMode.Coast);

    // CAN Bus Usage Optimisation.
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    acceleratorFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  public void setOpenLoop(double percentOutput) {
    acceleratorMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    acceleratorMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
