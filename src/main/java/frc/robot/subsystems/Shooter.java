package frc.robot.subsystems;

import static frc.robot.Constants.ID.SHOOTER_FOLLOWER_MOTOR_ID;
import static frc.robot.Constants.ID.SHOOTER_MOTOR_ID;
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

public class Shooter extends SubsystemBase {

  private static final TalonFXConfiguration SHOOTER_MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    // TODO: Determine how much current the shooter draws nominally and
    final var shooterCurrentLimit = new SupplyCurrentLimitConfiguration();
    shooterCurrentLimit.currentLimit = 10; // Amps
    shooterCurrentLimit.triggerThresholdCurrent = 15; // Amps
    shooterCurrentLimit.triggerThresholdTime = 0.5; // sec
    shooterCurrentLimit.enable = true;
    SHOOTER_MOTOR_CONFIG.supplyCurrLimit = shooterCurrentLimit;
  }

  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(SHOOTER_MOTOR_ID);
  private final WPI_TalonFX shooterFollowerMotor = new WPI_TalonFX(SHOOTER_FOLLOWER_MOTOR_ID);

  public Shooter() {
    shooterMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterMotor.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setNeutralMode(NeutralMode.Coast);

    shooterFollowerMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterFollowerMotor.follow(shooterMotor);
    shooterFollowerMotor.setInverted(InvertType.OpposeMaster);
    shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);

    // CAN Bus Usage Optimisation.
    shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  public void setOpenLoop(double percentOutput) {
    shooterMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    shooterMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
