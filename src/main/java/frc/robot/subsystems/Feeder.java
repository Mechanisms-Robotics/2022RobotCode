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

public class Feeder extends SubsystemBase {

  private static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final double FEEDER_SHOOT_SPEED = 0.50;
  private static final double FEEDER_INTAKE_SPEED = 1.0;
  private static final double FEEDER_OUTTAKE_SPEED = -1.0;
  private static final double FEEDER_BACKUP_SPEED = -0.5;

  static {
    // TODO: Determine how much current the feeder draws nominally and
    final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();
    feederCurrentLimit.currentLimit = 30; // Amps
    feederCurrentLimit.triggerThresholdCurrent = 35; // Amps
    feederCurrentLimit.triggerThresholdTime = 0.25; // sec
    feederCurrentLimit.enable = true;
    FEEDER_MOTOR_CONFIG.supplyCurrLimit = feederCurrentLimit;
  }

  private final WPI_TalonFX feederMotor = new WPI_TalonFX(30);

  public Feeder() {
    feederMotor.configAllSettings(FEEDER_MOTOR_CONFIG, startupCanTimeout);
    feederMotor.setInverted(TalonFXInvertType.Clockwise);
    feederMotor.setNeutralMode(NeutralMode.Coast);
    feederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    feederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  private void setOpenLoop(double percentOutput) {
    feederMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void intake() {
    setOpenLoop(FEEDER_INTAKE_SPEED);
  }

  public void outtake() {
    setOpenLoop(FEEDER_OUTTAKE_SPEED);
  }

  public void shoot() {
    setOpenLoop(FEEDER_SHOOT_SPEED);
  }

  public void backup() {
    setOpenLoop(FEEDER_BACKUP_SPEED);
  }

  public void stop() {
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
