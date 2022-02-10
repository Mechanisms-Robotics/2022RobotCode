package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  private static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = new TalonFXConfiguration();
  private static final double FEEDER_FEED_SPEED = 0.75;
  private static final double FEEDER_INTAKE_SPEED = 0.50;
  private static final double FEEDER_OUTTAKE_SPEED = -0.50;

  static {
    // TODO: Determine how much current the feeder draws nominally and
    final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();
    feederCurrentLimit.currentLimit = 10; // Amps
    feederCurrentLimit.triggerThresholdCurrent = 15; // Amps
    feederCurrentLimit.triggerThresholdTime = 0.5; // sec
    feederCurrentLimit.enable = true;
    FEEDER_MOTOR_CONFIG.supplyCurrLimit = feederCurrentLimit;
  }

  private final WPI_TalonFX feederMotor = new WPI_TalonFX(30);

  public Feeder() {
    feederMotor.configAllSettings(FEEDER_MOTOR_CONFIG, startupCanTimeout);
    feederMotor.setInverted(TalonFXInvertType.Clockwise);
    feederMotor.setNeutralMode(NeutralMode.Coast);
  }

  private void setOpenLoop(double percentOutput) {
    feederMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void intake() {
    this.setOpenLoop(FEEDER_INTAKE_SPEED);
  }

  public void outtake() {
    this.setOpenLoop(FEEDER_OUTTAKE_SPEED);
  }

  public void feed() {
    this.setOpenLoop(FEEDER_FEED_SPEED);
  }

  public void stop() {
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
