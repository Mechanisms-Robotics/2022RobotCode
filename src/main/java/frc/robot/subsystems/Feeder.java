package frc.robot.subsystems;

import static frc.robot.Constants.ID.FEEDER_MOTOR_ID;
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

  static {
    // TODO: Determine how much current the feeder draws nominally and
    final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();
    feederCurrentLimit.currentLimit = 10; // Amps
    feederCurrentLimit.triggerThresholdCurrent = 15; // Amps
    feederCurrentLimit.triggerThresholdTime = 0.5; // sec
    feederCurrentLimit.enable = true;
    FEEDER_MOTOR_CONFIG.supplyCurrLimit = feederCurrentLimit;
  }

  private final WPI_TalonFX feederMotor = new WPI_TalonFX(FEEDER_MOTOR_ID);

  public Feeder() {
    feederMotor.configAllSettings(FEEDER_MOTOR_CONFIG, startupCanTimeout);
    feederMotor.setInverted(TalonFXInvertType.Clockwise);
    feederMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setOpenLoop(double percentOutput) {
    feederMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
