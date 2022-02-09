package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {

  private static final TalonFXConfiguration SHOOTER_MOTOR_CONFIG = new TalonFXConfiguration();

  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> RANGE_TO_RPM =
      new InterpolatingTreeMap<>();
  private static final double DEFAULT_SHOOTER_VEL = 3000.0;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          0.0, // ks
          0.0, // kv
          0.0 // ka
          );

  static {
    final var shooterCurrentLimit = new SupplyCurrentLimitConfiguration();
    shooterCurrentLimit.currentLimit = 20; // Amps
    shooterCurrentLimit.triggerThresholdCurrent = 25; // Amps
    shooterCurrentLimit.triggerThresholdTime = 0.5; // sec
    shooterCurrentLimit.enable = true;
    SHOOTER_MOTOR_CONFIG.supplyCurrLimit = shooterCurrentLimit;

    RANGE_TO_RPM.put(new InterpolatingDouble(0.0), new InterpolatingDouble(3000.0));
    RANGE_TO_RPM.put(new InterpolatingDouble(20.0), new InterpolatingDouble(3000.0));
  }

  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(50);
  private final WPI_TalonFX shooterFollowerMotor = new WPI_TalonFX(51);

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

  private void setOpenLoop(double percentOutput) {
    shooterMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void shoot(double range) {
    final double velocity = RANGE_TO_RPM.getInterpolated(new InterpolatingDouble(range)).value;
    shooterMotor.set(
        ControlMode.Velocity,
        velocity,
        DemandType.ArbitraryFeedForward,
        feedforward.calculate(velocity));
  }

  public void shoot() {
    shooterMotor.set(
        ControlMode.Velocity,
        DEFAULT_SHOOTER_VEL,
        DemandType.ArbitraryFeedForward,
        feedforward.calculate(DEFAULT_SHOOTER_VEL));
  }

  public void stop() {
    shooterMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
