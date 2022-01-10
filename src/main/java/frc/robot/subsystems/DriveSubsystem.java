package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    public static final int MOTOR_ID = 60;

    private TalonFX motor;

    public DriveSubsystem() {
        motor = new TalonFX(MOTOR_ID);
    }

    public void set(double value) {
        motor.set(ControlMode.PercentOutput, value);
    }

    public void setInverted() {
        if (motor.getInverted())
            motor.setInverted(false);
        else
            motor.setInverted(true);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
