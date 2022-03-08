package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class PreAimCommand extends CommandBase {

  protected final double preAimAngle;
  protected final double preAimRange;
  protected final Hood hood;
  protected final Turret turret;
  protected final Shooter shooter;

  public PreAimCommand(Hood hood, Turret turret, Shooter shooter, double preAimAngle, double preAimRange) {
    this.preAimAngle = preAimAngle;
    this.preAimRange = preAimRange;
    this.hood = hood;
    this.turret = turret;
    this.shooter = shooter;
    addRequirements(turret, hood, shooter);
  }

  @Override
  public void initialize() {
    hood.aim(preAimRange);
    turret.snapTo(preAimAngle);
    shooter.shoot(preAimRange);
  }

  @Override
  public boolean isFinished() {
    return turret.isAimed() && shooter.atSpeed();
  }
}
