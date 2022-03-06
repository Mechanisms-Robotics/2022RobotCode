package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class PreAimCommand extends CommandBase {

  private final double preAimAngle;
  private final double preAimRange;
  private final Hood hood;
  private final Turret turret;

  public PreAimCommand(Hood hood, Turret turret, double preAimAngle, double preAimRange) {
    this.preAimAngle = preAimAngle;
    this.preAimRange = preAimRange;
    this.hood = hood;
    this.turret = turret;
    addRequirements(turret, hood);
  }

  @Override
  public void initialize() {
    hood.aim(preAimRange);
    turret.snapTo(preAimAngle);
  }

  @Override
  public boolean isFinished() {
    return turret.isAimed();
  }
}
