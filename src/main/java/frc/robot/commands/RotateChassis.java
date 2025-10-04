package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class RotateChassis extends Command {
  IO io;
  PIDController r = new PIDController(0.01, 0.00, 0.00); // TODO: tune PID
  double angle;

  public RotateChassis(IO io, double angle) {
    this.io = io;
    this.angle = angle;
    addRequirements(io.chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double rotation = io.chassis.getYaw() % 360;
    rotation += (rotation < 0) ? 360 : 0;
    io.chassis.drive(new ChassisSpeeds(0, 0, r.calculate(rotation, angle)));
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return r.atSetpoint();
  }
}
