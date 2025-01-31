package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class AimAndRangeCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final CommandPS5Controller controller;

  public AimAndRangeCommand(SwerveSubsystem swerveSubsystem, CommandPS5Controller dsController) {
    this.swerveSubsystem = swerveSubsystem;
    this.controller = dsController;
    addRequirements(swerveSubsystem);
  }

  // simple proportional turning control with Limelight.
  private double limelight_aim_proportional() {
    double kP = .035;
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    targetingAngularVelocity *= swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();
    targetingAngularVelocity *= -1.0;
    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  private double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  @Override
  public void execute() {
    var xSpeed = MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND) * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    var ySpeed = MathUtil.applyDeadband(controller.getLeftX(), 0.02) * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    var rot = MathUtil.applyDeadband(controller.getRightX(), 0.02) * swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();

    rot = limelight_aim_proportional();
    xSpeed = limelight_range_proportional();
    swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rot, false);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
