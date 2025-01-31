// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToDistanceCommand extends Command {
  /** Creates a new driveToDistanceCommand. */
  private final SwerveSubsystem swerveSubsystem;

  private final double distanceInMeters = 1;

  private Pose2d initialPosition;

  public driveToDistanceCommand(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
    this.initialPosition = swerveSubsystem.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(0.4, 0, 0.7));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var currentTranslation = swerveSubsystem.getPose().getTranslation();
    var initialTranslation = this.initialPosition.getTranslation();

    var distance = initialTranslation.getDistance(currentTranslation);

    return distance > this.distanceInMeters;

  }
}
