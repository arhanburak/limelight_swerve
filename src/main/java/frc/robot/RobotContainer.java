// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAndRangeCommand;
import frc.robot.commands.driveToDistanceCommand;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandPS5Controller dsController = new CommandPS5Controller(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> dsController.getLeftY(),
      () -> dsController.getLeftX())
      .withControllerRotationAxis(() -> -dsController.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(dsController::getRightX,
      dsController::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {

    // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle); saçma şema
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    dsController.PS().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    dsController.options()
        .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));

    dsController.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    dsController.triangle().onTrue(new driveToDistanceCommand(drivebase));

    dsController.circle().onTrue(new AimAndRangeCommand(drivebase, dsController));

    dsController.cross().whileTrue(
        drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> dsController.getLeftY(),
                () -> dsController.getLeftX())
                .aim(new Pose2d(17.55 / 2, 8.05 / 2, new Rotation2d(0)))
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.5)
                .allianceRelativeControl(true)));

    dsController.square().whileTrue(
        drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> 0.2,
                () -> dsController.getLeftX())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}