// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  // Stores drivetrain subsystem
  private Drivetrain drivetrain;

  // For joystick input
  private DoubleSupplier linearSupplier;
  private DoubleSupplier rotationalSupplier;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier linearSupplier, DoubleSupplier rotationalSupplier) {
    // Obtains drivetrain subsystem
    this.drivetrain = drivetrain;

    // Gets input source for driving
    this.linearSupplier = linearSupplier;
    this.rotationalSupplier = rotationalSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Starts with motors at rest
    drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets motor velocity depended on the input
    drivetrain.setChassisSpeeds(new ChassisSpeeds(linearSupplier.getAsDouble(), 0, rotationalSupplier.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Sets motors to rest
    drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command should run forever
    return false;
  }
}
