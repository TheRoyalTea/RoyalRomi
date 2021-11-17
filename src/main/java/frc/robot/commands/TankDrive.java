// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  // Stores drivetrain subsystem
  private Drivetrain drivetrain;

  // For joystick input
  private DoubleSupplier leftVelocitySupplier;
  private DoubleSupplier rightVelocitySupplier;

  /** Creates a new TankDrive. */
  public TankDrive(Drivetrain drivetrain, DoubleSupplier leftVelocitySupplier, DoubleSupplier rightVelocitySupplier) {
    // Obtains drivetrain subsystem
    this.drivetrain = drivetrain;

    // Gets input source for driving
    this.leftVelocitySupplier = leftVelocitySupplier;
    this.rightVelocitySupplier = rightVelocitySupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Starts with motors at rest
    drivetrain.setWheelSpeeds(new DifferentialDriveWheelSpeeds());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Converts input to meters per second
    double leftSpeed = leftVelocitySupplier.getAsDouble() * DriveConstants.MAX_VELOCITY;
    double rightSpeed = rightVelocitySupplier.getAsDouble() * DriveConstants.MAX_VELOCITY;

    // Sets motor velocity depended on the input
    drivetrain.setWheelSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Sets motors to rest
    drivetrain.setWheelSpeeds(new DifferentialDriveWheelSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command should run forever
    return false;
  }
}
