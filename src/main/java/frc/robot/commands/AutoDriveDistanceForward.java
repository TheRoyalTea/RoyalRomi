// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveDistanceForward extends CommandBase {
  /** Creates a new AutoDriveDistanceForward. */

  // Drivetrain subsystem for driving
  private Drivetrain drivetrain;

  // Velocity and Distance (Input)
  private double distance;
  private double linearVelocity;

  public AutoDriveDistanceForward(Drivetrain drivetrain, double distance, double linearVelocity) {
    // Gets drivetrain
    this.drivetrain = drivetrain;

    // Gets distance and velocity as input
    this.distance = distance;
    this.linearVelocity = linearVelocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stops motors in the start
    drivetrain.arcadeDrive(0, 0);

    // Resets encoders
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Moves using the velocity provided
    drivetrain.arcadeDrive(linearVelocity, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops robot
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Accounts for different readings in each wheel
    double avgDisplacementTraveled = (drivetrain.leftWheelDistanceTraveledInch() + drivetrain.rightWheelDistanceTraveledInch()) / 2;
    
    // Converts displacement to distance and checks if we have traveled more than the distance we wanted
    return Math.abs(avgDisplacementTraveled) >= distance;
  }
}
