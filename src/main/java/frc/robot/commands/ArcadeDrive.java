// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */

  // Drivetrain subsystem
  private Drivetrain drivetrain;

  // Supplies the value for linear and rotational velocities
  private Supplier<Double> linearVelocitySupplier;
  private Supplier<Double> rotateVelocitySupplier;

  public ArcadeDrive(Drivetrain drivetrain, Supplier<Double> linearVelocity, Supplier<Double> rotationalVelocity) {
    // Drivetrain subsystem saved as field
    this.drivetrain = drivetrain;

    // Gets controller axises as double suppliers
    this.linearVelocitySupplier = linearVelocity;
    this.rotateVelocitySupplier = rotationalVelocity;

    // Requires drivetrain as subsystem
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot uses arcadeDrive function to drive
    this.drivetrain.arcadeDrive(this.linearVelocitySupplier.get(), this.rotateVelocitySupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops any velocity at end of robot
    this.drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Never finishes, needs to run forever
    return false;
  }
}
