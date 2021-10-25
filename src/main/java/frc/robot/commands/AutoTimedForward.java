// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoTimedForward extends CommandBase {
  /** Creates a new AutoTimedForward. */

  // Declares drivetrain subsystem
  private Drivetrain drivetrain;

  // Declares timer to keep track of time
  private Timer timer;

  // States if the robot is done with its journey
  private boolean isDone;

  public AutoTimedForward(Drivetrain drivetrain) {
    // Obtains drivetrain from robot containter
    this.drivetrain = drivetrain;

    // Creates timer
    this.timer = new Timer();

    // Robot hasn't finished it's journey
    this.isDone = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Resets and starts timer
    timer.reset();
    timer.start();

    // Robot is not done
    isDone = false;

    // Sets motors to drive forward at max speed
    drivetrain.arcadeDrive(1, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If timer hasn't reached 1 seconds
    if(timer.get() < 1.0) {
      // Keep going forward
      drivetrain.arcadeDrive(1, 0);
    } else {
      // Updates the code that the robot is done
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops drivetrain motors
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Is not finished till 3 seconds passes on timer
    return isDone;
  }
}
