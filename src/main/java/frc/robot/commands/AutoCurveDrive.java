// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoCurveDrive extends CommandBase {

  // Holds the drivetrain subsystem to move the robot
  private Drivetrain drivetrain;

  // Holds the configurations to the trajectory
  private TrajectoryConfig config;

  // Holds the trajectory data
  private Trajectory trajectory;

  // Holds the trajectory specific command
  private RamseteCommand command;
  /** Creates a new AutoCurveDrive. */
  public AutoCurveDrive(Drivetrain drivetrain) {
    // Gets the drivetrain subsystem as input
    this.drivetrain = drivetrain;

    // Configures the trajectory generator
    this.config = new TrajectoryConfig(
      TrajectoryConstants.MAX_SPEED, TrajectoryConstants.MAX_ACCELERATION
      ).setKinematics(drivetrain.getKinematics());

    // Resets odometry
    drivetrain.resetOdometry(3, 3, new Rotation2d());

    // Generates the trajectory
    this.trajectory = TrajectoryGenerator.generateTrajectory(
      drivetrain.getPosition(), 
      List.of(new Translation2d(3.5, 3.5)),
      new Pose2d(4.3, 4, new Rotation2d()),
      config);

    // Generates the ramsete command
    this.command = new RamseteCommand(
      trajectory, 
      drivetrain::getPosition, 
      drivetrain.getTrajectoryController(), 
      drivetrain.getDriveFeedforward(), 
      drivetrain.getKinematics(), 
      drivetrain::getWheelVelocity, 
      drivetrain.getLeftWheelController(),
      drivetrain.getRightWheelController(), 
      drivetrain::setWheelSpeeds, 
      drivetrain);

      drivetrain.getField().getObject(TrajectoryConstants.TRAJ_OBJECT).setTrajectory(trajectory);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Displays trajectory on dashboard
    drivetrain.getField().getObject(TrajectoryConstants.TRAJ_OBJECT).setTrajectory(trajectory);

    // Runs the initialize method from the ramsete command
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Runs the execute method from the ramsete command
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Removes the trajectory from display
    drivetrain.getField().getObject(TrajectoryConstants.TRAJ_OBJECT).setTrajectory(new Trajectory());
    
    // Stops robot movement
    drivetrain.setWheelSpeeds(new DifferentialDriveWheelSpeeds());
    
    // Runs the end method from the ramsete command
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Runs the isFinished method from the ramsete command
    return command.isFinished();
  }
}
