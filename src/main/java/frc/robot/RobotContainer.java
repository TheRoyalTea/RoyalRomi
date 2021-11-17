// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.AutoCurveDrive;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // All subsystems generated here
  private final Drivetrain drivetrain = new Drivetrain();

  // All controllers generated here
  private final XboxController driveController = new XboxController(Constants.InputConstants.DRIVER_CONTROLLER_PORT);

  // All tele op commands generated here (inputs negated because y-axis inverted)
  private final TankDrive tankDrive = new TankDrive(drivetrain, () -> -driveController.getY(Hand.kLeft), () -> -driveController.getY(Hand.kRight));

  // All autonomous commands generated here
  private final AutoCurveDrive autoCurveDrive = new AutoCurveDrive(drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configures the axes bindings
    configureAxes();

    // Configure the button bindings
    configureButtonBindings();
  }


  // Holds all axes configurations (analog inputs)
  private void configureAxes() {
    // Makes driving the default command for drivetrain
    drivetrain.setDefaultCommand(tankDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoCurveDrive;
  }

  public Command autoCurve() {
    // Configures the trajectory generator
    TrajectoryConfig config = new TrajectoryConfig(
      TrajectoryConstants.MAX_SPEED, TrajectoryConstants.MAX_ACCELERATION
      ).setKinematics(drivetrain.getKinematics());

    // Resets odometry
    drivetrain.resetOdometry(3, 3, new Rotation2d());

    // Generates the trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      drivetrain.getPosition(), 
      List.of(new Translation2d(3.5, 3.5)),
      new Pose2d(4.3, 4, new Rotation2d()),
      config);

    // Displays trajectory on dashboard
    drivetrain.getField().getObject(TrajectoryConstants.TRAJ_OBJECT).setTrajectory(trajectory);

    // Generates the ramsete command
    RamseteCommand command = new RamseteCommand(
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

    return command.andThen(() -> drivetrain.setWheelSpeeds(new DifferentialDriveWheelSpeeds()));
  }
}
