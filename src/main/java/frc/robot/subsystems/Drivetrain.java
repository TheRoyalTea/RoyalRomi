// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.sensors.RomiGyro;

public class Drivetrain extends SubsystemBase {
  // Generates motor objects for drivetrain
  private static Spark motorLeft = new Spark(DriveConstants.LEFT_MOTOR_PORT);
  private static Spark motorRight = new Spark(DriveConstants.RIGHT_MOTOR_PORT);

  // Feedforward to calculate voltage needed to achieve desired 
  private static SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveConstants.STATIC_COEFFICIENT, DriveConstants.VELOCITY_COEFFICIENT, DriveConstants.ACCELERATION_COEFFICIENT);

  // PID Controllers to correct for errors in wheel voltages
  private static PIDController leftWheelController = new PIDController(DriveConstants.LEFT_WHEEL_PROPORTIONAL_GAINS, 0, 0);
  private static PIDController rightWheelController = new PIDController(DriveConstants.RIGHT_WHEEL_PROPORTIONAL_GAINS, 0, 0);;

  // Generates encoder objects to measure wheel distance
  private static Encoder encoderLeft = new Encoder(DriveConstants.LEFT_ENCODER_PORT_A, DriveConstants.LEFT_ENCODER_PORT_B);
  private static Encoder encoderRight = new Encoder(DriveConstants.RIGHT_ENCODER_PORT_A, DriveConstants.RIGHT_ENCODER_PORT_B);

  // Generates gyro object to measure robot heading
  private static RomiGyro gyro = new RomiGyro();

  // Object for conversion between robot velocity and wheel velocity (meters)
  private static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.ROBOT_TRACKWIDTH_INCHES));

  // Generates odometry object to keep track of where the robot is located at all times (starts at (3, 3, 0 degrees))
  private static DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d(3, 3, new Rotation2d()));

  // Field object to display robot location
  private static Field2d field = new Field2d();

  // Ramsete controller for drivetrain trajectory tracking
  private static RamseteController trajectoryController = new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA);

  // Obtains network table for robot
  private static NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
  private static NetworkTable networkTable = networkInstance.getTable(DriveConstants.DRIVE_NETWORK_TABLE); 

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Inverts right motor
    motorRight.setInverted(true);

    // Sets encoder distance per pulse to measure in meters
    encoderLeft.setDistancePerPulse((Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER_INCHES) * Math.PI) / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION);
    encoderRight.setDistancePerPulse((Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER_INCHES) * Math.PI) / DriveConstants.ENCODER_COUNTS_PER_REVOLUTION);

    // Resets encoder distance readings
    encoderLeft.reset();
    encoderRight.reset();

    // Resets gyro distance reading
    gyro.reset();

    // Resets odometry
    resetOdometry(3, 3, new Rotation2d());

    // Sets initial position on field
    field.setRobotPose(getPosition());
  }

  // Sets the velocities of each motors
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculates voltage required to attain speeds using voltage balance equation
    double leftVoltage = driveFeedforward.calculate(speeds.leftMetersPerSecond);
    double rightVoltage = driveFeedforward.calculate(speeds.rightMetersPerSecond);

    // Error correction for each wheel velocity
    double leftOffset = leftWheelController.calculate(encoderLeft.getRate(), speeds.leftMetersPerSecond - DriveConstants.DRIFT_VOLTAGE_CONSTANT);
    double rightOffset = rightWheelController.calculate(encoderRight.getRate(), speeds.rightMetersPerSecond);

    // Sets speed of each wheel seperately
    motorLeft.setVoltage(leftVoltage + leftOffset);
    motorRight.setVoltage(rightVoltage + rightOffset);
  }

  // Overloaded version of setWheelSpeeds
  public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
    setWheelSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
  }

  // Set the velocity of the chassis itself
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    // Converts chassis velocity to wheel velocities and sets the speeds
    setWheelSpeeds(kinematics.toWheelSpeeds(speeds));
  }

  // Gets left wheel distance traveled (meters)
  public double getLeftDistance() {
    return encoderLeft.getDistance();
  }

  // Gets right wheel distance traveled (meters)
  public double getRightDistance() {
    return encoderRight.getDistance();
  }

  // Gets left wheel velocity (m/s)
  public double getLeftVelocity() {
    return encoderLeft.getRate();
  }

  // Get right wheel velocity (m/s)
  public double getRightVelocity() {
    return encoderRight.getRate();
  }

  public DifferentialDriveWheelSpeeds getWheelVelocity() {
    return new DifferentialDriveWheelSpeeds(encoderLeft.getRate(), encoderRight.getRate());
  }

  // Gets robot's current heading
  public Rotation2d getHeading() {
    // Take negative angle because angles should grow when going left (unit circle)
    return Rotation2d.fromDegrees(-gyro.getAngleZ());
  }

  // Gets robot position
  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }

  // Resets odometry reading
  public void resetOdometry() {
    odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  // Overloads resetOdometry to read forced readings
  public void resetOdometry(double x, double y, Rotation2d heading) {
    odometry.resetPosition(new Pose2d(x, y, heading), getHeading());
  }

  // Gets feedforward controller
  public SimpleMotorFeedforward getDriveFeedforward() {
    return driveFeedforward;
  }

  // Gets left wheel PID controller
  public PIDController getLeftWheelController() {
    return leftWheelController;
  }

  // Gets right wheel PID controller
  public PIDController getRightWheelController() {
    return rightWheelController;
  }

  // Gets kinematic controller
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  // Gets trajectory controller
  public RamseteController getTrajectoryController() {
    return trajectoryController;
  }

  // Gets field object
  public Field2d getField() {
    return field;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Updates odometry
    odometry.update(getHeading(), getLeftDistance(), getRightDistance());

    // Updates position on field
    field.setRobotPose(getPosition());

    // Updates wheel distance driven in meters
    networkTable.getEntry("Left_Drive_Wheel_Distance_Meter").setValue(getLeftDistance());
    networkTable.getEntry("Right_Drive_Wheel_Distance_Meter").setValue(getRightDistance());

    // Updates wheel velocity in meters per second
    networkTable.getEntry("Left_Drive_Velocity_Meter_Per_Second").setValue(getLeftVelocity());
    networkTable.getEntry("Right_Drive_Velocity_Meter_Per_Second").setValue(getRightDistance());

    // Updates chassis velocity in meters per second
    ChassisSpeeds robotSpeeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()));
    networkTable.getEntry("Robot_Drive_Linear_Velocity_Radian_Per_Second").setValue(robotSpeeds.vxMetersPerSecond);
    networkTable.getEntry("Robot_Drive_Rotational_Velocity_Meter_Per_Second").setValue(robotSpeeds.omegaRadiansPerSecond);

    // Updates robot gyro
    networkTable.getEntry("Robot_Heading_Degrees").setValue(getHeading().getDegrees());

    // Updates robot position to smart dashboard table
    SmartDashboard.putData("Robot_Position", field);
  }
}
