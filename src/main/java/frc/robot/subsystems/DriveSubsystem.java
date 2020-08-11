/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable {

  // ** HARDWARE **\\

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstants.LEFT_DRIVE_MASTER_CAN_ID);
  private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(DriveConstants.LEFT_DRIVE_SLAVE_CAN_ID);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveConstants.RIGHT_DRIVE_MASTER_CAN_ID);
  private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(DriveConstants.RIGHT_DRIVE_SLAVE_CAN_ID);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // **TRAJECTORY**\\
  DifferentialDriveOdometry odometry;

  SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(PIDConstants.KS_FEEDFOWARD,
      PIDConstants.KV_FEEDFOWARD, PIDConstants.KA_FEEDFOWARD);
  PIDController leftPIDController = new PIDController(4, 0, 4.51);
  PIDController rightPIDController = new PIDController(4, 0, 4.51);
  Pose2d pose;

  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    // check the ramp doesnt seem to work
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
        new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading())));

  }

  /**
   * 
   * @return Returns the differential drive object
   */
  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {

    differentialDrive.arcadeDrive(fwd, rot);
    
  }


  public void stopMotors(){
    differentialDrive.stopMotor();
  }
  @Log
  public double getHeading() {

    // negative because of the unit circle
    // IEEEremainder is pretty much just modulo
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.IS_GYRO_REVERSED_FOR_PATHWEAVER ? -1.0 : 1.0);

  }

  @Log
  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public ADXRS450_Gyro getGyro() {
    return gyro;
  }

  public double getAverageEncoderDistanceMeters() {
    return (getLeftWheelPositionMeters() + getRightWheelPositionMeters() / 2.0);
  }


  /**
   * Returns the left wheel's position in meters
   * 
   * @return Wheel position in meters
   */
  @Log
  public double getLeftWheelPositionMeters() {

    return
    // 1 or 10?
    // get rid of magic numbers
    leftMaster.getSelectedSensorPosition() * DriveConstants.INVERT * DriveConstants.TICKS_TO_REVOLUTIOIN_MAG_ENCODER
        * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;

  }

  /**
   * Returns the right wheel's position in meters
   * 
   * @return Wheel position in meters
   */
  @Log
  public double getRightWheelPositionMeters() {

    return
    // removedd the gear ratio because the position is is reading from the axel
    rightMaster.getSelectedSensorPosition(0) * DriveConstants.TICKS_TO_REVOLUTIOIN_MAG_ENCODER
        * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;

  }

  public SimpleMotorFeedforward getFeedfoward() {

    return driveFeedforward;

  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {

    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));

  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {

    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);

  }

  /**
   * Zeroes the heading of the robot.
   */
  public void resetGyro() {

    gyro.reset();

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {

    return odometry.getPoseMeters();

  }

  public void setOutputVoltage(double leftVolts, double rightVolts) {

    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);

    differentialDrive.feed();
  }

  public void setVoltageCompensation(boolean isEnabled, double volts) {
    leftMaster.configVoltageCompSaturation(volts);
    leftMaster.enableVoltageCompensation(isEnabled);

    rightMaster.configVoltageCompSaturation(volts);
    rightMaster.enableVoltageCompensation(isEnabled);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(getLeftWheelSpeed(), getrightWheelSpeed());

  }

  /**
   * Returns the left wheel speed in meters per second
   * 
   * @return left wheel speed in meters per second
   */
  public double getLeftWheelSpeed() {

    return leftMaster.getSelectedSensorVelocity(0) * DriveConstants.INVERT
        * DriveConstants.TICKS_TO_REVOLUTION_SECONDS_MAG_ENCODER * (Math.PI * DriveConstants.WHEEL_DIAMETER_METERS);

  }

  /**
   * Returns the right wheel speed in meters per second
   * 
   * @return right wheel speed in meters per second
   */
  public double getrightWheelSpeed() {

    return rightMaster.getSelectedSensorVelocity(0) * DriveConstants.TICKS_TO_REVOLUTION_SECONDS_MAG_ENCODER
        * (Math.PI * DriveConstants.WHEEL_DIAMETER_METERS);

  }

  // public DifferentialDriveKinematics getKinematics() {

  // return diffDriveKine;
  // }

  public PIDController getLeftPIDController() {

    return leftPIDController;

  }

  public PIDController getRightPIDController() {

    return rightPIDController;

  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {

    differentialDrive.setMaxOutput(maxOutput);

  }

  @Override
  public void periodic() {

    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftWheelPositionMeters(), getRightWheelPositionMeters());

  }

}