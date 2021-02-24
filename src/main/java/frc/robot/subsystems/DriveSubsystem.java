/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static java.lang.Math.IEEEremainder;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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

  //  private final WPI_TalonSRX leftMaster_ = new WPI_TalonFX(DriveConstants.LEFT_DRIVE_MASTER_CAN_ID);
  //  private final WPI_Victor rightMaster_ = new WPI_TalonFX(DriveConstants.LEFT_DRIVE_MASTER_CAN_ID);
  //  private final WPI_TalonSRX leftSlave_ = new WPI_TalonFX(DriveConstants.LEFT_DRIVE_MASTER_CAN_ID);
  //  private final WPI_Victor rightSlave_ = new WPI_TalonFX(DriveConstants.LEFT_DRIVE_MASTER_CAN_ID);

  private final WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.LEFT_DRIVE_MASTER_CAN_ID);
  private final WPI_TalonFX  leftSlave = new WPI_TalonFX(DriveConstants.LEFT_DRIVE_SLAVE_CAN_ID);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.RIGHT_DRIVE_MASTER_CAN_ID);
  private final WPI_TalonFX  rightSlave = new WPI_TalonFX(DriveConstants.RIGHT_DRIVE_SLAVE_CAN_ID);

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
    //!Set brake to nuetralmode in firmware

    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    //leftMaster.setSensorPhase(true);
    // leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

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

  public double getHeading() {
    return IEEEremainder(gyro.getAngle(), 360) * 
    (DriveConstants.IS_GYRO_REVERSED_FOR_PATHWEAVER ? -1.0 : 1.0);
  }

  @Log
  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public ADXRS450_Gyro getGyro() {
    return gyro;
  }

  public double getAverageEncoderDistanceMeters() {
    return ((getLeftWheelPositionMeters() + getRightWheelPositionMeters()) / 2.0);
  }

  /**
   * Returns the left wheel's position in meters
   * 
   * @return Left wheel position in meters
   */
  @Log
  public double getLeftWheelPositionMeters() {

    return (leftMaster.getSelectedSensorPosition()  * DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.TALONFX_ENCODER_CPR)
        / DriveConstants.GEAR_RATIO;
    // return  leftMaster.getSelectedSensorPosition() * DriveConstants.INVERT * DriveConstants.TICKS_TO_REVOLUTIOIN_MAG_ENCODER
    // \* DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }
  /**
   * Returns the right wheel's position in meters
   * 
   * @return Left wheel position in meters
   */
  @Log
  public double getRightWheelPositionMeters() {
    return (rightMaster.getSelectedSensorPosition()  * DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.TALONFX_ENCODER_CPR) /
        DriveConstants.GEAR_RATIO;
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
    return new DifferentialDriveWheelSpeeds(getLeftWheelSpeed(), getRightWheelSpeed());
  }

  /**
   * Returns the left wheel speed in meters per second
   * 
   * @return left wheel speed in meters per second
   */
  public double getLeftWheelSpeed() {
    return leftMaster.getSelectedSensorVelocity(0) * 10 / DriveConstants.TALONFX_ENCODER_CPR / DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    // u/100ms *  1000ms/s * 1rev/2048u / 11.25 (GR) * Circumference
    // GR is used to convert from gearbox revolution to axel revolution
  }

  /**
   * Returns the right wheel speed in meters per second
   * 
   * @return right wheel speed in meters per second
   */
  public double getRightWheelSpeed() {
    return rightMaster.getSelectedSensorVelocity(0) * 10 / DriveConstants.TALONFX_ENCODER_CPR / DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
      // u/100ms *  1000ms/s * 1rev/2048u / 11.25 (GR) * Circumference
       // GR is used to convert from gearbox revolution to axel revolution
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