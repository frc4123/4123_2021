/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  // *IO* \\

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int AUXDRIVER_CONTROLLER_PORT = 1;
  }
  public static final class XboxConstants {

    // Button mappings
    static public int D_PAD = 0;
    static public int A_BUTTON = 1;
    static public int B_BUTTON = 2;
    static public int X_BUTTON = 3;
    static public int Y_BUTTON = 4;
    static public int LB_BUTTON = 5;
    static public int RB_BUTTON = 6;
    static public int BACK_BUTTON = 7;
    static public int START_BUTTON = 8;
    static public int LEFT_STICK = 9;
    static public int RIGHT_STICK = 10;

    // Axis control mappings
    // Notes:
    // - Left and right trigger use axis 3
    // - Left trigger range: 0 to 1
    // - Right trigger range: 0 to -1).
    static public int LEFT_AXIS_X = 6;
    static public int LEFT_AXIS_Y = 1;
    static public int LEFT_TRIGGER_AXIS = 2;
    static public int RIGHT_TRIGGER_AXIS = 3;
    static public int RIGHT_AXIS_X = 4;
    static public int RIGHT_AXIS_Y = 5;

    // Direction pad lookup angles
    static public int POV_UP = 0;
    static public int POV_RIGHT = 90;
    static public int POV_DOWNN = 180;
    static public int POV_LEFT = 270;
    
  }
      public static final class PS4ButtonConstants {
    
        static public int X_BUTTON = 1;
        static public int O_BUTTON = 2;
        static public int SQUARE_BUTTON = 3;
        static public int TRIANGLE_BUTTON = 4;
    
      }
  public static final class LogitecController {

    public static final int ONE_BUTTON = 1;
    public static final int TWO_BUTTON = 2;
    public static final int THREE_BUTTON = 3;
    public static final int FOUR_BUTTON = 4;
    public static final int LB_BUTTON = 5;
    public static final int RB_BUTTON = 6;

  }

  // *Robot Constants* \\

  public static final class DriveConstants {
    public static final int LEFT_DRIVE_MASTER_CAN_ID = 1;
    public static final int LEFT_DRIVE_SLAVE_CAN_ID = 2;
    public static final int RIGHT_DRIVE_MASTER_CAN_ID = 3;
    public static final int RIGHT_DRIVE_SLAVE_CAN_ID = 4;
    //middle to middle of the wheel 
    public static final double TRACK_WIDTH_METERS = 0.638;
    public static final int MAG_ENCODER_CPR = 4096;
    public static final int TALONFX_ENCODER_CPR = 2048;
    public static final double GEAR_RATIO =  11.25; //12:50 => 20:54 on a falconfx gives 14.8 fps 
    public static final double TICKS_TO_REVOLUTION_SECONDS_TALONFX = 10 * (1/TALONFX_ENCODER_CPR);
    // public static final double TICKS_TO_REVOLUTIOIN_MAG_ENCODER = 1/MAG_ENCODER_CPR;
    public static final double TICKS_TO_REVOLUTION_SECONDS_MAG_ENCODER = 10 * (1/MAG_ENCODER_CPR);
    public static final double WHEEL_DIAMETER_METERS = 0.15875;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double MAX_METERS_PER_SECOND = 0.25;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;
    public static final double MAX_VOLTAGE_AUTO = 10;
    public static final double STARTING_POSE_X = 12.8;
    public static final double STARTING_POSE_Y = -5.8;
    public static boolean IS_GYRO_REVERSED_FOR_PATHWEAVER = true;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);
    public static final SimpleMotorFeedforward SIMPLE_MOTOR_FEED_FOWARD = new SimpleMotorFeedforward(
        PIDConstants.KS_FEEDFOWARD, PIDConstants.KV_FEEDFOWARD, PIDConstants.KA_FEEDFOWARD);
  }


  public static final class ShooterConstants {
    public static final int SHOOTER_MASTER_CAN_ID = 7;
    public static final int SHOOTER_SLAVE_CAN_ID = 8;
  }

  public static final class WinchConstants {
    public static final int WINCH_MASTER_CAN_ID = 9;
    public static final int WINCH_SLAVE_CAN_ID = 10;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_CAN_ID = 11;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_ROLLER_CAN_ID = 5;
    public static final int INTAKE_GATE_CAN_ID = 6;
  }

  public static final class HopperConstants {
    public static final int INDEX_MOTOR_CAN_ID = 12;
  }


  public static final class PIDConstants {
    public static final double KS_FEEDFOWARD = 1.2;
    public static final double KV_FEEDFOWARD = 0.329;
    public static final double KA_FEEDFOWARD = 0.0933;
    public static final double OPTIMAL_KP = 4;
    public static final double OPTIMAL_KD = 4.51;
  }

  public static final class AutoAimConstants {
    public static final double KP_ROTATION_AUTOAIM = 0.025;
    public static final double KD_ROTATION_AUTOAIM = 0.0006;
    public static final double ANGLE_TOLERANCE = 1.0; // IN DEGREES
  }

  public static final class VoltageConstants {
    public static final double STOP = 0;
    public static final double INDEX_WHEEL_SPEED = 1.0;
    public static final double ELEVATOR_DOWN_VOLTAGE = -1.0;
    public static final double ELEVATOR_UP_VOLTAGE = 3.0;
    public static final double INTAKE_GATE_DOWN = 1.5;
    public static final double INTAKE_GATE_UP_VOLTAGE = -4.5;
    public static final double WINCH_DOWN_VOLTAGE = -4.0;
    public static final double WINCH_UP_VOLTAGE = 11.5;
    public static final double INTAKE_WHEEL_SPEED_OUT = .5;
    public static final double INTAKE_WHEEL_SPEED_IN = .45;
    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        DriveConstants.SIMPLE_MOTOR_FEED_FOWARD, DriveConstants.DRIVE_KINEMATICS, DriveConstants.MAX_VOLTAGE_AUTO);
    public static final double TURN_VOLTAGE_COMPENSATION_VOLTS = 5;

  }
}