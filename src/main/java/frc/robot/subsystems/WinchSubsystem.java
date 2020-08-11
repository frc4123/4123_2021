/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase {
  
  public static final WPI_TalonSRX winchMaster = new WPI_TalonSRX(WinchConstants.WINCH_MASTER_CAN_ID);
  public static final WPI_TalonSRX winchSlave = new WPI_TalonSRX(WinchConstants.WINCH_SLAVE_CAN_ID);
  
  SpeedControllerGroup winchMotors = new SpeedControllerGroup(winchMaster, winchSlave);

  public WinchSubsystem() {
    //moving to speed controller group
   // winchSlave.follow(winchMaster);

    winchMaster.configOpenloopRamp(1);
    winchSlave.configOpenloopRamp(1);

    winchMaster.setNeutralMode(NeutralMode.Brake);
    winchSlave.setNeutralMode(NeutralMode.Brake);

  }

  public void setWinchMotorVoltage(double voltage){
    winchMotors.setVoltage(voltage);
  }
 
}