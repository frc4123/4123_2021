/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */

  private final WPI_TalonSRX shooterTop = new WPI_TalonSRX(ShooterConstants.SHOOTER_MASTER_CAN_ID);
  private final WPI_TalonSRX shooterBottom = new WPI_TalonSRX(ShooterConstants.SHOOTER_SLAVE_CAN_ID);

  public ShooterSubsystem() {

    shooterBottom.setInverted(true);

  }

 public void setTopShooterMotorVoltage(double voltage) {

  shooterTop.setVoltage(voltage);
   
 }

 public void setBottomShooterMotorVoltage(double voltage) {

  shooterBottom.setVoltage(voltage);

 }

  

}