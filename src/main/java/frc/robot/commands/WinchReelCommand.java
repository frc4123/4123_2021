/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VoltageConstants;
import frc.robot.subsystems.WinchSubsystem;

public class WinchReelCommand extends CommandBase {
  
  WinchSubsystem winchSubsystem;

  public WinchReelCommand(WinchSubsystem winchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(winchSubsystem);
    this.winchSubsystem = winchSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    winchSubsystem.setWinchMotorVoltage(VoltageConstants.WINCH_UP_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    winchSubsystem.setWinchMotorVoltage(VoltageConstants.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}