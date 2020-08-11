/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VoltageConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeployCommand extends CommandBase {
  
  IntakeSubsystem intakeSubsystem;
  
  public IntakeDeployCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //go slower when going down
    intakeSubsystem.setIntakeGateVoltage(VoltageConstants.INTAKE_GATE_DOWN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeGateVoltage(VoltageConstants.STOP);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}