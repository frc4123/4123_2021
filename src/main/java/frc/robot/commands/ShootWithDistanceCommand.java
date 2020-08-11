/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithDistanceCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  NetworkTable table;
  NetworkTableEntry target3D;
  double topVoltage;
  double botVoltage;

  public ShootWithDistanceCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
    target3D = table.getEntry("tablePose");
  }

  private void getDistance() {
    double[] defaultArray = { 0.0, 0.0, 0.0 };
    double targetDistance = target3D.getDoubleArray(defaultArray)[0];

    boolean threeMeters = (targetDistance <= 3);
    boolean threeToFourMeters = (targetDistance > 3 && targetDistance <= 4);
    boolean fourToSixMeters = (targetDistance > 4 && targetDistance <= 6);

    if (threeMeters) {
      topVoltage = 5;
      botVoltage = 12;
    } else if (threeToFourMeters) {
      topVoltage = 11;
      botVoltage = 12;
    } else if (fourToSixMeters) {
      topVoltage = 12;
      botVoltage = 12;
    } else if (fourToSixMeters) {
      topVoltage = 12;
      botVoltage = 11.7;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getDistance();
    shooterSubsystem.setTopShooterMotorVoltage(topVoltage);
    shooterSubsystem.setBottomShooterMotorVoltage(botVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setTopShooterMotorVoltage(0);
    shooterSubsystem.setBottomShooterMotorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
