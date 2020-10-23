/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class moveauto extends CommandBase {
  /**
   * Creates a new DriveForwardAuto.
   */
  DriveTrain robotDrive;
  long startTime;
  

  public moveauto(DriveTrain robotDrive) {
    this.robotDrive = robotDrive;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.nanoTime();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotDrive.setLeftMotors(0.5);
    robotDrive.setRightMotors(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.setLeftMotors(0.0);
    robotDrive.setRightMotors(0.0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (double)(System.nanoTime() - startTime) / 1_000_000_000.0 > 5.0;
  }
}