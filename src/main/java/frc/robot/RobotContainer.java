// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final Indexer indexer;
  private final Intake intake;

  public RobotContainer() {

    if (RobotBase.isReal()) {
      indexer = new Indexer(new IndexerIOTalonFX());
      intake = new Intake(new IntakeIOTalonFX());
    } else {
      indexer = new Indexer(new IndexerIOSim());
      intake = new Intake(new IntakeIOSim());
    }

    indexer.setDefaultCommand(indexer.enableCommand());

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
