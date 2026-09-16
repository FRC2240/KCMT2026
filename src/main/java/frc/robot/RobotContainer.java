// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederSimIO;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {


  private final Indexer indexer;
  private final Shooter shooter;

  public RobotContainer() {

    if (RobotBase.isReal()) {
      indexer = new Indexer(new IndexerIOTalonFX());
      shooter = new Shooter(new ShooterIOTalonFX());
    } else {
      indexer = new Indexer(new IndexerIOSim());
      shooter = new Shooter(new ShooterIOSim());
    }

    feeder.setDefaultCommand(feeder.enableCommand());
    indexer.setDefaultCommand(indexer.enableCommand());
    shooter.setDefaultCommand(shooter.enableCommand());

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
