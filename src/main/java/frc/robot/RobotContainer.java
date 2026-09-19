// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);

  private final Indexer indexer;
  private final Shooter shooter;
  private final Drivetrain drivetrain;

  public RobotContainer() {

 if (RobotBase.isReal()) {
      indexer = new Indexer(new IndexerIOTalonFX());
      shooter = new Shooter(new ShooterIOTalonFX());
      drivetrain = new Drivetrain(
        new ModuleIOTalonFX(TunerConstants.FrontLeft),
        new ModuleIOTalonFX(TunerConstants.FrontRight),
        new ModuleIOTalonFX(TunerConstants.BackLeft),
        new ModuleIOTalonFX(TunerConstants.BackRight),
        new GyroIOPigeon2()
      );
    } else {
      indexer = new Indexer(new IndexerIOSim());
      shooter = new Shooter(new ShooterIOSim());
      drivetrain = new Drivetrain(
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new GyroIO() {}
      );
    }




    indexer.setDefaultCommand(indexer.enableCommand());
    shooter.setDefaultCommand(shooter.enableCommand());
    drivetrain.setDefaultCommand(DriveCommands.controlDrivetrainWithController(drivetrain, controller));

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
