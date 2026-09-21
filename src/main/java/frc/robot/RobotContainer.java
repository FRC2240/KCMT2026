// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.DriveCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.CameraIOPhotonVisionSim;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final Indexer indexer;
  private final Feeder feeder;

  private final CommandXboxController controller = new CommandXboxController(0);

  private final Shooter shooter;
  private final Drivetrain drivetrain;
  private final Vision vision;

  public RobotContainer() {
    switch (Constants.mode) {
      case REAL:
        drivetrain = new Drivetrain(
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight),
            new GyroIOPigeon2());

        vision = new Vision(
            drivetrain::addVisionMeasurement,
            new CameraIOLimelight("limelight-left", drivetrain::getRotation),
            new CameraIOLimelight("limelight-right", drivetrain::getRotation));
        
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        feeder = new Feeder(new FeederIOReal());

        break;
      case SIM:
        drivetrain = new Drivetrain(
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new GyroIO() {});

        vision = new Vision(
          drivetrain::addVisionMeasurement, 
          new CameraIOPhotonVisionSim(),
          new CameraIOPhotonVisionSim());

          indexer = new Indexer(new IndexerIOSim());
          shooter = new Shooter(new ShooterIOSim());
          feeder = new Feeder(new FeederIOSim());
        break;

      default:
      case REPLAY:
        drivetrain = new Drivetrain(
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new GyroIO() {});

        vision = new Vision(
          drivetrain::addVisionMeasurement, 
          new CameraIO() {},
          new CameraIO() {});

        indexer = new Indexer(new IndexerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        feeder = new Feeder(new FeederIO() {});

        break;
    }

    indexer.setDefaultCommand(indexer.enableCommand());
    drivetrain.setDefaultCommand(DriveCommands.controlDrivetrainWithController(drivetrain, controller));

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
