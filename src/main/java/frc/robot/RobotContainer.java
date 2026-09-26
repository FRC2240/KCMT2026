// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intakePivot.IntakePivotIOReal;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSim;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.CameraIOPhotonVisionSim;
import frc.robot.subsystems.vision.Vision;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);
  private final Drivetrain drivetrain;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Feeder feeder;
  private final IntakePivot intakePivot;
  private final IntakeRoller intakeRoller;
  private final ShootingController shootingController;

  public RobotContainer() {
    switch (Constants.mode) {
      case REAL:
        drivetrain = new Drivetrain(
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight),
            new GyroIOPigeon2(),
            controller);

        vision = new Vision(
            drivetrain::addVisionMeasurement,
            new CameraIOLimelight("limelight-left", drivetrain::getRotation),
            new CameraIOLimelight("limelight-right", drivetrain::getRotation));
        
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        feeder = new Feeder(new FeederIOReal());
        intakePivot = new IntakePivot(new IntakePivotIOReal());
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX(), drivetrain);
        shootingController = new ShootingController(drivetrain, shooter, feeder, indexer, intakePivot, intakeRoller);

        break;
      case SIM:
        drivetrain = new Drivetrain(
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new GyroIO() {},
            controller);

        vision = new Vision(
          drivetrain::addVisionMeasurement, 
          new CameraIOPhotonVisionSim(),
          new CameraIOPhotonVisionSim());

          indexer = new Indexer(new IndexerIOSim());
          shooter = new Shooter(new ShooterIOSim());
          feeder = new Feeder(new FeederIOSim());
          intakePivot = new IntakePivot(new IntakePivotIOSim());
          intakeRoller = new IntakeRoller(new IntakeRollerIOSim(), drivetrain);
          shootingController = new ShootingController(drivetrain, shooter, feeder, indexer, intakePivot, intakeRoller);

        break;
      default:
      case REPLAY:
        drivetrain = new Drivetrain(
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new GyroIO() {},
            controller);

        vision = new Vision(
          drivetrain::addVisionMeasurement, 
          new CameraIO() {},
          new CameraIO() {});

        indexer = new Indexer(new IndexerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        feeder = new Feeder(new FeederIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {}, drivetrain);
        shootingController = new ShootingController(drivetrain, shooter, feeder, indexer, intakePivot, intakeRoller);

        break;
    }

    indexer.setDefaultCommand(indexer.enableCommand());
    feeder.setDefaultCommand(feeder.enableCommand());
    shooter.setDefaultCommand(shooter.enableCommand());
    drivetrain.setDefaultCommand(drivetrain.driveWithControllerCommand());

    configureDefaults();
    configureBindings();
  }

  private void configureBindings() {
    // Disable Intake
    controller.leftTrigger().toggleOnTrue(intakeRoller.disableIntakeCommand());

    // Toggle Slow Mode
    controller.back().onTrue(drivetrain.suppliers.toggleSlowModeCommand());
/* 
    // Zero the Gyro
    controller.start().onTrue(drivetrain.rezeroGyro());
*/
    // Shoot
    controller.rightTrigger().whileTrue(shootingController.shoot());

    // Reverse Intake
    controller.povDown().whileTrue(intakeRoller.reverseIntakeCommand());

    // Pivot rezeroing
    controller.rightBumper().onTrue(intakePivot.rezeroCommand());

    // Contract Intake
    controller.x().toggleOnTrue(intakePivot.contractCommand().alongWith(intakeRoller.disableIntakeCommand()));

    // Align with Trench
    controller.y().whileTrue(drivetrain.driveCommand(List.of(drivetrain.suppliers.controllerDrive(), 
                drivetrain.suppliers.trenchAlign()))); }

  private void configureDefaults() {
   //Drive with Stick
    drivetrain.setDefaultCommand(drivetrain.driveWithControllerCommand());

    // indexer is disabled by default
    indexer.setDefaultCommand(indexer.disableCommand());

    // Intake is enabled by default
    intakeRoller.setDefaultCommand(intakeRoller.enableIntakeCommand());

    // Feeder is disabled by default
    feeder.setDefaultCommand(feeder.disableCommand());

    // Shooter coasts when not used (power saving)
    shooter.setDefaultCommand(shooter.coastCommand());

    // Pivot is extended by default
    intakePivot.setDefaultCommand(intakePivot.extendCommand());
    }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
