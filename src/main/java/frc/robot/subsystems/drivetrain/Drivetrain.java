package frc.robot.subsystems.drivetrain;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends SubsystemBase {

  public final DriveSuppliers suppliers;

  private final ModuleIO moduleIOs[] = new ModuleIO[4];
  private ModuleIOInputsAutoLogged moduleInputs[] = new ModuleIOInputsAutoLogged[] {
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged()
  };

  private final GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Rotation2d rawGyroRotation = Rotation2d.kZero;

  private static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    );
  
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics,
    Rotation2d.kZero, new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    },
    Pose2d.kZero
  );

  private final SwerveModulePosition[] lastPositions = new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  private ChassisSpeeds currentRobotRelativeChassisSpeeds = new ChassisSpeeds();

  public Drivetrain(ModuleIO frontLeftModuleIO,
      ModuleIO frontRightModuleIO,
      ModuleIO backLeftModuleIO,
      ModuleIO backRightModuleIO,
      GyroIO gyroIO,
      CommandXboxController controller
    ) {
    this.moduleIOs[0] = frontLeftModuleIO;
    this.moduleIOs[1] = frontRightModuleIO;
    this.moduleIOs[2] = backLeftModuleIO;
    this.moduleIOs[3] = backRightModuleIO;
    this.gyroIO = gyroIO;
    this.suppliers = new DriveSuppliers(this, controller);

    setupPathPlanner();

    OdometryThread.getInstance().start();
  }

  private void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
              this::getPose,
              this::resetPose,
              this::getChassisSpeeds,
              (speeds, feedforwards) -> driveRobotRelative(speeds),
              new PPHolonomicDriveController(
                      new PIDConstants(2.0, 0.0, 0.10), // Translation PID constants
                      new PIDConstants(2.0, 0.0, 0.10) // Rotation PID constants
              ),
              config,
              () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
              this);
    } catch (Exception e) {
      // Handle exception as needed
      System.out.println("error");
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // Update inputs
    OdometryThread.getInstance().lock();

    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
      Logger.processInputs("Module " + i, moduleInputs[i]);
    }

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Gyro", gyroInputs);

    OdometryThread.getInstance().unlock();

    // Update Position Estimate
    for (int i = 0; i < moduleInputs[0].timestamps.length; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = moduleInputs[moduleIndex].positionSamples[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters - lastPositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastPositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.samples[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      
      poseEstimator.updateWithTime(moduleInputs[0].timestamps[i], rawGyroRotation, new SwerveModulePosition[] {
        moduleInputs[0].positionSamples[i],
        moduleInputs[1].positionSamples[i],
        moduleInputs[2].positionSamples[i],
        moduleInputs[3].positionSamples[i],
      });
    }

    SwerveModuleState[] moduleStates = new SwerveModuleState[] {
      moduleInputs[0].state,
      moduleInputs[1].state,
      moduleInputs[2].state,
      moduleInputs[3].state,
    };
    // Calculate current chassis speeds
    currentRobotRelativeChassisSpeeds = kinematics.toChassisSpeeds(moduleStates); 

    // Log outputs
    Logger.recordOutput("Swerve Module States", moduleStates);

    Logger.recordOutput("Robot Position", poseEstimator.getEstimatedPosition());
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(discretizedSpeeds);

    // Optimize and find the cosine scale factor
    double currentScale = 1;
    for (int i = 0; i < moduleIOs.length; i++) {
      targetStates[i].optimize(moduleInputs[i].position.angle);
      currentScale = Math.min(currentScale, Math.abs(moduleIOs[i].getHeading().minus(targetStates[i].angle).getCos()));
    }

    // Scale all motors down
    for (int i = 0; i < moduleIOs.length; i++) { 
      targetStates[i].speedMetersPerSecond *= currentScale;
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.MAX_SPEED);

    // Apply
    for (int i = 0; i < moduleIOs.length; i++) {
      moduleIOs[i].setState(targetStates[i]);
    }
  }

  private void driveFieldRelative(ChassisSpeeds speeds) {
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimator.getEstimatedPosition().getRotation());
    driveRobotRelative(robotRelative);
  }

  public Command driveCommand(Supplier<ChassisSpeeds> speedSupplier) {
    return driveCommand(List.of(speedSupplier));
  }

  public Command driveCommand(List<Supplier<ChassisSpeeds>> speedSuppliers) {
    assert(speedSuppliers.size() > 0);
    return run(() -> {
      // Sum up the inputs
      ChassisSpeeds targetFieldRelativeSpeed = new ChassisSpeeds(0,0,0);
      for (Supplier<ChassisSpeeds> speedSupplier : speedSuppliers) {
        targetFieldRelativeSpeed = targetFieldRelativeSpeed.plus(speedSupplier.get());
      }

      driveFieldRelative(targetFieldRelativeSpeed);
    });
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }

  public void resetPose(Pose2d pose) {
    SwerveModulePosition[] currentPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      currentPositions[i] = moduleInputs[i].position != null ? 
                            moduleInputs[i].position : new SwerveModulePosition();
    }

    this.rawGyroRotation = pose.getRotation();

    poseEstimator.resetPosition(
        pose.getRotation(),
        currentPositions,
        pose
    );
  }

  public Command rezeroGyroCommand() {
    return runOnce(() -> {
      resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.kZero));
    });
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return currentRobotRelativeChassisSpeeds;
  }

  // Pre-made drive commands
  public Command driveWithControllerCommand() {
    return driveCommand(List.of(suppliers.controllerDrive(), suppliers.controllerTurn()));
  }
}
