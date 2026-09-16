package frc.robot.subsystems.drivetrain;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends SubsystemBase {

  private final Field2d field = new Field2d();

  private final ModuleIO moduleIOs[] = new ModuleIO[4];
  private ModuleIOInputsAutoLogged moduleInputs[] = new ModuleIOInputsAutoLogged[] {
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged()
  };

  private final GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

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

  public Drivetrain(ModuleIO frontLeftModuleIO,
      ModuleIO frontRightModuleIO,
      ModuleIO backLeftModuleIO,
      ModuleIO backRightModuleIO,
      GyroIO gyroIO) {
    moduleIOs[0] = frontLeftModuleIO;
    moduleIOs[1] = frontRightModuleIO;
    moduleIOs[2] = backLeftModuleIO;
    moduleIOs[3] = backRightModuleIO;

    this.gyroIO = gyroIO;
    
    SmartDashboard.putData(field);


    OdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    OdometryThread.getInstance().lock();

    // Update inputs
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
      Logger.processInputs("Module " + i, moduleInputs[i]);
    }

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Gyro", gyroInputs);

    OdometryThread.getInstance().unlock();

    for (int i = 0; i < moduleInputs[0].timestamps.length; i++) {

      poseEstimator.updateWithTime(gyroInputs.timestamps[i], gyroInputs.samples[i], new SwerveModulePosition[] {
        moduleInputs[0].positionSamples[i],
        moduleInputs[1].positionSamples[i],
        moduleInputs[2].positionSamples[i],
        moduleInputs[3].positionSamples[i],
      });

    }
    
    field.setRobotPose(poseEstimator.getEstimatedPosition());
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

      // Transform to swerve module states
      ChassisSpeeds targetRobotRelativeSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(targetFieldRelativeSpeed, poseEstimator.getEstimatedPosition().getRotation());
      ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(targetRobotRelativeSpeed, 0.02);
      SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(discretizedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 3);

      // Optimize and apply
      for (int i = 0; i < moduleIOs.length; i++) {
        targetStates[i].optimize(moduleIOs[i].getHeading());
        moduleIOs[i].setState(targetStates[i]);
      }
    });
  }
}
