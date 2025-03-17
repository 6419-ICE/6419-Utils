package org.ice.util.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.ice.util.sendable.AnnotatedSubsystemBase;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public abstract class REVDriveTrain extends AnnotatedSubsystemBase {
    private REVSwerveModule frontLeft,frontRight,backLeft,backRight;
    private SwerveDriveOdometry odometry;
    private SwerveDriveKinematics kinematics;
    private DriveTrainConfig config;
    public REVDriveTrain(REVSwerveModule frontLeft, REVSwerveModule frontRight, REVSwerveModule backLeft, REVSwerveModule backRight, DriveTrainConfig config) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.config = config;
        kinematics = new SwerveDriveKinematics(
                new Translation2d(config.wheelBase / 2, config.trackWidth / 2),
                new Translation2d(config.wheelBase / 2, -config.trackWidth / 2),
                new Translation2d(-config.wheelBase / 2, config.trackWidth / 2),
                new Translation2d(-config.wheelBase / 2, -config.trackWidth / 2)
        );
        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(getHeading(false)),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }
        );
        try {
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getRelativeSpeeds,
                    (speeds,feedForwards)->driveRelative(speeds),
                    new PPHolonomicDriveController(
                            config.translationPID,
                            config.rotationPID
                    ),
                    RobotConfig.fromGUISettings(),
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this
            );
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
    @Override
    public void periodic() {
        odometry.update(
                Rotation2d.fromDegrees(getHeading(true)),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }
        );
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(getHeading(true)),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }, pose
        );
    }
    public ChassisSpeeds getRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );
    }
    public void driveRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds,0.02);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretized);
        frontLeft.setDesiredState(states[0]); //1
        frontRight.setDesiredState(states[1]); //0
        backLeft.setDesiredState(states[2]); //3
        backRight.setDesiredState(states[3]); //2
    }
    /**
     * This is private because it kinda uses the ChassisSpeeds class wrong, and I dont want any accidental mix ups (this treats the chassis speeds' speedMetersPerSecond as a percent power for duty cycle)
     * @param speeds
     */
    private void driveRelativeRaw(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeft.driveRaw(states[0].angle,states[0].speedMetersPerSecond); //1
        frontRight.driveRaw(states[1].angle,states[1].speedMetersPerSecond); //0
        backLeft.driveRaw(states[2].angle,states[2].speedMetersPerSecond); //3
        backRight.driveRaw(states[3].angle,states[3].speedMetersPerSecond); //2
    }
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed *= config.maxSpeed;
        ySpeed *= config.maxSpeed;
        rot *= config.maxAngularSpeed;
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
        //new ChassisSpeeds(xSpeed,ySpeed,rot).toFieldRelativeSpeeds(Rotation2d.fromDegrees(getHeading()));
        if (fieldRelative) speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds,Rotation2d.fromDegrees(getHeading(false)));
        driveRelative(speeds);
    }
    public void driveRaw(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
        if (fieldRelative) speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds,Rotation2d.fromDegrees(getHeading(false)));
        driveRelativeRaw(speeds);
    }
    @Getter(key="Raw Heading")
    public abstract double getRawHeading();

    public double getHeading(boolean flipped) {
        return getRawHeading() * ((flipped ^ config.gyroReversed) ? -1 : 1);
    }

    public record DriveTrainConfig(
            double trackWidth,
            double wheelBase,
            boolean gyroReversed,
            double maxSpeed,
            double maxAngularSpeed,
            PIDConstants translationPID,
            PIDConstants rotationPID
    ) {}
}
