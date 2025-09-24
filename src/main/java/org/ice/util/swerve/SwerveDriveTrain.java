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

public abstract class SwerveDriveTrain extends AnnotatedSubsystemBase {
    private SwerveModule frontLeft,frontRight,backLeft,backRight;
    private SwerveModule[] moduleArr;
    private SwerveDriveOdometry odometry;
    private SwerveDriveKinematics kinematics;
    private DriveTrainConfig config;

    /**
     * Constructs a new REVDriveTrain with the four given {@link REVSwerveModule swerve modules}, and the given config
     * @param frontLeft the front left swerve module
     * @param frontRight the front right swerve module
     * @param backLeft the back left swerve module
     * @param backRight the back right swerve module
     * @param config config values for the new module
     */
    public SwerveDriveTrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, DriveTrainConfig config) {
        moduleArr = new SwerveModule[] {
                this.frontLeft = frontLeft,
                this.frontRight = frontRight,
                this.backLeft = backLeft,
                this.backRight = backRight
        };

        this.config = config;
        kinematics = new SwerveDriveKinematics(
                config.wheelLocations.frontLeft,
                config.wheelLocations.frontRight,
                config.wheelLocations.backLeft,
                config.wheelLocations.backRight
        );
        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(0),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }
        );
    }

    /**
     * Constructs a new REVDriveTrain object with four {@link REVSwerveModule modules} created using the given array of motor IDs, and the given config.
     * @param moduleConfig The module config used for all the modules
     * @param driveTrainConfig the config for the driveTrain
     */
    public SwerveDriveTrain(int frontLeftDrive, int frontLeftTurn, int frontRightDrive, int frontRightTurn, int backLeftDrive, int backLeftTurn, int backRightDrive, int backRightTurn, REVSwerveModule.ModuleConfig moduleConfig, DriveTrainConfig driveTrainConfig) {
        this(
                new REVSwerveModule(frontLeftDrive,frontLeftTurn,moduleConfig,0),
                new REVSwerveModule(frontRightDrive,frontRightTurn,moduleConfig,0),
                new REVSwerveModule(backLeftDrive,backLeftTurn,moduleConfig,0),
                new REVSwerveModule(backRightDrive,backRightTurn,moduleConfig,0),
                driveTrainConfig
        );
//        if (motorIDs.length != 8) throw new IllegalArgumentException("Expected 8 motor IDs, got " + motorIDs.length);
//        frontLeft = new REVSwerveModule(motorIDs[0],motorIDs[1],moduleConfig,0);
//        frontRight = new REVSwerveModule(motorIDs[2],motorIDs[3],moduleConfig,0);
//        backLeft = new REVSwerveModule(motorIDs[4],motorIDs[5],moduleConfig,0);
//        backRight = new REVSwerveModule(motorIDs[6],motorIDs[7],moduleConfig,0);
//        config = driveTrainConfig;
//        kinematics = new SwerveDriveKinematics(
//                config.wheelLocations.frontLeft,
//                config.wheelLocations.frontRight,
//                config.wheelLocations.backLeft,
//                config.wheelLocations.backRight
//        );
//        odometry = new SwerveDriveOdometry(
//                kinematics,
//                Rotation2d.fromDegrees(getHeading(false)),
//                new SwerveModulePosition[] {
//                        frontLeft.getPosition(),
//                        frontRight.getPosition(),
//                        backLeft.getPosition(),
//                        backRight.getPosition()
//                }
//        );
    }
    /**
     * Constructs a new DriveTrain object with four {@link CTRESwerveModule modules} created using the given array of motor IDs, and the given config.
     * @param moduleConfig The module config used for all the modules
     * @param driveTrainConfig the config for the driveTrain
     */
    public SwerveDriveTrain(int frontLeftDrive, int frontLeftTurn, int frontRightDrive, int frontRightTurn, int backLeftDrive, int backLeftTurn, int backRightDrive, int backRightTurn, CTRESwerveModule.ModuleConfig moduleConfig, DriveTrainConfig driveTrainConfig) {
        this(
                new CTRESwerveModule(frontLeftDrive, frontLeftTurn, moduleConfig, 0, driveTrainConfig.wheelLocations().frontLeft()),
                new CTRESwerveModule(frontRightDrive, frontRightTurn, moduleConfig, 0, driveTrainConfig.wheelLocations().frontRight()),
                new CTRESwerveModule(backLeftDrive, backLeftTurn, moduleConfig, 0, driveTrainConfig.wheelLocations().backLeft()),
                new CTRESwerveModule(backRightDrive, backRightTurn, moduleConfig, 0, driveTrainConfig.wheelLocations().backRight()),
                driveTrainConfig
        );
    }
    /**
     * Configures the PathPlanner AutoBuilder using the information from this class
     */
    public void configureAutoBuilder() {
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

    /**
     * Returns the current pose estimate of the robot based on encoders.
     * @return
     */
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
        //not actually quite sure what this does
        ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds,0.02);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretized);
        frontLeft.setDesiredState(states[0]); //1
        frontRight.setDesiredState(states[1]); //0
        backLeft.setDesiredState(states[2]); //3
        backRight.setDesiredState(states[3]); //2
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

    /**
     * Gets the raw, unmodified heading in degrees (does not include the gyro reversed config value)
     * @return the heading of the robot in degrees
     */
    @Getter(key="Raw Heading")
    public abstract double getRawHeading();

    /**
     * Returns the heading, with a boolean that inverts the value (left-positive -> right-positive and vice versa)
     * @param flipped if the value should be flipped
     * @return the heading of the robot, modified by if the gyro is flipped, and the value of the flipped parameter
     */
    public double getHeading(boolean flipped) {
        return getRawHeading() * ((flipped ^ config.gyroReversed) ? -1 : 1);
    }

    /**
     * Returns an array of all the underlying {@link SwerveModule} objects in the drive train
     * @return a 4 length array containing this DriveTrain's Swerve Modules in the order: frontLeft, frontRight, backLeft, backRight
     */
    public SwerveModule[] getModules() {
        return moduleArr;
    }

    /**
     * Config storing the constants needed to set up a swerve drive train
     * @param wheelLocations The locations of the four wheels relative to the center of the robot
     * @param gyroReversed Boolean that determines wether or not the gyro input should be reversed
     * @param maxSpeed The max speed of the robot in m/s
     * @param maxAngularSpeed The max angular speed of the robot in rad/s
     * @param translationPID PID used to control translational movement (left, right, forward, backward)
     * @param rotationPID PID used to control rotational movement (turn right, turn left)
     */
    public record DriveTrainConfig(
            WheelLocations wheelLocations,
            boolean gyroReversed,
            double maxSpeed,
            double maxAngularSpeed,
            PIDConstants translationPID,
            PIDConstants rotationPID
    ) {}

    /**
     * Record storing the locations of the four swerve module wheels, relative to the center of the robot
     * @param frontLeft the location of the center of the front left wheel relative to the center of the robot, in meters
     * @param frontRight the location of the center of the front right wheel relative to the center of the robot, in meters
     * @param backLeft the location of the center of the back left wheel relative to the center of the robot, in meters
     * @param backRight the location of the center of the back right wheel relative to the center of the robot, in meters
     */
    public record WheelLocations(
            Translation2d frontLeft,
            Translation2d frontRight,
            Translation2d backLeft,
            Translation2d backRight
    ) {
        /**
         * Creates a new WheelLocations object, with the four locations being generated based off of the given track width and wheel base
         * @param trackWidth The distance between the centers of the wheels on the left and right side of the robot
         * @param wheelBase The distance between the centers of the wheels on the front and back side of the robot
         */
        public WheelLocations(double trackWidth, double wheelBase) {
            this(
                    new Translation2d(wheelBase / 2, trackWidth / 2),
                    new Translation2d(wheelBase / 2, -trackWidth / 2),
                    new Translation2d(-wheelBase / 2, trackWidth / 2),
                    new Translation2d(-wheelBase / 2, -trackWidth / 2)
            );
        }
    }
}
