package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Vision;
import frc.robot.Constants.DriveConstants;

public class MecanumDrive extends SubsystemBase{
    
    //Motors
    private final WPI_TalonFX m_FrontLeft;
    private final WPI_TalonFX m_FrontRight;
    private final WPI_TalonFX m_BackLeft;
    private final WPI_TalonFX m_BackRight;

    //Gyro
    private final WPI_Pigeon2 m_Gyro;

    //Fun Things
    private final MecanumDriveKinematics m_Kinematics;
    private final MecanumDrivePoseEstimator m_PoseEstimator;

    //PID Controllers
    private final PIDController m_FrontLeft_PID;
    private final PIDController m_FrontRight_PID;
    private final PIDController m_BackLeft_PID;
    private final PIDController m_BackRight_PID;

    //Feed Forward Controller
    private final SimpleMotorFeedforward m_Feedforward;  
    
    //Vision
    private final Vision m_Vision;

    //Field Sim
    private final Field2d m_Field;


    public MecanumDrive() {

        //Assign Motors
        m_FrontLeft = new WPI_TalonFX(DriveConstants.FRONT_LEFT_ID);
        m_FrontRight = new WPI_TalonFX(DriveConstants.FRONT_RIGHT_ID);
        m_BackLeft = new WPI_TalonFX(DriveConstants.BACK_LEFT_ID);
        m_BackRight = new WPI_TalonFX(DriveConstants.BACK_RIGHT_ID);
     
        //Assign Gyro
        m_Gyro = new WPI_Pigeon2(DriveConstants.GYRO_ID);

        //Set Kinematics
        m_Kinematics = new MecanumDriveKinematics(
            DriveConstants.FRONT_LEFT_POSITION,
            DriveConstants.FRONT_RIGHT_POSITION,
            DriveConstants.BACK_LEFT_POSITION,
            DriveConstants.BACK_RIGHT_POSITION
        );

        //Set Pose Estimator
        m_PoseEstimator = new MecanumDrivePoseEstimator(
            m_Kinematics, 
            m_Gyro.getRotation2d(),
            getWheelPositions(), 
            new Pose2d()
        );

        //TODO These should be fine ish but might need tuned.
        //Setup PID Gains
        m_FrontLeft_PID = new PIDController(1, 0, 0);
        m_FrontRight_PID = new PIDController(1, 0, 0);
        m_BackLeft_PID = new PIDController(1, 0, 0);
        m_BackRight_PID = new PIDController(1, 0, 0);
        
        //TODO Characterize the drivetrain to get these values.
        //Setup Characterization Gains
        m_Feedforward = new SimpleMotorFeedforward(1, 3,  0);

        //Instantiate Vision
        m_Vision = new Vision();

        //Instantitate Field
        m_Field = new Field2d();

        //Configure Motors
        m_FrontLeft.setInverted(InvertType.None);
        m_FrontRight.setInverted(InvertType.InvertMotorOutput);
        m_BackLeft.setInverted(InvertType.None);
        m_BackRight.setInverted(InvertType.InvertMotorOutput);

    }

    //Gets all motor position in meters
    public MecanumDriveWheelPositions getWheelPositions(){
        return new MecanumDriveWheelPositions(
            m_FrontLeft.getSelectedSensorPosition() * DriveConstants.FALCON_TO_METERS,
            m_FrontRight.getSelectedSensorPosition() * DriveConstants.FALCON_TO_METERS,
            m_BackLeft.getSelectedSensorPosition() * DriveConstants.FALCON_TO_METERS,
            m_BackRight.getSelectedSensorPosition() * DriveConstants.FALCON_TO_METERS
        );
    }

    //Gets all motor velocities in meters/s
    public MecanumDriveWheelSpeeds getWheelSpeeds(){
        return new MecanumDriveWheelSpeeds(
            m_FrontLeft.getSelectedSensorVelocity() * 10 * DriveConstants.FALCON_TO_METERS,
            m_FrontRight.getSelectedSensorVelocity() * 10 * DriveConstants.FALCON_TO_METERS,
            m_BackLeft.getSelectedSensorVelocity() * 10 * DriveConstants.FALCON_TO_METERS,
            m_BackRight.getSelectedSensorVelocity() * 10 * DriveConstants.FALCON_TO_METERS
        );
    }

    public Pose2d getPose(){
        return m_PoseEstimator.getEstimatedPosition();
    }

    //Set wheel speeds in meters/s
    public void setSpeeds(MecanumDriveWheelSpeeds speeds){
        final double frontLeftFeedforward = m_Feedforward.calculate(speeds.frontLeftMetersPerSecond);
        final double frontRightFeedforward = m_Feedforward.calculate(speeds.frontRightMetersPerSecond);
        final double backLeftFeedforward = m_Feedforward.calculate(speeds.rearLeftMetersPerSecond);
        final double backRightFeedforward = m_Feedforward.calculate(speeds.rearRightMetersPerSecond);
    
        final double frontLeftOutput = m_FrontLeft_PID.calculate(
            getWheelSpeeds().frontLeftMetersPerSecond, 
            speeds.frontLeftMetersPerSecond
        );

        final double frontRightOutput = m_FrontRight_PID.calculate(
            getWheelSpeeds().frontRightMetersPerSecond, 
            speeds.frontRightMetersPerSecond
        );

        final double backLeftOutput = m_BackLeft_PID.calculate(
            getWheelSpeeds().rearLeftMetersPerSecond, 
            speeds.rearLeftMetersPerSecond
        );

        final double backRightOutput = m_BackRight_PID.calculate(
            getWheelSpeeds().rearRightMetersPerSecond, 
            speeds.rearRightMetersPerSecond
        );
    
        m_FrontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
        m_FrontRight.setVoltage(frontRightOutput + frontRightFeedforward);
        m_BackLeft.setVoltage(backLeftOutput + backLeftFeedforward);
        m_BackRight.setVoltage(backRightOutput + backRightFeedforward);
    }

    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative){

        var mecanumDriveWheelSpeeds =
        m_Kinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, m_Gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rSpeed));

        mecanumDriveWheelSpeeds.desaturate(DriveConstants.MAX_SPEED);
        setSpeeds(mecanumDriveWheelSpeeds);

    }

    public void resetPose(Pose2d pose){
        m_PoseEstimator.resetPosition(m_Gyro.getRotation2d(), getWheelPositions(), pose);
    }

    public void updatePoseEstimation(){
        m_PoseEstimator.update(m_Gyro.getRotation2d(), getWheelPositions());

        Pair<Pose2d, Double> result = m_Vision.getEstimatedGlobalPose(m_PoseEstimator.getEstimatedPosition());
        var camPose = result.getFirst();
        var camPoseObsTime = result.getSecond();

        if (camPose != null) {
            m_PoseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
            m_Field.getObject("Cam Est Pos").setPose(camPose);
        } else {
            // move it way off the screen to make it disappear
            m_Field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }

        m_Field.setRobotPose(m_PoseEstimator.getEstimatedPosition());

    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset pose estimation for the first path you run during auto
              if(isFirstPath){
                  this.resetPose(traj.getInitialHolonomicPose());
              }
            }),
            new PPMecanumControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                m_Kinematics,
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                6.0, // Max wheel velocity meters per second
                this::setSpeeds, // MecanumDriveWheelSpeeds consumer
                this // Requires this drive subsystem
            )
        );
    }

    @Override
    public void periodic() {
        updatePoseEstimation();
    }

}
