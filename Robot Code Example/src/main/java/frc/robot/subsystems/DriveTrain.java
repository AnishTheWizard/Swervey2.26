package frc.robot.subsystems;


import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.motionoftheocean.MotionOfTheOcean;
import frc.libs.swervey.Swerve;
import frc.libs.swervey.SwerveBuilder;
import frc.libs.wrappers.GenericEncoder;
import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.Gyro;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


import frc.libs.sharkodo.sharkodo.SharkOdo;
import frc.libs.sharkodo.profiler.SwerveProfiler;

public class DriveTrain extends SubsystemBase {

    private final static DriveTrain INSTANCE = new DriveTrain();

    public static DriveTrain getInstance() {
        return INSTANCE;
    }

    private Swerve swerve;

    private SharkOdo sharkOdo;

    private DriveTrain() {

        GenericMotor[] drives = new GenericMotor[Constants.NUMBER_OF_MODULES];
        GenericMotor[] steers = new GenericMotor[Constants.NUMBER_OF_MODULES];
        GenericEncoder[] encoders = new GenericEncoder[Constants.NUMBER_OF_MODULES];

        for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
            TalonFX drive = new TalonFX(RobotMap.DRIVE_MOTORS[i]);
            TalonFX steer = new TalonFX(RobotMap.STEER_MOTORS[i]);

            CANCoder encoder = new CANCoder(RobotMap.ENCODERS[i]);

            drive.setNeutralMode(NeutralMode.Brake);

            drives[i] = new GenericMotor(drive);
            steers[i] = new GenericMotor(steer);

            encoders[i] = new GenericEncoder(encoder, Constants.OVERFLOW_THRESHOLD, Constants.MODULE_OFFSETS[i]);
        }

        Gyro gyro = new Gyro(0);

        swerve = new SwerveBuilder(drives, steers, encoders, gyro)
                .PIDGains(Constants.MODULE_GAINS, Constants.SCHEDULED_GAINS, Constants.STEER_AND_ROTATE_THRESHOLDS)
                .modulePositions(Constants.MODULE_POSITIONS)
                .speedBounds(Constants.SPEED_BOUNDS)
                .accelerationParameters(Constants.ACCELERATION_PARAMETERS)
                .autonomousParameters(Constants.TICKS_PER_INCHES, Constants.ALLOWED_ERRORS, Constants.VELOCITY_FEED_FORWARD)
                .buildSwerve();

        // SwerveProfiler profiler = new SwerveProfiler(swerve::getAllModuleDrivePose, swerve::getHeading, Constants.TICKS_PER_INCHES, 20);
        
        // this.sharkOdo = new SharkOdo(profiler, 20);

        // MotionOfTheOcean.addPositionFunctions(sharkOdo::getPoseAsArray, this::toPose);

        new TalonFX(0).set(ControlMode.)

    }

    public void control(double x, double y, double rotate) {
        swerve.control(x, y, -rotate);
    }

    public void reset() {
        swerve.reset();
        swerve.zeroGyro();
        // sharkOdo.resetPose();
    }

    public void startOdometry() {
        sharkOdo.startOdometryThread();
    }

    public void toPose(double[] target) {
        swerve.toPose(target);
    }

    public void toPoseWithMotion(double[] target) {
        swerve.toPoseWithMotion(target);
    }

    @Override
    public void periodic() {
        for(int i=0; i < Constants.NUMBER_OF_MODULES; i++) {
            SmartDashboard.putNumber("module offset " + i, swerve.getModuleRotationalPose(i));
        }

        double[] pose = swerve.getPose();

        SmartDashboard.putNumber("x", pose[0]);
        SmartDashboard.putNumber("y", pose[1]);
        SmartDashboard.putNumber("theta", pose[2]);
        SmartDashboard.putNumber("velocity", pose[3]);

        int i = 0;
        // for(double[] arr : swerve.getAllModuleDrivePoseDelta()) {
        //     SmartDashboard.putString("deez: " + i, Arrays.toString(arr));
        //     i++;
        // }
    }
}

