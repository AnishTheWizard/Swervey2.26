package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.swervey.Swerve;
import frc.libs.swervey.SwerveBuilder;

public class DriveTrain extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    private final static DriveTrain INSTANCE = new DriveTrain();

    public static DriveTrain getInstance() {
        return INSTANCE;
    }

    private Swerve swerve;

    private DriveTrain() {
        GenericMotor =
        this.swerve = new SwerveBuilder()
    }
}

