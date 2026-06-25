package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDrive extends SubsystemBase{
    private ChassisSpeeds chassisSpeeds; 
    private CommandXboxController controller; 

    private final StructArrayPublisher<SwerveModuleState> publisher; 

    double chasisHeight = 0.60; 
    double chasisWidth = 0.60; 

    Translation2d frontleft = new Translation2d(chasisHeight / 2, chasisWidth / 2);
    Translation2d frontRight = new Translation2d(chasisHeight / 2, -chasisWidth / 2);
    Translation2d backLeft = new Translation2d(-chasisHeight / 2, chasisWidth / 2);
    Translation2d backRight = new Translation2d(-chasisHeight / 2, -chasisWidth / 2);

    KrakenSwerveModule frontLeftModule = new KrakenSwerveModule(frontleft, 1, 2, 3);
    KrakenSwerveModule frontRightModule = new KrakenSwerveModule(frontRight, 4, 5, 6);
    KrakenSwerveModule backLeftModule = new KrakenSwerveModule(backLeft, 7, 8, 9);
    KrakenSwerveModule backRightModule = new KrakenSwerveModule(backRight, 10, 11, 12);

    KrakenSwerveModule[] swerveArray = new KrakenSwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        swerveArray[0].getPosition(), swerveArray[1].getPosition(), swerveArray[2].getPosition(), swerveArray[3].getPosition());

    public void setStates(ChassisSpeeds desired){
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desired);
        swerveArray[0].updateDesiredState(newStates[0]);
        swerveArray[1].updateDesiredState(newStates[1]);
        swerveArray[2].updateDesiredState(newStates[2]);
        swerveArray[3].updateDesiredState(newStates[3]);
    }

    public void updateModules(){
        swerveArray[0].Update();
        swerveArray[1].Update();
        swerveArray[2].Update();
        swerveArray[3].Update();
    }
    
    public SwerveDrive(CommandXboxController controller){
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/swerveStates", SwerveModuleState.struct).publish();
        this.controller = controller; 
    }

    @Override
    public void periodic() {
       chassisSpeeds = new ChassisSpeeds(
        controller.getLeftY(),

        controller.getLeftX(), 

        controller.getRightX()
       );

       setStates(chassisSpeeds);

       updateModules();

        

       publisher.set(new SwerveModuleState[]{
        frontLeftModule.getDesiredState(), 
        frontRightModule.getDesiredState(), 
        backLeftModule.getDesiredState(), 
        backRightModule.getDesiredState()
       });
    }
}
