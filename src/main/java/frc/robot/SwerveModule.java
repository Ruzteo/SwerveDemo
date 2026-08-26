package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class SwerveModule {
    private SwerveModuleState desiredState;
    private SwerveModuleState currentState;  
    private Translation2d position; 
    private KrakenAngleModule angle; 
    private KrakenDriveModule drive; 

    public SwerveModule(Translation2d position, int driveMotorID, int angleMotorID, int encoderID){
        desiredState = new SwerveModuleState();
        this.position = position; 
        angle = new KrakenAngleModule(angleMotorID, encoderID); 
        drive = new KrakenDriveModule(driveMotorID); 
    }

    public void updateDesiredState(SwerveModuleState newState){
        desiredState = newState; 
    }

    public SwerveModuleState getDesiredState(){
        return desiredState; 
    }

    public SwerveModuleState getCurrentState(){
        return currentState; 
    }

    public Translation2d getPosition(){
        return position; 
    }

    
    public void Update(){//Updates the updates 
        drive.setVelocity(desiredState.speedMetersPerSecond);
        desiredState.optimize(null);
    }

    
    
}


