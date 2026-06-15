package frc.robot;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class KrakenSwerveModule {
    SwerveModuleState state; 
    Translation2d position; 

    SparkMax driveMotor; 
    SparkMax angleMotor; 
    AbsoluteEncoder angleEncoder; 

    public KrakenSwerveModule(Translation2d position, int driveMotorID, int angleMotorID, int encoderID){
        state = new SwerveModuleState();
        this.position = position; 
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
    }

    public void updateState(SwerveModuleState newState){
        state = newState; 
    }

    public SwerveModuleState getState(){
        return state; 
    }

    public Translation2d getPosition(){
        return position; 
    }

    
    
}


