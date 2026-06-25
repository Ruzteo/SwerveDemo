package frc.robot;
import java.util.logging.Logger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenSwerveModule {
    private SwerveModuleState desiredState;
    private SwerveModuleState currentState;  
    private Translation2d position; 

    PIDController drivePID; 
    PIDController anglePID; 

    

    SimpleMotorFeedforward angleFF; 

    private TalonFX driveMotor; 
    private TalonFX angleMotor; 
    private AbsoluteEncoder angleEncoder; 

    DCMotorSim angleSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 10.0, 1), 
            DCMotor.getKrakenX60(1));

    DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 10.0, 1), 
            DCMotor.getKrakenX60(1));

    public KrakenSwerveModule(Translation2d position, int driveMotorID, int angleMotorID, int encoderID){
        desiredState = new SwerveModuleState();
        this.position = position; 
        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);

        drivePID = new PIDController(0.1, 0, 0); 

        anglePID = new PIDController(3, 0, 0.4);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);

        
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

    public void calculateAngle(){

        double currentAngleRad = angleSim.getAngularPositionRad() / 12;
        Rotation2d currentAngle = new Rotation2d(currentAngleRad);
        desiredState.optimize(currentAngle); 

        double desiredAngleRad = desiredState.angle.getRadians();
        double voltage = anglePID.calculate(currentAngleRad, desiredAngleRad);

        voltage = MathUtil.clamp(voltage, -12, 12);
        angleSim.setInputVoltage(voltage);
        angleSim.update(0.02);
        
        SmartDashboard.putNumber("DesiredAngle: ", Math.toDegrees(desiredAngleRad));
        SmartDashboard.putNumber("CurrentAngle: ", Math.toDegrees(currentAngleRad));
        //System.out.println("DesiredAngle: " + Math.toDegrees(desiredAngleRad) + " CurrentAngle: " + Math.toDegrees(currentAngleRad));
        
    }

    public void calculateDrive(){
        //TODO
    }

    public void Update(){//call periodically 
        calculateAngle();
        calculateDrive();
    }
    
}


