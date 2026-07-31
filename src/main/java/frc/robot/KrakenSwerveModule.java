package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenSwerveModule {
    private SwerveModuleState desiredState;
    private SwerveModuleState currentState;  
    private Translation2d position; 

    private static double kDt = 0.02; 
    private static double kMaxVelocity = 1.75; 
    private static double kMaxAcceleration = 0.75; 
    private static double kP = 1.0; 
    private static double kI = 0.0;
    private static double kD = 0.7;
    private static double kS = 1.1;
    private static double kA = 0;
    private static double kV = 1.3;

    private PIDController drivePID; 
    private ProfiledPIDController anglePID; 

    private SimpleMotorFeedforward angleFF; 

    private TrapezoidProfile angle_profile; 

    private TalonFX driveMotor; 
    private TalonFX angleMotor; 
    private AbsoluteEncoder angleEncoder; 

    DCMotorSim angleSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, 1), 
            DCMotor.getKrakenX60(1));

    DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.01, 1), 
            DCMotor.getKrakenX60(1));

    public KrakenSwerveModule(Translation2d position, int driveMotorID, int angleMotorID, int encoderID){

        desiredState = new SwerveModuleState();
        this.position = position; 
        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);

        drivePID = new PIDController(0.1, 0, 0); 


        angleFF = new SimpleMotorFeedforward(kS, kV, kA);
        TrapezoidProfile.Constraints angle_constraits = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
        anglePID = new ProfiledPIDController(kP, kI, kD, angle_constraits, kDt);
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
    
        double currentAngleRad = angleSim.getAngularPositionRad();
        Rotation2d currentAngle = new Rotation2d(currentAngleRad);

        desiredState.optimize(currentAngle); 
        double desiredAngleRad = desiredState.angle.getRadians();
        anglePID.setGoal(desiredAngleRad);

        double voltagePID = anglePID.calculate(currentAngleRad);
        double voltageFF = angleFF.calculate(anglePID.getGoal().velocity); 
        double voltage = voltageFF + voltagePID; 
        voltage = MathUtil.clamp(voltage,-12, 12);
    

        angleSim.setInputVoltage(voltage);
        angleSim.update(kDt);
        
        SmartDashboard.putNumber("DesiredAngle: ", Math.toDegrees(desiredAngleRad));
        SmartDashboard.putNumber("CurrentAngle: ", Math.toDegrees(currentAngleRad));
        SmartDashboard.putNumber("Error: ", anglePID.getPositionError());
        SmartDashboard.putNumber("Velocity: ", angleSim.getAngularVelocityRadPerSec());
    }

    public void calculateDrive(){
        //TODO
    }

    public void Update(){//call periodically 
        calculateAngle();
        calculateDrive();
    }
    
}


