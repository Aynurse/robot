package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;
    private final TalonFXSimState turretSimMotor;
    private final DCMotorSim turretMotorSim;
    private double targetAngle;


    //telemetry publisher bir şey bir şey çok garip
    //network tablesa böyle fırlatıyomuşuz verileri

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable turretStateTable = inst.getTable("turretState");

    private final DoublePublisher targetAnglePublisher = turretStateTable.getDoubleTopic("turret targetAngle").publish();
    private final DoublePublisher currentAnglePublisher = turretStateTable.getDoubleTopic("turret currentAngle").publish();
    private final DoublePublisher turretkPPublisher = turretStateTable.getDoubleTopic("turret kP").publish();
    private final DoublePublisher turretkIPublisher = turretStateTable.getDoubleTopic("turret kI").publish();
    private final DoublePublisher turretkDPublisher = turretStateTable.getDoubleTopic("turret kD").publish();
    private final DoublePublisher turretkSPublisher = turretStateTable.getDoubleTopic("turret kS").publish();
    private final DoublePublisher turretkVPublisher = turretStateTable.getDoubleTopic("turret kV").publish();

        //bunlar da hangi değişkeni fırlarmak istediğimizi belirlemek içinmiş..

    private final DoubleSubscriber targetAngleSubscriber = turretStateTable.getDoubleTopic("turret targetAngle").subscribe(Constants.SwerveConstants.ZERO_TURRET);
    private final DoubleSubscriber turretkPSubscriber = turretStateTable.getDoubleTopic("turret kP").subscribe(Constants.SwerveConstants.TURRET_kP);
    private final DoubleSubscriber turretkISubscriber = turretStateTable.getDoubleTopic("turret kI").subscribe(Constants.SwerveConstants.TURRET_kI);
    private final DoubleSubscriber turretkDSubscriber = turretStateTable.getDoubleTopic("turret kD").subscribe(Constants.SwerveConstants.TURRET_KD);
    private final DoubleSubscriber turretkSSubscriber = turretStateTable.getDoubleTopic("turret kS").subscribe(Constants.SwerveConstants.TURRET_kS);
    private final DoubleSubscriber turretkVSubscriber = turretStateTable.getDoubleTopic("turret kV").subscribe(Constants.SwerveConstants.TURRET_kV);


    public TurretSubsystem() {

       

        turretMotor = new TalonFX(Constants.SwerveConstants.TURRET_MOTOR_ID);
        //burda idsini verdim

        pidController = new PIDController(Constants.SwerveConstants.TURRET_kP, Constants.SwerveConstants.TURRET_kI, Constants.SwerveConstants.TURRET_KD);
        //burda da pidyi belirliyorz..

        feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.TURRET_kS, Constants.SwerveConstants.TURRET_kV);


        turretSimMotor = turretMotor.getSimState();
        turretMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.025, 50),
        DCMotor.getKrakenX60(1));

        

        configureTurret();
    }

    public void configureTurret() {

        TalonFXConfiguration turretConfiguration = new TalonFXConfiguration();
        //burda yeni configrasyon oluştur dedim 

        turretConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //burda dururken direkt dursun dedim 

        turretConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //burda saat yönünün tersini pozitif al dedim

        turretMotor.getConfigurator().apply(turretConfiguration);
        //burda da her şeyi turretMotoruna uygula dedim
    }

    public double getAngle() {

       StatusSignal<Angle> turretPosition = turretMotor.getPosition();
       //turret motorunun pozisyonunu al

       double turretAngle = turretPosition.getValueAsDouble();
       //bunu doublea çevir

       double turretDegrees = turretAngle * 360;
       //açıya çevir

       return turretDegrees;
       //açıyı çıkar
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
        targetAngle = MathUtil.clamp(targetAngle, Constants.SwerveConstants.MIN_ANGLE, Constants.SwerveConstants.MAX_ANGLE);
        targetAnglePublisher.set(targetAngle);
        
    }



    public void periodic() {

         double currentAngle = getAngle();
        //şu anki açıyı alıyorsun

        targetAngle = targetAngleSubscriber.get();

        targetAngle = MathUtil.clamp(targetAngle, Constants.SwerveConstants.MIN_ANGLE, Constants.SwerveConstants.MAX_ANGLE);
        //yavrumun açısı çok büyük olamasın diye veriyoz..

        pidController.setP(turretkPSubscriber.get());
        //pyi veriyoz

        pidController.setI(turretkISubscriber.get());
        //isini veriyoz

        pidController.setD(turretkDSubscriber.get());

        feedforward.setKs(turretkSSubscriber.get());

        feedforward.setKv(turretkVSubscriber.get());


         double pidoutput = pidController.calculate(currentAngle, targetAngle);
        //Bu ikisi arasındaki farka göre motoru ne kadar çalıştırmalıyım

        double pidVoltage = pidoutput * Constants.SwerveConstants.BATTERY;


        StatusSignal<AngularVelocity> velocity = turretMotor.getVelocity();
        double dvelocity = velocity.getValueAsDouble();
        double ffvoltage = feedforward.calculate(dvelocity);


        double totalVoltage = pidVoltage + ffvoltage;
        

        totalVoltage = MathUtil.clamp(totalVoltage, -12, 12);
        //hız sınırı biri ve -1i aşmamasını engelliyor



        if( currentAngle >= Constants.SwerveConstants.MAX_ANGLE && totalVoltage > 0) {
            //açı max açıdan büyük ve output hala ileri gitmeye devam ediyorsa
            totalVoltage = 0;
        }

        if (currentAngle <= Constants.SwerveConstants.MIN_ANGLE && totalVoltage < 0) {
            //açı min açıdan küçük ve output hala geri gitmeye devam ediyorsa
            totalVoltage = 0;
        }

        turretMotor.setControl(new DutyCycleOut(totalVoltage / 12));
        //outputu hız olarak turret motoruna gönder

        targetAnglePublisher.set(targetAngle);
        //target angleı telemetrye veriyoruz

        currentAnglePublisher.set(currentAngle);
        //current angleı telemetrye veriyoruz

        turretkPPublisher.set(turretkPSubscriber.get());
        turretkIPublisher.set(turretkISubscriber.get());
        turretkDPublisher.set(turretkDSubscriber.get());
        turretkSPublisher.set(turretkSSubscriber.get());
        turretkVPublisher.set(turretkVSubscriber.get());


    }

    public void simulationPeriodic() {

        double simVoltage = turretSimMotor.getMotorVoltage();
        //bu simülasyondaki motorun voltjaını alıyor

        turretMotorSim.setInputVoltage(simVoltage);
        //bu da o voltajı motora koyuyor..

        turretMotorSim.update(0.02);
        //bu 20 ms'de bir updateliyor

        Angle turretPosition = turretMotorSim.getAngularPosition();
        //bu turretpozisyonunu açı olarak alıyor cnm 
        turretSimMotor.setRawRotorPosition(turretPosition);

    }
    
}