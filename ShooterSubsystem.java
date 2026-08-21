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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor;
    private double targetRPM;
    private double currentRPM;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;
    private final TalonFXSimState shooterSimMotor;
    private final DCMotorSim shooterMotorSim;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable shooterStateTable = inst.getTable("shooterState");

    //publisherlar

    private final DoublePublisher targetRPMPublisher = shooterStateTable.getDoubleTopic("shooter targetRPM").publish();
    private final DoublePublisher currentRPMPublisher = shooterStateTable.getDoubleTopic("shooter currentRPM").publish();
    private final DoublePublisher shooterkPPublisher = shooterStateTable.getDoubleTopic("shooter kP").publish();
    private final DoublePublisher shooterkIPublisher = shooterStateTable.getDoubleTopic("shooter kI").publish();
    private final DoublePublisher shooterkDPublisher = shooterStateTable.getDoubleTopic("shooter kD").publish();
    private final DoublePublisher shooterkSPublisher = shooterStateTable.getDoubleTopic("shooter kS").publish();
    private final DoublePublisher shooterkVPublisher = shooterStateTable.getDoubleTopic("shooter kV").publish();
    private final BooleanPublisher isReadyPublisher = shooterStateTable.getBooleanTopic("isFinished").publish();

    //subsricberlar veya ondan işt

    private final DoubleSubscriber targetRPMSubscriber = shooterStateTable.getDoubleTopic("shooter targetRPM").subscribe(targetRPM);
    private final DoubleSubscriber shooterkPSubscriber = shooterStateTable.getDoubleTopic("shooter kP").subscribe(Constants.SwerveConstants.SHOOTER_kP);
    private final DoubleSubscriber shooterkISubscriber = shooterStateTable.getDoubleTopic("shooter kI").subscribe(Constants.SwerveConstants.SHOOTER_kI);
    private final DoubleSubscriber shooterkDSubscriber = shooterStateTable.getDoubleTopic("shooter kD").subscribe(Constants.SwerveConstants.SHOOTER_kD);
    private final DoubleSubscriber shooterkSSubscriber = shooterStateTable.getDoubleTopic("shooter kS").subscribe(Constants.SwerveConstants.SHOOTER_kS);
    private final DoubleSubscriber shooterkVSubscriber = shooterStateTable.getDoubleTopic("shooter kV").subscribe(Constants.SwerveConstants.SHOOTER_kV);
        
    

    public ShooterSubsystem() {
        shooterMotor = new TalonFX(Constants.SwerveConstants.SHOOTER_MOTOR_ID);
        //id

        pidController = new PIDController(Constants.SwerveConstants.SHOOTER_kP, Constants.SwerveConstants.SHOOTER_kI, Constants.SwerveConstants.SHOOTER_kD);
        //pid

        feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.SHOOTER_kS, Constants.SwerveConstants.SHOOTER_kV);
        //feedforward

        shooterSimMotor = shooterMotor.getSimState();
        //simülasyondaki halini alıyor

        shooterMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.025, 1),
        DCMotor.getKrakenX60(1));
        //simğlasyonda kullanabilmek için motorun bilgilerini alıyor
        //shooter için ortalama 1, 1.5 veya 2 gibi orantılar kullanılıyor

        configureShooter();
    }

    public void configureShooter() {
        TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration();
        //yeni obje oluşturmaca 
        shooterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //motor saat yönünün tersinde dönünce pozitif dönmüş oluyo
    
        shooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //dururken brake gibi direkt durmasına gerek yok shooterın
        //güç verilme kesilince salınarak durmasını istediğim için coast yaptım

        shooterMotor.getConfigurator().apply(shooterConfiguration);
        //motoru configre ettm
    }

    public double getVelocity() {

        StatusSignal<AngularVelocity> currentVelocity = shooterMotor.getVelocity();
        //shooter motorunun hızını alıp değişkene koyuyorum 

        return currentVelocity.getValueAsDouble();
        //bu değişkeni double olarak çıkarıp döndürüyorum 

    }

    public double getRPM() {

    StatusSignal<AngularVelocity> rps = shooterMotor.getVelocity();
    //aynı şekilde hızı alıp değişkene koyuroum 

    double drps = rps.getValueAsDouble();
    //double olarak döndürdüğümü bilmek için drps yaptım ismini
    //double rps yani waow

    double rpm = drps * 60;
    //rpme dönüştürmek için 60la çarptm

    return rpm;
    }

    public void setTargetRPM(double rpm) {

        targetRPM = rpm;
        //valla sadece rpmi koyuyom bşaka bişi yok

    }

    public void periodic() {

        currentRPM = getRPM();
        //değişkene koyuyomm

        targetRPM = targetRPMSubscriber.get();

        targetRPM = MathUtil.clamp(targetRPM, 0, 6000);
        //rpmi 6000 içinde tutma denemeleri...

        //networktables

         pidController.setP(shooterkPSubscriber.get());
         pidController.setI(shooterkISubscriber.get());
         pidController.setD(shooterkDSubscriber.get());
         feedforward.setKs(shooterkSSubscriber.get());
         feedforward.setKv(shooterkVSubscriber.get());

         
        double targetRPS = targetRPM / 60;
        //rpse döndürmek için rpmi 60a bölmüşüm
        //feedforward volt istediği için bmyle bişi ypatım btw
        

        double ffVoltage = feedforward.calculate(targetRPS);
        //dediğimden


        double pidoutput = pidController.calculate(currentRPM, targetRPM);
        //pidyle hesaplama

        double pidVoltage = pidoutput * Constants.SwerveConstants.BATTERY;
        //volta dönüştürme çünkü feedforwardla çalışabilmek için volt lazım

        double totalVoltage = pidVoltage + ffVoltage;
        //evet ikisini topladm..

        totalVoltage = MathUtil.clamp(totalVoltage, 0, 12);
        //12 ve -12 arasında değerlerin olması için clamp

        double output = totalVoltage / 12;
        //sonra output -1 ve 1 arası güç istediği için 12'ye bölüoz

        shooterMotor.setControl(new DutyCycleOut(output));
        //outputa koyuyrouz

        //networktables

        shooterkPPublisher.set(shooterkPSubscriber.get());
        shooterkIPublisher.set(shooterkISubscriber.get());
        shooterkDPublisher.set(shooterkDSubscriber.get());
        shooterkSPublisher.set(shooterkSSubscriber.get());
        shooterkVPublisher.set(shooterkVSubscriber.get());
        targetRPMPublisher.set(targetRPM);
        currentRPMPublisher.set(currentRPM);

        //networktablesda kırmızı mı yeşil yanıyo onu ayarlıyor
        //aslında abs kullanmama gerek yoktu sanki btw
        boolean isReady = Math.abs(targetRPM - currentRPM) < 25;
        isReadyPublisher.set(isReady);
        
        
    }

    public void simulationPeriodic() {

        double simVoltage = shooterSimMotor.getMotorVoltage();
        //simülasyon motorundan voltaj almaca

        shooterMotorSim.setInputVoltage(simVoltage);
        //bu voltajı koymaca 

        shooterMotorSim.update(0.02);

        AngularVelocity velocity = shooterMotorSim.getAngularVelocity();
        //hız alma

        shooterSimMotor.setRotorVelocity(velocity);
        //ve onu da koyma gibi bişey


    }
    
}
