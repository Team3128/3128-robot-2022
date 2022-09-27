package frc.team3128.subsystems;
//import javax.management.DescriptorKey;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import frc.team3128.Constants.CompressorConstants;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.util.AllocationException;


public class Compressor extends Object implements AutoCloseable, Sendable {

    private PneumaticsBase m_module;
    // private boolean pressureSwitch = false;
    // public boolean enabled = true;
    // public boolean closedLoop = true;


    
    public Compressor(int mod, PneumaticsModuleType modType){
        // module = mod;
        // m_module = modType;

        m_module = PneumaticsBase.getForType(mod, modType);

        if (!m_module.reserveCompressor()){
            m_module.close();
            throw new AllocationException("Compressor already allocated");
        }

        m_module.enableCompressorDigital();

        HAL.report(tResourceType.kResourceType_Compressor, mod + 1);
        SendableRegistry.addLW(this, "Compressor", mod);
    }

    public Compressor(PneumaticsModuleType modType){
        this(PneumaticsBase.getDefaultForType(modType), modType);
    }
    
    // public void close(PneumaticsModuleType mod) throws Exception{
    //     Compressor com = new Compressor(mod);
    //     com.close();
    // }



    public void close(){ //throws exception needed?
        m_module.close();
        SendableRegistry.remove(this);
        m_module.unreserveCompressor();
        m_module = null;
    }

    @Deprecated(since="2022", forRemoval = true)
    public void stop(){
        //setClosedLoop(false);
        disable(); //defined later
    }

    @Deprecated(since="2022", forRemoval = true)
    public void start(){
        //setClosedLoop(true);
        enableDigital(); //defined later
    }

    public boolean enabled(){
        return m_module.getCompressor();
    }
    
    public boolean getPressureSwitchValue(){
        return m_module.getPressureSwitch();
    }

    public double getCurrent(){
        return m_module.getCompressorCurrent();
    }

    public double getAnalogVoltage(){
        return m_module.getAnalogVoltage(0);
    }

    public double getPressure(){
        return m_module.getPressure(0);
    }

    public void disable(){
        m_module.disableCompressor();
    }

    public void enableDigital(){
        m_module.enableCompressorDigital();
    }

    public void enableAnalog(double minPressure, double maxPressure){
        m_module.enableCompressorAnalog(minPressure, maxPressure);
    }

    public void enableHybrid(double minPressure, double maxPressure){
        m_module.enableCompressorHybrid(minPressure, maxPressure);
    }

    public CompressorConfigType getConfigType(){
        return m_module.getCompressorConfigType();
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Compressor");
        builder.addBooleanProperty("Enabled", this::enabled, null);
        builder.addBooleanProperty("Pressure Switch", this::getPressureSwitchValue, null);
    }

}
