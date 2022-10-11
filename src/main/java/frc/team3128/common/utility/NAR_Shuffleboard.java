package frc.team3128.common.utility;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class NAR_Shuffleboard{

    private static class entryInfo {
        
        private NetworkTableEntry m_data;

        private Supplier<Object> m_supply;

        private SimpleWidget m_entry;

        public entryInfo(SimpleWidget entry, Supplier<Object> supply){
            m_supply = supply;
            m_entry = entry;
            m_data = entry.getEntry();
        }

        public void update() {
            if(m_supply == null) return;
            m_data.setValue(m_supply.get());
        }
    }

    private static HashMap<String, HashMap<String, entryInfo>> tabs;

    static {
        tabs = new HashMap<String, HashMap<String,entryInfo>>();
    }

    private static void create_tab(String tabName) {
        tabs.put(tabName, new HashMap<String,entryInfo>());
    }

    public static SimpleWidget addData(String tabName, String name, Object data) {
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,data);
        tabs.get(tabName).put(name, new entryInfo(entry,null));
        return entry;
    }

    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply){
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,supply.get());
        tabs.get(tabName).put(name, new entryInfo(entry,supply));
        return entry;
    }

    public static void put(String tabName, String name, Object data) {
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        if (tabs.get(tabName).containsKey(name)) {
            tabs.get(tabName).get(name).m_data.setValue(data);
            return;
        }
        addData(tabName, name, data);
    }

    public static ComplexWidget addComplex(String tabName, String name, Sendable data) {
        return Shuffleboard.getTab(tabName).add(name, data);
    }

    public static DoubleSupplier debug(String tabName, String name) {
        if(!tabs.containsKey(tabName)){
            create_tab(tabName);
        }
        SimpleWidget tab = addData(tabName,name,0);
        return ()-> tab.getEntry().getDouble(0);
    }

    public static HashMap<String,DoubleSupplier> PID_Setup(String tabName, String suffix) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        HashMap<String,DoubleSupplier> PID_C = new HashMap<String,DoubleSupplier>();
        for (String i : new String[]{"KF","KP","KI","KD","SETPOINT"}) {
            NetworkTableEntry entry = tab.add(suffix + "_" + i,0).getEntry();
            PID_C.put(i,()-> entry.getDouble(0));
        }
        return PID_C;
    }

    public static Object getValue(String tabName, String name){
        return tabs.get(tabName).get(name).m_data.getValue();
    }

    public static SimpleWidget getEntry(String tabName,String name) {
        return tabs.get(tabName).get(name).m_entry;
    }

    public static void update() {
        for(String i : tabs.keySet()){
            for(String j : tabs.get(i).keySet()){
                tabs.get(i).get(j).update();
            }
        }
    }

}