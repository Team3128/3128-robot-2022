package frc.team3128.common.utility;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class NAR_Shuffleboard{

    private static class entryInfo {
        
        private NetworkTableEntry m_data;

        private Supplier<Object> m_supply;

        public entryInfo(SimpleWidget entry, Supplier<Object> supply){
            m_supply = supply;
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

    public static SimpleWidget addData(String tabName, String name, Object data){
        return addData(tabName,name,()-> data);
    }

    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply){
        if(!tabs.containsKey(tabName)){
            create_tab(tabName);
        }
        SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,supply.get());
        tabs.get(tabName).put(name, new entryInfo(entry,supply));
        return entry;
    }

    public static ComplexWidget addSubsystem(String tabName, String name, Sendable data) {
        return Shuffleboard.getTab(tabName).add(name, data);
    }

    public static void debug_slider(String tabName, String name, double min, double max) {
        if(!tabs.containsKey(tabName)){
            create_tab(tabName);
        }
        SimpleWidget tab = Shuffleboard.getTab(tabName).add(name,0)
                                        .withWidget(BuiltInWidgets.kNumberSlider)
                                        .withProperties(Map.of("min",min,"max",max));
        tabs.get(tabName).put(name, new entryInfo(tab, null));
    }

    public static Object getEntry(String tabName, String name){
        return tabs.get(tabName).get(name).m_data.getValue();
    }

    public static void update() {
        for(String i : tabs.keySet()){
            for(String j : tabs.get(i).keySet()){
                tabs.get(i).get(j).update();
            }
        }
    }

}