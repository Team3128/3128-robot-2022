package frc.team3128.common.utility;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.lang.model.element.Name;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class NAR_Shuffleboard{

    private static class entryInfo {
        
        public NetworkTableEntry m_data;

        public SimpleWidget m_entry;

        private Supplier<Object> m_supply;

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

    public static SimpleWidget addData(String tabName, String name, Object data){
        return addData(tabName,name,()-> data);
    }

    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply){
        if(!tabs.containsKey(tabName)){
            create_tab(tabName);
        }
        else {
            if(tabs.get(tabName).containsKey(name)) throw new IllegalArgumentException("DEBUG ISSUE");
        }
        SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,supply.get());
        tabs.get(tabName).put(name, new entryInfo(entry,supply));
        return entry;
    }

    public static SimpleWidget getEntry(String tabName, String name){
        return tabs.get(tabName).get(name).m_entry;
    }

    public static void update() {
        for(String i : tabs.keySet()){
            for(String j : tabs.get(i).keySet()){
                tabs.get(i).get(j).update();
            }
        }
    }

    public static Object getData(String tabName, String dataName) {
        return tabs.get(tabName).get(dataName).m_data.getValue();
    }
}