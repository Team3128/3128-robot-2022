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

public class NAR_Shuffleboard{

    private static class tabInfo {
        
        public NetworkTableEntry m_data;

        private Supplier<Object> m_supply;

        public tabInfo(NetworkTableEntry data, Supplier<Object> supply){
            m_supply = supply;
            m_data = data;
        }

        public void update() {
            if(m_supply == null) return;
            m_data.setValue(m_supply.get());
        }
    }

    private static HashMap<String, HashMap<String, tabInfo>> tabs;

    static {
        tabs = new HashMap<String, HashMap<String,tabInfo>>();
    }

    private static void create_tab(String tabName) {
        tabs.put(tabName, new HashMap<String,tabInfo>());
    }

    public static void addData(String tabName, String name, Supplier<Object> supply){
        addData(tabName,name,supply.get(), supply);
    }

    public static void addData(String tabName, String name, Object data){
        addData(tabName,name,data,null);
    }

    private static void addData(String tabName, String name, Object data, Supplier<Object> supply){
        if(!tabs.containsKey(tabName)){
            create_tab(tabName);
        }
        else {
            if(tabs.get(tabName).containsKey(name)) throw new IllegalArgumentException("DEBUG ISSUE");
        }
        NetworkTableEntry entry = Shuffleboard.getTab(tabName).add(name,data).getEntry();
        tabs.get(tabName).put(name, new tabInfo(entry,supply));
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