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

/**
 * Wrapper for {@link Shuffleboard}
 * @author Mason Lam
 * @since 2022 RAPID REACT
 */
public class NAR_Shuffleboard {

    /**
     * Storage class for NAR_Shuffleboard
     */
    private static class entryInfo {
        
        private NetworkTableEntry m_data;

        private Supplier<Object> m_supply;

        private SimpleWidget m_entry;

        /**
         * Creates a new entry Info
         *
         * @param entry Widget where the entry 
         * @param supply supplier updating the entry
         */
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

    private static HashMap<String, HashMap<String, entryInfo>> tabs = new HashMap<String, HashMap<String,entryInfo>>();;

    /**
   * Creates a new tab entry
   *
   * @param tabName the title of the new tab
   */
    private static void create_tab(String tabName) {
        tabs.put(tabName, new HashMap<String,entryInfo>());
    }

    /**
   * Displays a value in Shuffleboard
   *
   * @param tabName the title of the tab to select
   * @param name the name of the entry
   * @param data value to display
   * @return simple widget that can be modified
   */
    public static SimpleWidget addData(String tabName, String name, Object data) {
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,data);
        tabs.get(tabName).put(name, new entryInfo(entry,null));
        return entry;
    }

    /**
     * Displays an updating value in Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param supply object supplier to constantly update value
     * @return simple widget that can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply){
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,supply.get());
        tabs.get(tabName).put(name, new entryInfo(entry,supply));
        return entry;
    }

    /**
     * Puts a value in Shuffleboard similar to SmartDashboard, use in periodic
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param data value to display
     * @param x -coord of the entry
     * @param y -coord of the entry
     */
    public static void put(String tabName, String name, Object data, int x, int y) {
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        if (tabs.get(tabName).containsKey(name)) {
            tabs.get(tabName).get(name).m_data.setValue(data);
            return;
        }
        addData(tabName, name, data).withPosition(x,y);
    }

    /**
     * Displays complex values, like subsystems and command, works on all classes that extend sendable
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param data complex value to display
     * @return complex widget that can be modified
     */
    public static ComplexWidget addComplex(String tabName, String name, Sendable data) {
        return Shuffleboard.getTab(tabName).add(name, data);
    }

    /**
     * Creates a debug entry, allows user to edit variable from Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param Default starting value for the entry
     * @param x x-coord of the entry
     * @param y y-coord of the entry
     * @return DoubleSupplier containing the value in the entry
     */
    public static DoubleSupplier debug(String tabName, String name, double Default, int x, int y) {
        if(!tabs.containsKey(tabName)){
            create_tab(tabName);
        }
        SimpleWidget tab = addData(tabName,name,Default);
        tab.withPosition(x,y);
        return ()-> tab.getEntry().getDouble(Default);
    }

    /**
     * Creates a quick PID Tuning setup
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param suffix String that goes before PID entry names
     * @return HashMap with keys "KF","KP","KI","KD", and "SETPOINT"
     */
    public static HashMap<String,DoubleSupplier> PID_Setup(String tabName, String suffix) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        HashMap<String,DoubleSupplier> PID_C = new HashMap<String,DoubleSupplier>();
        for (String i : new String[]{"KF","KP","KI","KD","SETPOINT"}) {
            NetworkTableEntry entry = tab.add(suffix + "_" + i,0).getEntry();
            PID_C.put(i,()-> entry.getDouble(0));
        }
        return PID_C;
    }

    /**
     * Get the value from an entry
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @return Object stored in the entry
     */
    public static Object getValue(String tabName, String name){
        return tabs.get(tabName).get(name).m_data.getValue();
    }

    /**
     * Get the Simple Widget object from an entry
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @return SimpleWidget stored in the entry
     */
    public static SimpleWidget getEntry(String tabName,String name) {
        return tabs.get(tabName).get(name).m_entry;
    }

    /**
     * Updates every entry
     */
    public static void update() {
        for(String i : tabs.keySet()){
            for(String j : tabs.get(i).keySet()){
                tabs.get(i).get(j).update();
            }
        }
    }

}