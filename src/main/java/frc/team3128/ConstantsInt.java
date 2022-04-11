package frc.team3128;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3128.common.utility.Log;

public class ConstantsInt extends Constants {

    public static HashMap<String, Class<?>> categories;
    public static HashMap<String, List<Field>> constants;
    private static final List<Class<?>> primitiveNumbers;


    //How To Use: Redefine any constants you want to change through NarwhalDashboard in this class.
    //The Constants will be overridden and NarwhalDashboard will be able to change it
    //IMPORTANT: Omit "final" when temporarily adding constants to this class
    
    public static class ConversionConstants extends Constants.ConversionConstants {}
    public static class DriveConstants extends Constants.DriveConstants {}
    public static class ClimberConstants extends Constants.ClimberConstants {}
    public static class ShooterConstants extends Constants.ShooterConstants {
        // public static double SET_RPM = 3000;
        // public static double SET_ANGLE = 25;
    }
    public static class HopperConstants extends Constants.HopperConstants {}
    public static class IntakeConstants extends Constants.IntakeConstants {}
    public static class VisionConstants extends Constants.VisionConstants {}


    static {
        categories = new HashMap<String, Class<?>>();
        constants = new HashMap<String, List<Field>>();
        primitiveNumbers = Arrays.asList(int.class, long.class, double.class, byte.class, short.class);
        categories.put("ConversionConstants", ConstantsInt.ConversionConstants.class);
        categories.put("DriveConstants", ConstantsInt.DriveConstants.class);
        categories.put("ClimberConstants", ConstantsInt.ClimberConstants.class);
        categories.put("ShooterConstants", ConstantsInt.ShooterConstants.class);
        categories.put("HopperConstants", ConstantsInt.HopperConstants.class);
        categories.put("IntakeConstants", ConstantsInt.IntakeConstants.class);
        categories.put("VisionConstants", ConstantsInt.VisionConstants.class);

        for(String category : categories.keySet()) {
            constants.put(category, new ArrayList<Field>());
        }

        
        //initTempConstants();
    }

    public static List<Field> getConstantInfo(String category) {
        return constants.get(category);
    }

    public static void initTempConstants() {
        for(String cat : categories.keySet()) {
            Class<?> categoryClass = categories.get(cat);
            for(Field field : categoryClass.getFields()) {
                try {
                    Field backConstantField = categoryClass.getSuperclass().getField(field.getName());
                    field.set(null, backConstantField.get(null));
                    Log.info("Constants Interface", "Adding Constants Interface Field: "+field.getName());
                    constants.get(cat).add(field);
                } catch (NoSuchFieldException e) {
                    Log.info("Constants Interface", "Constants Field does not Exist: "+field.getName());
                } catch (SecurityException e) {
                    Log.info("Constants Interface", "Reflection API blocked by security manager");
                } catch (IllegalArgumentException e) {} 
                catch (IllegalAccessException e) {
                    Log.info("Constants Interface", "Skipping, Constant set to final: "+field.getName());
                }
            }
        }
    }


    /**
     * Updates a constant with the given name to a given value
     * @throws IllegalArgumentException constants sub-class doesn't exist
     * @throws IllegalAccessException invalid value for the given constant
     */
    public static void updateConstant(String category, String name, String value){
        Class<?> clazz = categories.get(category);
        if(clazz == null) throw new IllegalArgumentException("Invalid Constants Sub-Class");
        for(Field field : clazz.getFields()) {
            if(field.getName().equals(name)) {
                try {
                    Object toUse = value;
                    Log.info("Constants Interface", field.getType().toString());
                    if(field.getType().getSuperclass() != null && field.getType().getSuperclass().equals(Number.class)) {
                        toUse = parseNumber(value, field.getType());
                    }
                    else if(primitiveNumbers.contains(field.getType())) {
                        toUse = parseNumber(value, field.getType());
                    }
                    if(toUse == null) {
                        throw new IllegalArgumentException("Invalid value for constant");
                    }
                    field.set(null, toUse);
                } catch (IllegalAccessException e) {
                    Log.info("Constants Interface", "Constant Change Operation Blocked");
                    e.printStackTrace();
                }
                catch(IllegalArgumentException e) {
                    Log.info("Constants Interface", "Invalid value for constant");
                    e.printStackTrace();
                }
                return;
            }
        }
        throw new IllegalArgumentException("Constant does not exist");
    }

    private static Number parseNumber(String value, Class<?> type) {
        try {
        if(type.equals(Integer.class) || type.equals(int.class)) {
           return Integer.parseInt(value);
        }
        else if(type.equals(Double.class) || type.equals(double.class)) {
            return Double.parseDouble(value);
        }
        }
        catch(NumberFormatException e) {}

        return null;
    }


}