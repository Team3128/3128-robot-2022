package frc.team3128;

import static org.junit.Assert.assertNotNull;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;

import frc.team3128.common.utility.Log;

public class ConstantsInt {

    public static HashMap<String, Class<?>> categories;     // HashMap storing each class in the Constants class

    static {
        categories = new HashMap<String, Class<?>>();
        //Add each class to the HashMap
        categories.put("ConversionConstants", Constants.ConversionConstants.class);
        categories.put("DriveConstants", Constants.DriveConstants.class);
        categories.put("ClimberConstants", Constants.ClimberConstants.class);
        categories.put("ShooterConstants", Constants.ShooterConstants.class);
        categories.put("HopperConstants", Constants.HopperConstants.class);
        categories.put("IntakeConstants", Constants.IntakeConstants.class);
        categories.put("VisionConstants", Constants.VisionConstants.class);
        
        initTempConstants();
    }
    
    //Init the temp constants by removing the final modifier from each field
    private static void initTempConstants() {
        /*Reflect prevents the editing of the modifiers of a field by running a security check
        preventing Field.getFields() from accessing the modifiers of a field. We can get around this
        by calling getDeclaredFields0, a method in Class that does not have a security change.*/
        Method getDeclaredFields0 = null;   //Method to get the declared fields of a class
        try {
            //get the non security check get fields method
            getDeclaredFields0 = Class.class.getDeclaredMethod("getDeclaredFields0", boolean.class );
        } catch (NoSuchMethodException | SecurityException e) {}
        getDeclaredFields0.setAccessible(true);     //set the method to be accessible
        Field[] fields = null;          //Store the all the fields of the Field class (this kinda confusing)
        try {
            fields = (Field[]) getDeclaredFields0.invoke(Field.class, false);
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {}
        Field modifiers = null;     //Store the modifiers field of the Field class
        for (Field field : fields) {
            //Get the modifiers field of the Field class
            if ("modifiers".equals(field.getName())) {
                modifiers = field;
                break;
            }
        }
        modifiers.setAccessible(true);      //set the modifiers field to be accessible

        /*Reflect has another security check where changing the modifiers field does not override
         the value and the modifier change is not permanent, however each field object has a root
         field, which when modified permanently changes the field*/
        Method getRoot = null;      //Method to get the root field of a field
        try {
            getRoot = Field.class.getDeclaredMethod("getRoot");
        } catch (NoSuchMethodException | SecurityException e1) {} 
        getRoot.setAccessible(true);        //set the method to be accessible
        //Cycle through each class in the Constants class
        for (Class<?> cat : categories.values()) {
            //Get the field of each constant in the class
            for (Field field : cat.getFields()) {
                try {
                    field = (Field) getRoot.invoke(field);      //get the root of the field
                    modifiers.setInt(field, modifiers.getInt(field) & ~Modifier.FINAL);     //Remove the final modifier
                } catch (IllegalArgumentException | IllegalAccessException | InvocationTargetException e) {}
            }
        }
    }

    //Change the value of a constant
    public static void updateConstant(String category, String name, String value){
        Class<?> clazz = categories.get(category);  //Get the specified Constant class
        if(clazz == null) throw new IllegalArgumentException("Invalid Constants Sub-Class");
        try {
            Field field = clazz.getField(name); //Get the field of the specified constant
            try {
                Object toUse = parseData(value);   //Parse the value to the correct type
                Log.info("Constants Interface", field.getType().toString());
                assertNotNull(toUse);   //Check that the value is not null
                field.set(null, toUse);     //Set the value of the constant
            } catch (IllegalAccessException|IllegalArgumentException e) {
                Log.info("Constants Interface", "Constant Change Operation Blocked, Check If Constant Is Valid");
                e.printStackTrace();
            }
            return;
        }
        catch (NoSuchFieldException e) {
            throw new IllegalArgumentException("Constant does not exist");
        }
    }

    //Take a string and convert it to a usable type
    private static Object parseData(String value) {
        try {
            Double number = Double.parseDouble(value);  //Convert the String to a double value
            //Return an int form of the double value if the value is an int
            if(number % 1 == 0) {
                return number.intValue();
            }
            return number;
        }
        catch(NumberFormatException e) {}

        //Return a boolean value if the String is a boolean
        if (value.equalsIgnoreCase("true") || value.equalsIgnoreCase("false")) {
            return Boolean.parseBoolean(value);
        }

        return null;    //Return null if the value is not a number or boolean
    }

    /*Get a constant from a specifc group of constants
     * Necessary because of constant folding ಠ╭╮ಠ in Java*/
    public static Object getConstant(String category, String name) {
        Field field = null; //Get the field of a constant in the specified category
        try {
            field = categories.get(category).getField(name);
        } catch (NoSuchFieldException | SecurityException e) {}
        assertNotNull(field);   //Check that field is not null
        try {
            return field.get(null); //Get the value of the constant
        } catch (IllegalArgumentException | IllegalAccessException e) {
            throw new IllegalArgumentException("Constant does not exist");
        }
    }

    //Return each field of a constants class
    public static Field[] getConstantInfo(String category) {
        return categories.get(category).getFields();
    }
}