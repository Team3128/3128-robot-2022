package frc.team3128;

import static org.junit.Assert.assertNotNull;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashMap;

import frc.team3128.common.utility.Log;

/**
 * Constants class to update the values of constants.
 * @author Sohan Agarkar, Mason Lam
 */

public class ConstantsInt {

    private static HashMap<String, Class<?>> categories;     // HashMap storing each class in the Constants class

    public static HashMap<String, ArrayList<Field>> editConstants;

    //Members of the Field class used to change the finality of a field
    private static Method getRoot;
    private static Field modifiers;

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

        editConstants = new HashMap<String, ArrayList<Field>>();

        for(String category : categories.keySet()){
            editConstants.put(category, new ArrayList<Field>());
        }

        initTempConstants();
    }
    
    //Init the temp constants by removing the final modifier from each field
    private static void initTempConstants() {
        /*Reflect prevents the editing of the modifiers of a field by running a security check
        preventing Field.getFields() from accessing the modifiers of a field. We can get around this
        by calling getDeclaredFields0, a method in Class that does not have a security check.*/
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
        modifiers = null;     //Store the modifiers field of the Field class
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
        //Method getRoot = null;      //Method to get the root field of a field
        try {
            getRoot = Field.class.getDeclaredMethod("getRoot");
        } catch (NoSuchMethodException | SecurityException e1) {} 
        getRoot.setAccessible(true);        //set the method to be accessible
    }

    //Change the value of a constant
    public static void updateConstant(String category, String name, String value) throws IllegalArgumentException {
        String callerClass = Thread.currentThread().getStackTrace()[2].getClassName();
        if(!callerClass.equals("frc.team3128.common.narwhaldashboard.NarwhalDashboard")) throw new IllegalArgumentException("Caller class is not valid!");
        Class<?> clazz = categories.get(category);  //Get the specified Constant class
        if(clazz == null) throw new IllegalArgumentException("Invalid Constants Sub-Class");
        try {
            Field field = clazz.getField(name); //Get the field of the specified constant
            try {
                Object toUse = parseData(value);   //Parse the value to the correct type
                Log.info("Constants Interface", field.getType().toString());
                assertNotNull(toUse);   //Check that the value is not null
                field.set(null, toUse);     //Set the value of the constant
            } catch (Throwable e) {
                Log.info("Constants Interface", "Constant Change Operation Blocked, Check If Constant Is Valid and Editable");
                throw new IllegalArgumentException("Constant Change Operation Blocked; Check if Constant is Valid and Editable");
            }
        }
        catch (NoSuchFieldException e) {
            throw new IllegalArgumentException("Constant does not exist");
        }
    }

    //Take a string and convert it to a usable type
    private static Object parseData(String value) {
        try {
            return Integer.parseInt(value);
        }
        catch(NumberFormatException e) {}

        try {
            return Double.parseDouble(value);
        } catch(NumberFormatException e) {}

        //Return a boolean value if the String is a boolean
        if (value.equalsIgnoreCase("true") || value.equalsIgnoreCase("false")) {
            return Boolean.parseBoolean(value);
        }

        return null;    //Return null if the value is not a number or boolean
    }

    //Return each field of a constants class
    public static ArrayList<Field> getConstantInfo(String category) {
        return editConstants.get(category);
    }

    //Make a constant editable
    public static void addConstant(String category, String name) throws IllegalArgumentException{
        try {
            Field field = categories.get(category).getField(name); 
            try {
                removeFinal(field);     //Make the field non-final
                editConstants.get(category).add(field);     //Make the field editable by the UI
            }
            catch(Exception e) {
                throw new IllegalArgumentException("Internal Error. Unable to unfinalize this constant");
            }
        }
        catch (NoSuchFieldException | SecurityException e) {
            throw new IllegalArgumentException(name+" does not exist");
        }
    }

    //Remove the final modifier from a constant
    private static void removeFinal(Field field) throws Exception {
            field = (Field) getRoot.invoke(field);      //get the root of the field
            modifiers.setInt(field, modifiers.getInt(field) & ~Modifier.FINAL);     //Remove the final modifier
    }
}