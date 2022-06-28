package frc.team3128.common.narwhaldashboard;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import frc.team3128.common.utility.Log;
import frc.team3128.common.hardware.limelight.*;
import frc.team3128.ConstantsInt;

public class NarwhalDashboard extends NarwhalDashboardBase {

    private static HashMap<String, Limelight> limelights = new HashMap<String, Limelight>();

    private static String selectedLimelight = null;
    private static boolean pushed = false;
    private static volatile boolean constantsChanged = true;

    public NarwhalDashboard(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
    }

    public NarwhalDashboard(InetSocketAddress address) {
        super(address);
    }

    public static void addLimelight(Limelight light) {
        limelights.put(light.hostname, light);
    }

    public static void setSelectedLimelight(Limelight ll){
        selectedLimelight = ll.hostname;
    }

    // Called once on connection with web server
    @SuppressWarnings("unchecked")
    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        pushed = false;

        (new Thread(() -> {
            while (conn.isOpen()) {

                // Call the baseOnOpen method for data not changing from year to year
                JSONObject obj = baseOnOpen(conn, handshake);

                obj.put("selected_limelight", selectedLimelight);
                if(selectedLimelight != null) {
                    obj.put("selected_pipeline", limelights.get(selectedLimelight).getSelectedPipeline());
                }

                JSONObject constantsObj = new JSONObject();
                for(String category : ConstantsInt.categories.keySet()) {
                    JSONArray catArr = new JSONArray();
                    List<Field> fields = ConstantsInt.getConstantInfo(category);
                    for(Field field : fields) {
                        try {
                        // get value from Field
                        Object value = field.get(null);
                        JSONObject newConstant = new JSONObject();
                        newConstant.put("key", field.getName());
                        newConstant.put("value", value);
    
                        catArr.add(newConstant);
                        Log.info("Narwhal Dashboard", "Constant Of "+newConstant.toJSONString());
                        }
                        catch(IllegalAccessException e) {
                            continue;
                        }
                    } 
                    constantsObj.put(category, catArr);  
                }
                obj.put("constants", constantsObj);

                if(!pushed) {
                    JSONArray autoProgramArr = new JSONArray();
                    for (String autoName : autoPrograms) {
                        autoProgramArr.add(autoName);
                    }
                    obj.put("auto", autoProgramArr);

                    JSONArray limelightsArr = new JSONArray();
                    for(Limelight lime : limelights.values()) {
                        limelightsArr.add(lime.hostname);
                    }
                    obj.put("limelight", limelightsArr);

                    JSONArray limelightsOptionsArr = new JSONArray();
                    for(Pipeline pipeline : Pipeline.values()) {
                        limelightsOptionsArr.add(pipeline.toString());
                    }
                    obj.put("pipeline", limelightsOptionsArr);

                    pushed = true;
                }
                
                conn.send(obj.toJSONString());
                
                try {
                    Thread.sleep(UPDATE_WAVELENGTH);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        })).start();
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        // Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + "
        // has closed its connection.");
        pushed = false;
    }

    // Called by request from web server 
    @Override
    public void onMessage(WebSocket conn, String message) {
        Log.info("NarwhalDashboard", message);
        String[] parts = message.split(":");

        // Calls base onMessage method for handling data not changing from year to year
        // If the message was for none of the base groups, check the year-specific ones. 
        if (!baseOnMessage(conn, message, parts)) {
            
            // Receive limelight selection (could be consolidated with pipeline)
            if(parts[0].equals("selectlimelight")){
                    selectedLimelight = parts[1];

                    if(selectedLimelight.equals("null")){
                        selectedLimelight = null;
                    } else {
                        Log.info("NarwhalDashboard", "Unable to Parse Limelight Change Request from Dashboard");
                    }

            // Receive pipeline selection (could be consolidated with limelight)
            } else if(parts[0].equals("selectpipeline")) {
                    String pipelineStr = parts[1];

                    if(pipelineStr.equals("null")){
                        pipelineStr = null;
                    } else if(limelights.containsKey(selectedLimelight)) {
                        limelights.get(selectedLimelight).setPipeline(Pipeline.valueOf(pipelineStr));
                    }
                    else {
                        Log.info("NarwhalDashboard", "Unable to Parse Pipeline Change Request from Dashboard");
                    }
            } else if(parts[0].equals("changeconstant")) {
                String category = parts[1];
                String name = parts[2];
                String value = parts[3];
                ConstantsInt.updateConstant(category, name, value);
                //constantsChanged = true;
            } else {
                Log.info("NarwhalDashboard", "Message recieved: " + message);
            }
        }
        
    }

    @Override
    public void onStart() {

    }
}
