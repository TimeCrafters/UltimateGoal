package org.cyberarm.NeXT;

import android.os.Environment;
import android.util.Log;

import com.google.gson.Gson;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class StateConfiguration {
  private boolean loadSuccessful = false;
  private ArrayList<DataStruct> dataStructs;
  private HashMap<String, DataStruct> actions;

  public StateConfiguration() {
    dataStructs = new ArrayList<>();
    actions = new HashMap<>();

    // Load configuration file.
    if (loadJSON()) {
      Log.i("TC_CONFIG", "Successfully loaded configuration file.");

      for (DataStruct dataStruct : dataStructs) {
        actions.put(dataStruct.name.trim(), dataStruct);
      }

    } else {
      Log.e("TC_CONFIG", "FAILED TO LOAD CONFIGURATION FILE!");
    }
  }

  public DataStruct get(String key) throws NullPointerException {
    if (actions.get(key.trim()) == null) {
      throw(new NullPointerException("Config has no Action called \""+ key.trim() +"\""));
    }

    return actions.get(key.trim());
  }

  public boolean allow(String key) throws NullPointerException {
    if (get(key.trim()) == null) {
      throw(new NullPointerException("Config has no Action called \""+ key.trim() +"\""));
    }

    return get(key.trim()).enabled;
  }

  private boolean loadJSON() {
    boolean loadSuccessful = false;

    File file = new File(getDirectory() + File.separator + "config.json");
    StringBuilder text = new StringBuilder();

    if (file.exists()) {
      try {
        BufferedReader br = new BufferedReader(new FileReader(file));
        String line;

        while ((line = br.readLine()) != null) {
          text.append(line);
          text.append('\n');
        }
        br.close();

        Gson gson = new Gson();
        DataStruct[] array = gson.fromJson(text.toString(), DataStruct[].class);
        this.dataStructs = new ArrayList<>(Arrays.asList(array));

        loadSuccessful = true;

      } catch (IOException e) {
        System.out.println(e);
        // TODO: handle this
      }
    }

    this.loadSuccessful = loadSuccessful;
    return loadSuccessful;
  }

  private String getDirectory() {
    return Environment.getExternalStorageDirectory().getAbsolutePath() + File.separator + "FIRST_TC_CONFIG";
  }

  public boolean wasLoadSuccessful() { return loadSuccessful; }
}
