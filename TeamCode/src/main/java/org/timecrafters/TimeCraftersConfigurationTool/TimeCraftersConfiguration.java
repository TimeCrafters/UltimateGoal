package org.timecrafters.TimeCraftersConfigurationTool;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.timecrafters.TimeCraftersConfigurationTool.backend.Config;
import org.timecrafters.TimeCraftersConfigurationTool.backend.Settings;
import org.timecrafters.TimeCraftersConfigurationTool.backend.TAC;
import org.timecrafters.TimeCraftersConfigurationTool.backend.config.Action;
import org.timecrafters.TimeCraftersConfigurationTool.backend.config.Configuration;
import org.timecrafters.TimeCraftersConfigurationTool.backend.config.Group;
import org.timecrafters.TimeCraftersConfigurationTool.backend.config.Presets;
import org.timecrafters.TimeCraftersConfigurationTool.backend.config.Variable;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.ActionDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.ActionSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.ConfigDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.ConfigSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.ConfigurationDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.ConfigurationSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.GroupDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.GroupSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.PresetsDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.PresetsSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.SettingsDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.SettingsSerializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.VariableDeserializer;
import org.timecrafters.TimeCraftersConfigurationTool.serializers.VariableSerializer;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;

public class TimeCraftersConfiguration {
    private static final String TAG = "TCT|TCConfig";
    private Config config;

    public TimeCraftersConfiguration() {
        Settings settings = loadSettings();
        this.config = loadConfig(settings.config);
    }

    public TimeCraftersConfiguration(String configName) {
        this.config = loadConfig(configName);
    }

    public Config getConfig() {
        return config;
    }

    public Group group(String groupName) {
        for (final Group group : config.getGroups()) {
            if (group.name.trim().equals(groupName.trim())) {
                return group;
            }
        }

        throw(new RuntimeException("Failed to find a group named:\"" + groupName.trim() + "\" in config \"" + config.getName() + "\""));
    }

    public Action action(String groupName, String actionName) {
        final Group group = group(groupName);

        for (Action action : group.getActions()) {
            if (action.name.trim().equals(actionName.trim())) {
                return action;
            }
        }

        throw(new RuntimeException("Failed to find an action named:\"" + actionName.trim() + "\" in group \"" + groupName.trim() + "\" in config \"" + config.getName() + "\""));
    }

    public Variable variable(String groupName, String actionName, String variableName) {
        final Action action = action(groupName, groupName);

        for (Variable variable : action.getVariables()) {
            if (variable.name.trim().equals(variableName.trim())) {
                return variable;
            }
        }

        throw(new RuntimeException("Failed to find a variable named \"" + variableName.trim() + "\" in action:\"" + actionName.trim() +
                "\" in group \"" + groupName.trim() + "\" in config \"" + config.getName() + "\""));
    }

    private Settings loadSettings() {
        File settingsFile = new File(TAC.SETTINGS_PATH);

        if (!settingsFile.exists()) {
            throw( new RuntimeException("Unable to load settings.json, file does not exist!") );
        }

        try {
            return gsonForSettings().fromJson(new FileReader(settingsFile), Settings.class);
        } catch (FileNotFoundException e) {
            throw( new RuntimeException("Unable to load settings.json") );
        }
    }

    private Config loadConfig(String name) {
        if (name.equals("")) {
            throw(new RuntimeException("Cannot load a config with an empty name!"));
        }

        String path = TAC.CONFIGS_PATH + File.separator + name + ".json";
        File configFile = new File(path);

        if (configFile.exists() && configFile.isFile()) {
            try {
                Config config = gsonForConfig().fromJson(new FileReader(configFile), Config.class);
                config.setName(name);

                return config;
            } catch (FileNotFoundException e) {
                e.printStackTrace();
                throw(new RuntimeException("Unable to find a config file named \"" + name + "\""));
            }
        } else {
            throw(new RuntimeException("Unable to find a config file named \"" + name + "\""));
        }
    }

    private Gson gsonForSettings() {
        return new GsonBuilder()
                .registerTypeAdapter(Settings.class, new SettingsSerializer())
                .registerTypeAdapter(Settings.class, new SettingsDeserializer())
                .create();
    }

    public Gson gsonForConfig() {
        return new GsonBuilder()
                .registerTypeAdapter(Config.class, new ConfigSerializer())
                .registerTypeAdapter(Config.class, new ConfigDeserializer())

                .registerTypeAdapter(Configuration.class, new ConfigurationSerializer())
                .registerTypeAdapter(Configuration.class, new ConfigurationDeserializer())

                .registerTypeAdapter(Group.class, new GroupSerializer())
                .registerTypeAdapter(Group.class, new GroupDeserializer())

                .registerTypeAdapter(Action.class, new ActionSerializer())
                .registerTypeAdapter(Action.class, new ActionDeserializer())

                .registerTypeAdapter(Variable.class, new VariableSerializer())
                .registerTypeAdapter(Variable.class, new VariableDeserializer())

                .registerTypeAdapter(Presets.class, new PresetsSerializer())
                .registerTypeAdapter(Presets.class, new PresetsDeserializer())
                .create();
    }
}
