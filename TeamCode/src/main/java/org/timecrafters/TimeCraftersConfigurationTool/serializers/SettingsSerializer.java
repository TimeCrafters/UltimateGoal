package org.timecrafters.TimeCraftersConfigurationTool.serializers;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

import org.timecrafters.TimeCraftersConfigurationTool.backend.Settings;

import java.lang.reflect.Type;

public class SettingsSerializer implements JsonSerializer<Settings> {
    @Override
    public JsonElement serialize(Settings settings, Type type, JsonSerializationContext context) {
        JsonObject container = new JsonObject();
        JsonObject result = new JsonObject();
        result.add("hostname", new JsonPrimitive(settings.hostname));
        result.add("port", new JsonPrimitive(settings.port));
        result.add("config", new JsonPrimitive(settings.config));

        container.add("data", result);

        return container;
    }
}
