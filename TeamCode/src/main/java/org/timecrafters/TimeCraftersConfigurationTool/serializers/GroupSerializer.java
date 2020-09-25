package org.timecrafters.TimeCraftersConfigurationTool.serializers;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

import org.timecrafters.TimeCraftersConfigurationTool.backend.config.Action;
import org.timecrafters.TimeCraftersConfigurationTool.backend.config.Group;

import java.lang.reflect.Type;

public class GroupSerializer implements JsonSerializer<Group> {
    @Override
    public JsonElement serialize(Group group, Type type, JsonSerializationContext context) {
        JsonObject container = new JsonObject();

        container.add("name", new JsonPrimitive(group.name));
        container.add("actions", context.serialize(group.getActions().toArray(), Action[].class));

        return container;
    }
}