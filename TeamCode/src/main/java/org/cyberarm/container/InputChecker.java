package org.cyberarm.container;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;

public class InputChecker {
  private Gamepad gamepad;
  private HashMap<String, Byte> buttons;
  private ArrayList<String> buttonList;
  private byte NULL    = 0,
               PRESSED = 1,
               RELEASED= 2;
  public InputChecker(Gamepad gamepad) {
    this.gamepad = gamepad;
    buttons = new HashMap<>();
    buttonList = new ArrayList<>();

    buttonList.add("a"); buttonList.add("b"); buttonList.add("x"); buttonList.add("y");
    buttonList.add("start"); buttonList.add("guide"); buttonList.add("back");
    buttonList.add("left_bumper"); buttonList.add("right_bumper");
    buttonList.add("left_stick_button"); buttonList.add("right_stick_button");
    buttonList.add("dpad_left"); buttonList.add("dpad_right");
    buttonList.add("dpad_up"); buttonList.add("dpad_down");
  }

  public void update() {
    for (int i = 0; i < buttonList.size(); i++) {
      try {
        Field field = gamepad.getClass().getDeclaredField(buttonList.get(i));

        if (field.getBoolean(gamepad)) {
          buttons.put(buttonList.get(i), PRESSED);
        } else {
          if (buttons.get(buttonList.get(i)) != null && buttons.get(buttonList.get(i)) == PRESSED) {
            buttons.put(buttonList.get(i), RELEASED);
          }
        }
      } catch (NoSuchFieldException|IllegalAccessException e) {
        e.printStackTrace();
      }
    }
  }

  public boolean check(String button) {
    boolean state = false;
    if (buttons.containsKey(button) && buttons.get(button) == RELEASED) {
      Log.d("InputChecker","button \""+button+"\" has been released on \"gamepad"+gamepad.getGamepadId()+"\"");
      state = true;
      buttons.put(button, NULL);
    }
    return state;
  }
}
