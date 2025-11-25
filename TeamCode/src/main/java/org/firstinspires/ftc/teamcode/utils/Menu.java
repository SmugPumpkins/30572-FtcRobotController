package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Menu {
    private final String name;
    private final Menu parent;
    private final ArrayList<MenuItem> options = new ArrayList<>(0);
    private int current_index = 0;
    private final Telemetry tel;
    private boolean just_fired = false;
    private Menu activeMenu = this;
    public Menu (Telemetry input_telemetry,  String input_name, Menu parent_input){
        name = input_name;
        parent = parent_input;
        tel = input_telemetry;
    }
    public Menu (Telemetry input_telemetry,  String input_name){
        name = input_name;
        parent = null;
        tel = input_telemetry;
    }

    public void addSubmenu(String addName){
        Menu menuToAdd = new Menu(tel, addName, this);
        MenuItem menuChildToAdd = new MenuItem(addName, menuToAdd);
        options.add(menuChildToAdd);
    }

    public void addAction(String addName, Runnable addAction){
        MenuItem menuActionToAdd = new MenuItem(addName, addAction);
        options.add(menuActionToAdd);
    }

    public Menu getMenuByName(String childName){
        Menu child = null;
        for (MenuItem option : options){
            if (option.getName().equals(childName)){
                child = option.getMenu();
            }
        }
        return child;
    }
    public void navigateDown(){
        current_index++;
        if (current_index > activeMenu.options.size()  -1){
            current_index = 0;
        }
    }
    public void navigateUp(){
        current_index--;
        if (current_index < 0){
            current_index = activeMenu.options.size() - 1;
        }
    }
    public void select() {
        MenuItem item = activeMenu.options.get(current_index);

        if (item.hasAction()) {
            item.runAction();
        }

        if (item.hasSubmenu()) {
            activeMenu = item.getMenu();
            current_index = 0;
        }
    }

    public void back(){
        if (activeMenu.parent == null){
            return;
        }
        activeMenu = activeMenu.parent;
        current_index = 0;

    }

    public void navigate(Gamepad pad){
        if (!just_fired) {
            if (pad.dpad_down) {
                navigateDown();
            }
            if (pad.dpad_up) {
                navigateUp();
            }
            if (pad.a){
                select();
            }
            if (pad.b){
                back();
            }
        }
        boolean[] buttons = {pad.dpad_up, pad.dpad_down, pad.a, pad.b};
        for (boolean button : buttons){
            if (button){
                just_fired = true;
                break;
            }
            just_fired = false;
        }
        updateDisplay();
    }

    public void updateDisplay(){
        tel.addLine("==|  " + activeMenu.name + "  |==");
        for (int i = 0; i < activeMenu.options.size(); i++){
            if (i == current_index){
                tel.addLine(">" + activeMenu.options.get(i).getName());
            } else {
                tel.addLine(activeMenu.options.get(i).getName());
            }
        }
    }
}

