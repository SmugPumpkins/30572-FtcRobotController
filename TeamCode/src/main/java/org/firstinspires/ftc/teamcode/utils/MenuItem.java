package org.firstinspires.ftc.teamcode.utils;

public class MenuItem {
    private final String name;
    private final Menu submenu;
    private final Runnable action;

    public MenuItem(String input_name, Menu input_submenu){
        name = input_name;
        submenu = input_submenu;
        action = null;
    }
    public MenuItem(String input_name, Runnable input_action){
        name = input_name;
        submenu = null;
        action = input_action;
    }
    public Menu getMenu(){
        return submenu;
    }

    public void runAction(){
        if (action != null) action.run();
    }

    public String getName(){
        return name;
    }
    public boolean hasSubmenu() {
        return submenu != null;
    }
    public boolean hasAction(){
        return action != null;
    }
}
