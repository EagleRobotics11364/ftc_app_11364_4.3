package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.item;

import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.item.MenuItem;

public class MenuItemInt extends MenuItem<Integer> {


    public MenuItemInt(String key, String description, int startingValue, int lowestPossible, int highestPossible) {
        super.key = key;
        super.description = description;
        super.value = startingValue;

    }

    @Override
    public Integer iterateForward() {
        return null;
    }

    @Override
    public Integer iterateBackwards() {
        return null;
    }

    @Override
    public boolean hasNext() {
        return false;
    }

    @Override
    public boolean hasPrevious() {
        return false;
    }
}
