package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Set;

public class IterableTelemetryMenu {
    private List<MenuItem> allMenuItems;
    private MenuItem currentItem;
    private int position;
    private Telemetry telemetry;

    public IterableTelemetryMenu(Telemetry telemetry) {
        this.telemetry = telemetry;
        allMenuItems = new ArrayList<>();
        position = -1;
    }

    public boolean add(MenuItem menuItem) {
        if (keyAlreadyExists(menuItem.getKey())) return false;
        else {
            allMenuItems.add(menuItem);
            if (allMenuItems.size() == 1) position = 0;
            refresh();
            return true;
        }
    }

    public boolean add(MenuItem... items) {
        for (MenuItem item : items) {
            if (!keyAlreadyExists(item.getKey())) allMenuItems.add(item);
        }
        if (position == -1) position = 0;
        refresh();
        return true;
    }

    public boolean removeMenuItem(String key) {
        for (MenuItem menuItem : allMenuItems) {
            if (menuItem.getKey().equals(key)) {
                allMenuItems.remove(menuItem);
                if (allMenuItems.isEmpty()) position = -1;
                refresh();
                return true;
            }
        }
        return false;
    }

    /*public <T> MenuItem<? extends T> getMenuItem(Class<T> classOrInterface, String key) {
        for (MenuItem menuItem : allMenuItems) {
            if (menuItem.getKey().equals(key)) {
                try {
                    return (MenuItem<T>) menuItem;
                } catch (ClassCastException e) {
                    continue;
                }
            }
        }
        return null;
    }*/

    private boolean keyAlreadyExists(String keyToCheck) {
        for (MenuItem menuItem : allMenuItems) {
            if (menuItem.getKey().equals(keyToCheck)) return true;
        }
        return false;
    }

    public void nextItem() {
        if (!allMenuItems.isEmpty()) {
            if (position < allMenuItems.size()) {
                position++;
            }
        }
        refresh();
    }

    public void previousItem() {
        if (!allMenuItems.isEmpty()) {
            if (position > 0) {
                position--;
            }
        }
        refresh();
    }

    public void iterateForward() {
        if (!allMenuItems.isEmpty()) {
            allMenuItems.get(position).iterateForward();
        }
        refresh();
    }

    public void iterateBackward() {
        if (!allMenuItems.isEmpty()) {
            allMenuItems.get(position).iterateBackwards();
        }
        refresh();
    }

    public void refresh() {
        if (!allMenuItems.isEmpty()) {
            MenuItem currentItem;
            for (int i = 0; i < allMenuItems.size(); i++) {
                currentItem = allMenuItems.get(i);
                telemetry.addData(((i == position) ? "--- " : "") + currentItem.getDescription(),
                        (currentItem.hasPrevious() ? " << " : "") + currentItem.getValue() + (currentItem.hasNext() ? " >> " : ""));
            }
        }
        telemetry.update();
    }
}
