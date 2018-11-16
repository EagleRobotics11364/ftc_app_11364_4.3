package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.item;

public abstract class MenuItem<T> {
    protected T value;
    protected String key;
    protected String description;

    public String getKey() {
        return key;
    }

    public String getDescription() {
        return description;
    }

    public T getValue() {
        return value;
    }

    public void setValue(T value) {
        this.value = value;
    }

    public abstract T iterateForward();
    public abstract T iterateBackwards();

    public abstract boolean hasNext();
    public abstract boolean hasPrevious();


}
