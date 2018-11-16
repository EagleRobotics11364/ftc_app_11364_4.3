package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu;

public interface MenuItem<T> {
    String getKey();
    String getDescription();

    T getValue();
    void setValue(T value);

    T iterateForward();
    T iterateBackwards();

    boolean hasNext();
    boolean hasPrevious();


}
