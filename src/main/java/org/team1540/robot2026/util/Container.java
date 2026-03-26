package org.team1540.robot2026.util;

// Stupid Java lambda fix
public class Container<T> {
    public T value;

    public Container() {}

    public Container(T initialValue) {
        value = initialValue;
    }
}
