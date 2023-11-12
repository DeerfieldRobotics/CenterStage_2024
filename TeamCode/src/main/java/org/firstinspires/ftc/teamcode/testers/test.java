package org.firstinspires.ftc.teamcode.testers;

import java.util.Collections;
import java.util.HashMap;
import java.util.TreeMap;

public class test {
    static HashMap<Integer, Integer> whites = new HashMap<>();
    TreeMap<Integer, Integer> tm = new TreeMap<>(Collections.reverseOrder());
    public static void main(String[] args){
        for(int i = 0; i < 20; i++){
            whites.put(100+i,i);
        }

        TreeMap<Integer, Integer> tm = new TreeMap<>(Collections.reverseOrder());

        tm.putAll(whites);

        double avg = ((int) tm.values().toArray()[0] + (int) tm.values().toArray()[1] + (int) tm.values().toArray()[2] + (int) tm.values().toArray()[3])/4.0;
        System.out.println(avg);
    }
}
