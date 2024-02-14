package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;

public class Kalman {

    public ArrayList<ArrayList<Double> > kalmanFilter(ArrayList<Double> measurements){
        double estimate = measurements.get(0);
        double prevEstimate = estimate;

        ArrayList<Double> estimates, K_vals;
        estimates.add(0);

        double Q = 0.4, R = 3, P = 1, K = 0;
        
        for(int i = 0; i < measurements.size(); i++){
            P += Q;
            K = P/(P+R);
            K_vals.add(K);
            P = (1-K)*P;
        }

        for(int i = 1; i < measurements.size(); i++){
            estimate += model_of_velo.get(i) - model_of_velo(i-1);
            estimates.add(estimate);
            previousEstimate = estimate;
        }

        ArrayList<ArrayList<Double> > ret = {estimates, K_vals};
    }
}