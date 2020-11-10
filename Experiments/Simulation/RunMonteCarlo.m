clear;

filename = 'BSpline_d3_step5_300s';
% filename = 'BSplineRandom_d3_15s';

numCalibrations = 1000;

rng('shuffle');
MonteCarloCalibration(filename, numCalibrations);