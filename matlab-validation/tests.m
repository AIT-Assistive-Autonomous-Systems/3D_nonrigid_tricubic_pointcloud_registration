clc; clear; close;

tcivoxel = TCIVoxel;

tcivoxel.f = readmatrix('../f_test-values.txt');
xn = [0.2 0.4 0.6]';
p = tcivoxel.p(xn)