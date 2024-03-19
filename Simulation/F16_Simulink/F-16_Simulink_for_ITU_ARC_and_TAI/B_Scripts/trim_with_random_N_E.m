clear all
clc

load("C:\Users\mdemir\Desktop\Aircraft_model\08M_500m_-15degGamma_Trim.mat")

statesFinal(10) = 0 + (2000-0)*rand(1);
statesFinal(11) = 0 + (2000-0)*rand(1);