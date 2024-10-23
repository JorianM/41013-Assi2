
env=Environment();

tablePOS=[-0.4,0,0];
ShakerPOS=[0,-0.6,0.9];
Bottle1POS=[0, 0.8, 2];
Bottle2POS=[-0.5, 0.8, 2];
Bottle3POS=[-1,0.8,2];
%load in variables.
[p1,s1,b1,b2,b3] = env.simEnvironment(tablePOS,ShakerPOS,Bottle1POS,Bottle2POS,Bottle3POS);