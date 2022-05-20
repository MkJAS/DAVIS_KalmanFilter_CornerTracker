HC 43 - 120
KF 56 - 156



xd_ = -10;       %velocity
yd_ = 0;        %velocity
x_ = 203;        %initial x state. Taken from harris corner detection on octogon shape 
y_ = 82;        %initial y state 

x2_ = 215;
y2_ = 82;
x3_ = 220;
y3_ = 93;
x4_ = 216;
y4_ = 104;
x5_ = 203;
y5_ = 104;
x6_ = 198;
y6_ = 93;

% idx_f = 123;
x = (xyt{1,1}(123));                                        %initial x measurment: Found by manually going through Events to find on closest to harris corner detection
y = (xyt{1,2}(123));                                        %initial y measurment
t1 = xyt{1,3}{123}.Ns*10^-9 + xyt{1,3}{123}.s;
xd = 0.5;                                                   %initial velocity measurment not really relevant anyways
yd = 0.05;                                                  %initial y velocity measurment
acc = 50;  