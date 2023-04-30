clear;

IMUcsv = 'Lab4IMU.csv';
GPScsv = 'Lab4GPS.csv';

IMU = readmatrix(IMUcsv);
GPS = readmatrix(GPScsv);

sizeIMU = length(IMU(:,1));
sizeGPS = length(GPS(:,1));

Time = IMU(:,1);
FirstTimeNum = IMU(1,1);
Time = Time - FirstTimeNum;
TimeGPS = GPS(:,1);
FirstTimeNum = GPS(1,1);
TimeGPS = TimeGPS - FirstTimeNum;

Lat = GPS(:,6);
Lon =  GPS(:,7);
Alt = GPS(:,8);
UTM_east = GPS(:,9);
UTM_north = GPS(:,10);

Roll = IMU(:,6);
Pitch = IMU(:,7);
Yaw = IMU(:,8);
GyroX = IMU(:,18);
GyroY = IMU(:,19);
GyroZ = IMU(:,20);
AccX = IMU(:,22);
AccY = IMU(:,23);
AccZ = IMU(:,24);
MagX = IMU(:,30);
MagY = IMU(:,31);
MagZ = IMU(:,32);
%% 
%{
%To eyeball and find roundabout entry/exit timing
figure
plot(Time,MagX)
title('Finding circle timing');
xlabel('Time(in Seconds)');
ylabel('Magnetic Field along X');
%roughly 85s -> 170s
%}

RoundTime = [];
RoundMagX = [];
RoundMagY = [];
RoundMagZ = [];
RoundGyroZ = [];

%Extracting the roundabout data
for i = 1:sizeIMU
    if (Time(i) > 85) && (Time(i) < 170)
        RoundTime = [RoundTime; Time(i)];
        RoundMagX = [RoundMagX; MagX(i)];
        RoundMagY = [RoundMagY; MagY(i)];
        RoundMagZ = [RoundMagZ; MagZ(i)];
        RoundGyroZ = [RoundGyroZ; GyroZ(i)];
    end
end
%% 

% Hard Iron Offset

MagXoffset = (max(RoundMagX) + min(RoundMagX)) / 2;
MagYoffset = (max(RoundMagY) + min(RoundMagY)) / 2;
MagZoffset = (max(RoundMagZ) + min(RoundMagZ)) / 2;

%Hard-iron for roundabout alone
%CorX = RoundMagX - MagXoffset;
%CorY = RoundMagY - MagYoffset;

%{
figure
subplot(2,1,1)
scatter(RoundMagX, RoundMagY)
title('Magnetic Field along X & Y on roundabout');
xlabel('Magnetic Field along X');
ylabel('Magnetic Field along Y');
zlabel('Time(in Seconds)');
subplot(2,1,2)
scatter(CorX, CorY)
title('Corrected Magnetic Field along X & Y on roundabout');
xlabel('Corrected Magnetic Field along X');
ylabel('Corrected Magnetic Field along Y');
zlabel('Time(in Seconds)');
%}

% Soft Iron 

MagXavg = (max(RoundMagX) - min(RoundMagX)) / 2;
MagYavg = (max(RoundMagY) - min(RoundMagY)) / 2;
MagZavg = (max(RoundMagZ) - min(RoundMagZ)) / 2;

Avg_offset = (MagXavg + MagYavg + MagZavg) / 3;

MagXscalingFactor = Avg_offset / MagXavg;
MagYscalingFactor = Avg_offset / MagYavg;
MagZscalingFactor = Avg_offset / MagZavg;

% Corrected Readings

CorrectedMagX = (MagX - MagXoffset) * MagXscalingFactor;
CorrectedMagY = (MagY - MagYoffset) * MagYscalingFactor;
CorrectedMagZ = (MagZ - MagZoffset) * MagZscalingFactor;

%% 

% Corrected Readings for circle part
CorrectedRoundMagX = (RoundMagX - MagXoffset) * MagXscalingFactor;
CorrectedRoundMagY = (RoundMagY - MagYoffset) * MagYscalingFactor;
CorrectedRoundMagZ = (RoundMagZ - MagZoffset) * MagZscalingFactor;

%{
figure
subplot(2,1,1)
scatter(RoundMagX, RoundMagY)
title('Magnetic Field along X & Y on roundabout');
xlabel('Magnetic Field along X');
ylabel('Magnetic Field along Y');
zlabel('Time(in Seconds)');
subplot(2,1,2)
scatter(CorrectedRoundMagX, CorrectedRoundMagY)
title('Corrected Magnetic Field along X & Y on roundabout');
xlabel('Corrected Magnetic Field along X');
ylabel('Corrected Magnetic Field along Y');
zlabel('Time(in Seconds)');
%}
%{
%MagX Comparison
figure
subplot(2,1,1)
plot(Time, MagX)
title('Magnetic Field along X vs Time');
xlabel('Time(in Seconds)');
ylabel('Magnetic Field along X');
subplot(2,1,2)
plot(Time, CorrectedMagX)
title('Corrected Magnetic Field along X vs Time');
xlabel('Time(in Seconds)');
ylabel('Corrected Magnetic Field along X');

%MagY Comparison
figure
subplot(2,1,1)
plot(Time, MagY)
title('Magnetic Field along Y vs Time');
xlabel('Time(in Seconds)');
ylabel('Magnetic Field along Y');
subplot(2,1,2)
plot(Time, CorrectedMagY)
title('Corrected Magnetic Field along Y vs Time');
xlabel('Time(in Seconds)');
ylabel('Corrected Magnetic Field along Y');
%}
%% 

% Calculating Yaw Angle from corrected Magnetometer reading
YawFromMag = atan2(CorrectedMagY,CorrectedMagX);

% Yaw Angle from Gyroscope(Integration)
YawFromGyro = cumtrapz(1/40,GyroZ);
YawFromGyroWrapped = wrapToPi(YawFromGyro);
%{
figure
subplot(2,1,1)
plot(Time,YawFromMag)
title('Yaw Angle from Corrected Magnetometer Reading vs Time');
xlabel('Time(in Seconds)');
ylabel('Yaw Angle');
subplot(2,1,2)
plot(Time,YawFromGyroWrapped)
title('Yaw Angle from Gyroscope vs Time');
xlabel('Time(in Seconds)');
ylabel('Yaw Angle');
%}
%% 

% Using low-pass filter for Magnetometer Yaw Angle
% We choose arbitrary filter value lower then fs(frequency = 40)
lowPassFilter = 12;
%figure;
%lowpass(YawFromMag, lowPassFilter, 40);
YawFromMagFiltered = lowpass(YawFromMag, lowPassFilter, 40);

% Using high-pass filter for Gyro Yaw Angle
% We choose arbitrary filter value higher than fs(40)
highPassFilter = 28;
%figure;
%highpass(YawFromGyroWrapped, highPassFilter, 40);
YawFromGyroFiltered = highpass(YawFromGyroWrapped, highPassFilter, 40);

% Complementary filter
YawComplementary = YawFromMagFiltered + YawFromGyroFiltered;
YawComplementary = YawComplementary * 180/pi; %convert to degrees
%{
plot(Time, YawComplementary)
title('Yaw Angle from Complementary Filter vs Time');
xlabel('Time(in Seconds)');
ylabel('Yaw Angle');
%}
%{
subplot(2,1,1)
plot(Time, YawComplementary)
title('Yaw Angle from Complementary Filter vs Time');
xlabel('Time(in Seconds)');
ylabel('Yaw Angle');
subplot(2,1,2)
plot(Time, Yaw)
title('Yaw Angle from IMU Reading vs Time');
xlabel('Time(in Seconds)');
ylabel('Yaw Angle');
%}
%% 

%Taking data for bias
PostTime = [];
PostAccX = [];
PostGyroZ = [];
PostAccY = [];
for i = 1:sizeIMU
    if (Time(i) < 210) && (Time(i) > 170)
        PostTime = [PostTime; Time(i)];
        PostAccX = [PostAccX; AccX(i)];
        PostAccY = [PostAccY; AccY(i)];
        PostGyroZ = [PostGyroZ; GyroZ(i)];
    end
end

%data after 175s
TestTime = [];
TestAccX = [];
TestGyroZ = [];
TestAccY = [];
TestYawComplementary = [];
for i = 1:sizeIMU
    if (Time(i) > 175)
        TestTime = [TestTime; Time(i)-175];
        TestAccX = [TestAccX; AccX(i)];
        TestAccY = [TestAccY; AccY(i)];
        TestGyroZ = [TestGyroZ; GyroZ(i)];
        TestYawComplementary = [TestYawComplementary; YawComplementary(i)];
    end
end
TestTimeGPS = [];
TestLat = [];
TestLon = [];
TestUTME = [];
TestUTMN = [];
for i = 1:sizeGPS
    if (TimeGPS(i) > 175)
        TestTimeGPS = [TestTimeGPS; TimeGPS(i)-175];
        TestLat = [TestLat; Lat(i)];
        TestLon = [TestLon; Lon(i)];
        TestUTME = [TestUTME; UTM_east(i)];
        TestUTMN = [TestUTMN; UTM_north(i)];
    end
end
%% 

%Integrating Linear Acceleration along X
FwdVelocityFromLin = cumtrapz(1/40, TestAccX);
FwdVelocityFromLin = FwdVelocityFromLin * 3.6; %m/s -> km/h

%Finding Velocity via GPS data
dLat = diff(TestLat);
dLon = diff(TestLon);
DistDeg = [];
for i = 1:size(TestLat) - 1
    a = hypot(dLat(i), dLon(i));
    DistDeg = [[DistDeg];a];
end                                                                    
d2m = 4E+7/360;                                                     
DistMtr = DistDeg * d2m;                                            
dTime = diff(TestTimeGPS);                                                 
Velocity = DistMtr ./ dTime; 
FwdVelocityFromGPS = Velocity * 3.6; %kmph
%{
subplot(2,1,1)
plot(TestTime, FwdVelocityFromLin);
title('Forward Velocity from Integrating Linear Acceleration along X vs Time');
xlabel('Time(in Seconds)');
ylabel('Velocity in km/h');
subplot(2,1,2)
plot(TestTimeGPS(2:size(TestTimeGPS)), FwdVelocityFromGPS)
title('Forward Velocity from GPS vs Time');
xlabel('Time(in Seconds)');
ylabel('Velocity in km/h');
%}
%% 

%Adjusting Velocity from Integration
AccXX = TestAccX - mean(TestAccX);
FwdVelocityFromLinX = cumtrapz(1/40, AccXX);
FwdVelocityFromLinXX = FwdVelocityFromLinX - min(FwdVelocityFromLinX);
%{
figure
plot(TestTime,FwdVelocityFromLinXX);
title('Forward Velocity after removing bias');
xlabel('Time(in Seconds)');
ylabel('Velocity in km/h');
%}
%% 

%displacement from IMU vs GPS

DisplacementIMU = cumtrapz(1/40,FwdVelocityFromLinXX)/1000;
DisplacementGPS = cumsum(FwdVelocityFromGPS)/1000;
%{
subplot(2,1,1)
plot(TestTime, DisplacementIMU);
title('Displacement from Integrating Linear Velocity along X vs Time');
xlabel('Time(in Seconds)');
ylabel('Displacement in Km');
subplot(2,1,2)
plot(TestTimeGPS(2:size(TestTimeGPS)), DisplacementGPS)
title('Displacement from GPS vs Time');
xlabel('Time(in Seconds)');
ylabel('Displacement in Km');
%}
%% 

% X**obs = X**, we take the FwdVelocity found from removing acceleration bias*

FwdVelocity = cumtrapz(1/40,TestAccX);

% Omega x X* = Y**

Omega = TestGyroZ;
SideAcc = FwdVelocity .* Omega;

%Comparing Y** to Y**obs
%{
figure;
hold on;
plot(TestTime, SideAcc);
plot(TestTime, TestAccY);
title('Accelaration vs Time');
xlabel('Time(in Seconds)');
ylabel('Acceleration in m/s^2');
hold off;
legend('Acceleration Y from X*xOmega','Acceleration Y from sensor')
%}
%% 

%Finding the easting and northing component of the filtered acceleration
A = cos(TestYawComplementary);
B = sin(TestYawComplementary);
AccNorth = AccXX .* B;
AccEast = AccXX .* A;

VelNorth = cumtrapz(1/40,AccNorth);
VelEast = cumtrapz(1/40,AccEast);

DisNorth = cumtrapz(1/40,VelNorth);
DisEast = cumtrapz(1/40,VelEast);
%{
subplot(2,1,1)
plot(TestUTME, TestUTMN);
title('North vs East displacement pathing from GPS');
xlabel('Easting');
ylabel('Northing');
subplot(2,1,2)
plot(DisEast,DisNorth);
title('North vs East displacement pathing from Dead Reckoning method');
xlabel('Easting');
ylabel('Northing');
%}
%% 

%Finding Xc 
Xc = TestAccY - SideAcc;
OmegaDot = diff(TestGyroZ);
Xc = Xc(2:size(Xc)) ./ OmegaDot;

figure;
plot(TestTime(2:size(TestTime)), Xc);
title('Offset vs Time');
xlabel('Time(in seconds)');
ylabel('Offset');

Xcc = [];
%Remove infinites to find mean
for i = 1:size(Xc)
    if (Xc(i) > -999999999) && (Xc(i) < 999999999)
        Xcc = [Xcc;Xc(i)];
    end
end

MeanXcc = mean(Xcc);





