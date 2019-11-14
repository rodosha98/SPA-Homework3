clear all; clc;
ptCloud = pcread('cloud1.ply');
pcshow(ptCloud);
ptCloud.Location;
% remove NaN valuse from the cloud
ptCloudOut = removeInvalidPoints(ptCloud);


% remove noise from the Cloud
ptCloudMain = pcdenoise(ptCloudOut)
figure(1);
pcshow(ptCloudMain)
title('Point Cloud');

%% Box definition

n = length(ptCloudMain.Color); 

% desired color of  the Box EGB
r = 90:160;
g = 0:70;
b = 0:70;
lr = length(r);
lg = length(g);
lb = length(b);

k = 1;

for i = 1:n
    for ir = 1:lr
        if ptCloudMain.Color(i,1) == r(ir)
            for ig = 1:lg  
                if ptCloudMain.Color(i,2) == g(ig)
                    for ib = 1:lb
                        if ptCloudMain.Color(i,3) == b(ib)
                            k = k+1 ;
                        end
                    end
                end
            end
        end
    end
end
                        


%% New Cloud of points

box_points = zeros(k, 3);
box_color = zeros(k,3)
kp = 1;

for i = 1:n
    for ir = 1:lr
        if ptCloudMain.Color(i,1) == r(ir)
            for ig = 1:lg  
                if ptCloudMain.Color(i,2) == g(ig)
                    for ib = 1:lb
                        if ptCloudMain.Color(i,3) == b(ib)
                           box_points(kp,:) = ptCloudMain.Location(i,:);
                           box_color(kp,:) = ptCloudMain.Color(i,:);
                           kp = kp + 1;
                        end
                    end
                end
            end
        end
    end
end

%%
boxCloud = pointCloud(box_points, 'Color', uint8(box_color));
figure(2);
pcshow(boxCloud)
title('Red Box Cloud')        
%% Plane Fitting
maxDistance = 0.04;
refvector = [0,0,1];
maxAngularDistance = 5;

[model,inlierIndices,outlierIndices] = pcfitplane(boxCloud,maxDistance, refvector);

plane2 = select(boxCloud,inlierIndices);
remainPtCloud = select(boxCloud,outlierIndices);
figure(3);
pcshow(remainPtCloud)
title('Remaining Point Cloud')

figure(4);
pcshow(plane2)
title('Plane2')

%% Centre 1 
n1 = length(plane2.Location)

x1s = 0;
x1sum = 0;
x1min1 = plane2.Location(1,1);
x1min2 =plane2.Location(1,1);
x1min3 = plane2.Location(1,1);
x1max1 = plane2.Location(1,1);
x1max2 = plane2.Location(1,1);
x1max3 = plane2.Location(1,1);

y1s = 0;
y1sum = 0;
y1min1 =plane2.Location(1,2);
y1min2 =plane2.Location(1,2);
y1min3 =plane2.Location(1,2);
y1max1 = plane2.Location(1,2);
y1max2 = plane2.Location(1,2);
y1max3 = plane2.Location(1,2);

z1s = 0
z1sum = 0;
z1min1 = plane2.Location(1,3);
z1min2 = plane2.Location(1,3);
z1min3 = plane2.Location(1,3);

z1max1 = plane2.Location(1,3);
z1max2 = plane2.Location(1,3);
z1max3 = plane2.Location(1,3);
for i = 1:n1
    x1s = plane2.Location(i,1);
    x1sum = x1sum + x1s;
    if x1s > x1max1
        x1max1 = x1s
        y1max1 = plane2.Location(i,2);
        z1max1 = plane2.Location(i,3);
    elseif x1s < x1min1
        x1min1 = x1s
        y1min1 = plane2.Location(i,2);
        z1min1 = plane2.Location(i,3);
    end
    
    y1s = plane2.Location(i,2);
    y1sum = y1sum + y1s;
     if y1s > y1max2
        y1max2 = y1s
        x1max2 = plane2.Location(i,1);
        z1max2 = plane2.Location(i,3);
    elseif y1s < y1min2
        y1min2 = y1s
        x1min2 = plane2.Location(i,1);
        z1min2 = plane2.Location(i,3);
    end
    
    z1s = plane2.Location(i,3);
    z1sum = z1sum + z1s;
     if z1s > z1max3
        z1max3 = z1s
        y1max3 = plane2.Location(i,2);
        x1max3 = plane2.Location(i,1);
    elseif z1s < z1min3
        z1min3 = z1s
        y1min3 = plane2.Location(i,2);
        x1min3 = plane2.Location(i,1);
    end
    
end

xc1 = (x1sum)/n1;
yc1 = (y1sum)/n1;
zc1 = (z1sum)/n1;
centre1 = [xc1; yc1; zc1];

%%

figure(5);
hold on
pcshow(plane2)
title('Plane 2')
% Center
scatter3(xc1, yc1, zc1,'b*')

%scatter3(x1max1, y1max1, z1max1,'g*')
scatter3(x1max2, y1max2, z1max2,'g*')
%scatter3(x1max3, y1max3, z1max3,'g*')


%scatter3(x1min1, y1min1, z1min1,'r*')
scatter3(x1min2, y1min2, z1min2,'r*')
%scatter3(x1min3, y1min3, z1min3,'r*')

%Rough estimation of height

h =   y1max2 - y1min2 ;





%%
 maxDistance3 = 0.001;

[model3,inlierIndices3,outlierIndices3] = pcfitplane(remainPtCloud,maxDistance3);
plane3 = select(remainPtCloud,inlierIndices3);
figure(6);
pcshow(plane3)
title('Plane 3')


%% Centre 2 
n2 = length(plane3.Location)

xs = 0;
x2sum = 0;
x2max1 = plane3.Location(1,1);
x2max2 = plane3.Location(1,1);
x2max3 = plane3.Location(1,1);
x2min1 = plane3.Location(1,1);
x2min2 = plane3.Location(1,1);
x2min3 = plane3.Location(1,1);


y2sum = plane3.Location(1,2);
y2max1 = plane3.Location(1,2);
y2max2 = plane3.Location(1,2);
y2max3 = plane3.Location(1,2);
y2min1 = plane3.Location(1,2);
y2min2 = plane3.Location(1,2);
y2min3 = plane3.Location(1,2);

z2sum = plane3.Location(1,3);
z2max1 = plane3.Location(1,3);
z2max2 = plane3.Location(1,3);
z2max3 = plane3.Location(1,3);
z2min1 = plane3.Location(1,3);
z2min2 = plane3.Location(1,3);
z2min3 = plane3.Location(1,3);

for i = 1:n2
    x2s = plane3.Location(i,1);
    x2sum = x2sum + x2s;
    if x2s > x2max1
        x2max1 = x2s
        y2max1 = plane3.Location(i,2);
        z2max1 = plane3.Location(i,3);
    elseif x2s < x2min1
        x2min1 = x2s
        y2min1 = plane3.Location(i,2);
        z2min1 = plane3.Location(i,3);
    end
    
    y2s = plane3.Location(i,2);
    y2sum = y2sum + y2s;
    if y2s > y2max2
        y2max2 = y2s
        x2max2 = plane3.Location(i,1);
        z2max2 = plane3.Location(i,3);
    elseif y2s < y2min2
        y2min2 = y2s
        x2min2 = plane3.Location(i,1);
        z2min2 = plane3.Location(i,3);
    end
    
    z2s = plane3.Location(i,3);
    z2sum = z2sum + z2s;
    if z2s > z2max3
        z2max3 = z2s
        y2max3 = plane3.Location(i,2);
        x2max3 = plane3.Location(i,1);
    elseif z2s < z2min3
        z2min3 = z2s
        y2min3 = plane3.Location(i,2);
        x2min3 = plane3.Location(i,1);
    end
end

xc2 = (x2sum)/n2;
yc2 = (y2sum)/n2;
zc2 = (z2sum)/n2;
centre2 = [xc2; yc2; zc2]


figure(17);
hold on
pcshow(plane3)
scatter3(xc2, yc2, zc2,'b*')


%scatter3(x2max1, y2max1, z2max1,'g*')
scatter3(x2max2, y2max2, z2max2,'g*')
%scatter3(x2max3, y2max3, z2max3,'g*')


scatter3(x2min1, y2min1, z2min1,'r*')
%scatter3(x2min2, y2min2, z2min2,'r*')
%scatter3(x2min3, y2min3, z2min3,'r*')

hold off

%% Plane equation 
A2 = [xc2, yc2, zc2];
B2 = [x2max2, y2max2, z2max2];
C2 = [x2min1, y2min1, z2min1];

a = B2 - A2;
b = C2 - A2;
nor = cross(a,b);
normal = nor/norm(nor)
A = normal(1);
B = normal(2);
C = normal(3);
D = -A*xc2 - B*yc2 - C*zc2;



hold off

%% REsults
height = h;

sh = normal*height/2;

centre = centre2 + sh';
xc = centre(1);
yc = centre(2);
zc = centre(3);

figure(10);
hold on
pcshow(boxCloud)
scatter3(xc1, yc1, zc1,'r*')
scatter3(xc2, yc2, zc2,'g*')
scatter3(xc, yc, zc,'b*')



hold off









