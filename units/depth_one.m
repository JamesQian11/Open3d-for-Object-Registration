close all;
clear all;


ptCloud_fixed = pcread('raw_3.ply');
ptCloud_moving = pcread('raw_4.ply');
ptCloud_fixed  = pcdenoise(ptCloud_fixed);
ptCloud_moving = pcdenoise(ptCloud_moving);

gridSize = 0.5;
ptCloud_fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
ptCloud_moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);


tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPlane','Extrapolate', true,'MaxIterations',2000);
ptCloudAligned = pctransform(ptCloud_moving,tform);
ptCloudScene = pcmerge(ptCloud_fixed, ptCloudAligned,0.001);

accumTform = tform;
figure(1);
pcshow(ptCloudScene);
