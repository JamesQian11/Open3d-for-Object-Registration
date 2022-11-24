
close all;
clear all;


ptCloud_fixed = pcread('depth_240.ply');
ptCloud_moving = pcread('depth_241.ply');

ptCloud_fixed  = pcdenoise(ptCloud_fixed);
ptCloud_moving = pcdenoise(ptCloud_moving);



gridSize = 0.5;
ptCloud_fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
ptCloud_moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);


tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPoint','Extrapolate', true,'MaxIterations',2000);
ptCloudAligned = pctransform(ptCloud_moving,tform);
ptCloudScene = pcmerge(ptCloud_fixed, ptCloudAligned,0.001);
accumTform = tform;
figure(1);
pcshow(ptCloudScene);

for i = 242:256   %continous loop for depth images
    ptCloud_fixed = pcread("depth_"+(i-1)+".ply");
    ptCloud_moving = pcread("depth_"+(i-1)+".ply");
    
    
    ptCloud_fixed  = pcdenoise(ptCloud_fixed);
    ptCloud_moving = pcdenoise(ptCloud_moving);
    
    
    
    gridSize = 0.5;
    ptCloud_fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
    ptCloud_moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);

    
    tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPoint','Extrapolate', true,'MaxIterations',2000);
    
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloud_moving, accumTform);
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned,0.001);
   
end



figure;
pcshow(ptCloudScene);
xlabel('X');
ylabel('Y');
zlabel('Z');
