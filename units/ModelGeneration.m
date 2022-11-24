%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filename : ModelGeneration.m
% Author   : Gurjeet Singh
% Date     : 30th September 2020
% Description : Code to generate 3D model from depth images
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;

theta = 1.309;
n_r = 640;
n_c = 480;

filename_fixed = "depth_1.png";
filename_moving = "depth_2.png";
I_fixed  = imread(filename_fixed);
I_moving = imread(filename_moving);
I_fixed  = imbilatfilt(I_fixed);
I_moving = imbilatfilt(I_moving);
I_fixed  = im2double(I_fixed);
I_moving = im2double(I_moving);
I_fixed  = I_fixed(:,:,1);
I_moving = I_moving(:,:,1);

pc_fixed = []; 
for m=1:640
    for n=1:480
        if(I_fixed(m,n)~=0)
            r_i = m;
            c_i = n;
            d_i = I_fixed(m,n);
            alpha_h = (pi-theta)/2;
            gamma_h = alpha_h + (c_i*theta/n_c);
            x = d_i/tan(gamma_h);
            
            alpha_v = 2*pi-(theta/2);
            gamma_v = alpha_v + (r_i*theta/n_r);
            y = d_i*tan(gamma_v);
            %pc_fixed  = [pc_fixed;m n I_fixed(m,n)];
            pc_fixed = [pc_fixed;x y d_i];%%
        end
    end
end
    
pc_moving = []; 
for m=1:640
    for n=1:480
        if(I_moving(m,n)~=0)
            r_i = m;
            c_i = n;
            d_i = I_moving(m,n);
            alpha_h = (pi-theta)/2;
            gamma_h = alpha_h + (c_i*theta/n_c);
            x = d_i/tan(gamma_h);
            
            alpha_v = 2*pi-(theta/2);
            gamma_v = alpha_v + (r_i*theta/n_r);
            y = d_i*tan(gamma_v);
            %pc_moving  = [pc_moving;m n I_moving(m,n)];
            pc_moving = [pc_moving;x y d_i];
        end
    end
end
    
ptCloud_fixed  = pointCloud(pc_fixed);
ptCloud_moving = pointCloud(pc_moving);

ptCloud_fixed  = pcdenoise(ptCloud_fixed);
ptCloud_moving = pcdenoise(ptCloud_moving);



%gridSize = 0.1;
%ptCloud_fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
%ptCloud_moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);

tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPlane','Extrapolate', true,'MaxIterations',2000);
ptCloudAligned = pctransform(ptCloud_moving,tform);
ptCloudScene = pcmerge(ptCloud_fixed, ptCloudAligned,0.0001);

accumTform = tform;
figure(1);
pcshow(ptCloudScene);
for i = 3:25   %continous loop for depth images
    filename_fixed  = "PhotonIC_"+(i-1)+".png";
    filename_moving = "PhotonIC_"+(i)+".png";
    I_fixed  = imread(filename_fixed);
    I_moving = imread(filename_moving);
    I_fixed  = imbilatfilt(I_fixed);
    I_moving = imbilatfilt(I_moving);
    I_fixed  = im2double(I_fixed);
    I_moving = im2double(I_moving);
    I_fixed  = I_fixed(:,:,1);
    I_moving = I_moving(:,:,1);
    pc_fixed = []; 
    for m=1:240
        for n=1:180
            if(I_fixed(m,n)~=0)
                r_i = m;
                c_i = n;
                d_i = I_fixed(m,n);
                alpha_h = (pi-theta)/2;
                gamma_h = alpha_h + (c_i*theta/n_c);
                x = d_i/tan(gamma_h);
            
                alpha_v = 2*pi-(theta/2);
                gamma_v = alpha_v + (r_i*theta/n_r);
                y = d_i*tan(gamma_v);
                %pc_fixed  = [pc_fixed;m n I_fixed(m,n)];
                pc_fixed = [pc_fixed; x y d_i];
            end
        end
    end
    
    pc_moving = []; 
    for m=1:240
        for n=1:180
            if(I_moving(m,n)~=0)
                r_i = m;
                c_i = n;
                d_i = I_moving(m,n);
                alpha_h = (pi-theta)/2;
                gamma_h = alpha_h + (c_i*theta/n_c);
                x = d_i/tan(gamma_h);
            
                alpha_v = 2*pi-(theta/2);
                gamma_v = alpha_v + (r_i*theta/n_r);
                y = d_i*tan(gamma_v);
                %pc_moving  = [pc_moving;m n I_moving(m,n)];
                pc_moving = [pc_moving;x y d_i];
            end
        end
    end
    
    ptCloud_fixed  = pointCloud(pc_fixed);
    ptCloud_moving = pointCloud(pc_moving);
    
    ptCloud_fixed  = pcdenoise(ptCloud_fixed);
    ptCloud_moving = pcdenoise(ptCloud_moving);
    
    
    
    %gridSize = 0.1;
    %fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
    %moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);

    
    tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPlane','Extrapolate', true,'MaxIterations',2000);
    
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloud_moving, accumTform);
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned,0.0001);
    %figure;
    %pcshow(ptCloudScene);
    %xlabel('X')
    %ylabel('Y');
    %zlabel('Z');
end



figure;
%ptCloudScene = pcdenoise(ptCloudScene);

pcshow(ptCloudScene);
xlabel('X');
ylabel('Y');
zlabel('Z');
