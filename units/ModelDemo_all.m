%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filename : ModelGeneration.m
% Author   : Gurjeet Singh
% Date     : 30th September 2020
% Description : Code to generate 3D model from depth images
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;

theta_h = 1.029;
theta_v = 0.802;
n_r = 450;
n_c = 480;

filename_fixed = "raw_26.csv";
filename_moving = "raw_27.csv";
I_fixed  = csvread(filename_fixed);
I_moving = csvread(filename_moving);
I_fixed  = imbilatfilt(I_fixed);%双边滤波函数
I_moving = imbilatfilt(I_moving);
I_fixed  = im2double(I_fixed);
I_moving = im2double(I_moving);
I_fixed  = I_fixed(:,:,1);
I_moving = I_moving(:,:,1);

pc_fixed = []; 
for m=1:450
    for n=1:480
        if(I_fixed(m,n)>10 && I_fixed(m,n)<80)
            r_i = m;
            c_i = n;
            d_i = I_fixed(m,n);
            alpha_h = (pi-theta_h)/2;
            gamma_h = alpha_h + (c_i*theta_h/n_c);
            x = d_i/tan(gamma_h);
          
            alpha_v = 2*pi-(theta_v/2);
            gamma_v = alpha_v + (r_i*theta_v/n_r);
            y = d_i*tan(gamma_v)*(-1);
           
            %if(y>-0.3)
            pc_fixed = [pc_fixed;x y d_i];
            
            %end
        end
    end
end
    
pc_moving = []; 
for m=1:450
    for n=1:480
        if(I_moving(m,n)>10 && I_moving(m,n)<80)
            r_i = m;
            c_i = n;
            d_i = I_moving(m,n);
            alpha_h = (pi-theta_h)/2;
            gamma_h = alpha_h + (c_i*theta_h/n_c);
            x = d_i/tan(gamma_h);
            
            alpha_v = 2*pi-(theta_v/2);
            gamma_v = alpha_v + (r_i*theta_v/n_r);
            y = d_i*tan(gamma_v)*(-1);
            
            %if (y>-0.3)
            pc_moving = [pc_moving;x y d_i];
            %end
        end
    end
end
  
ptCloud_fixed  = pointCloud(pc_fixed);
ptCloud_moving = pointCloud(pc_moving);

ptCloud_fixed  = pcdenoise(ptCloud_fixed);
ptCloud_moving = pcdenoise(ptCloud_moving);



gridSize = 0.1;
ptCloud_fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
ptCloud_moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);

%percentage = 0.1;
%ptCloud_fixed = pcdownsample(ptCloud_fixed,'random',percentage);
%ptCloud_moving = pcdownsample(ptCloud_moving,'random',percentage);

tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPoint','Extrapolate', true,'MaxIterations',2000);
ptCloudAligned = pctransform(ptCloud_moving,tform);
ptCloudScene = pcmerge(ptCloud_fixed, ptCloudAligned,0.1);
accumTform = tform;
figure(1);
pcshow(ptCloudScene);

for i = 28:100   %continous loop for depth images
    filename_fixed  = "raw_"+(i-1)+".csv";
    filename_moving = "raw_"+(i)+".csv";
    I_fixed  = csvread(filename_fixed);
    I_moving = csvread(filename_moving);
    I_fixed  = imbilatfilt(I_fixed);
    I_moving = imbilatfilt(I_moving);
    I_fixed  = im2double(I_fixed);
    I_moving = im2double(I_moving);
    I_fixed  = I_fixed(:,:,1);
    I_moving = I_moving(:,:,1);
    pc_fixed = []; 
    for m=1:450
        for n=1:480
            if(I_fixed(m,n)>10 && I_fixed(m,n)<80)
                r_i = m;
                c_i = n;
                d_i = I_fixed(m,n);
                alpha_h = (pi-theta_h)/2;
                gamma_h = alpha_h + (c_i*theta_h/n_c);
                x = d_i/tan(gamma_h);
          
                alpha_v = 2*pi-(theta_v/2);
                gamma_v = alpha_v + (r_i*theta_v/n_r);
                y = d_i*tan(gamma_v)*(-1);
           
                pc_fixed = [pc_fixed;x y d_i];
            end
        end
    end
    
    pc_moving = []; 
    for m=1:450
        for n=1:480
            if(I_moving(m,n)>10 && I_moving(m,n)<80)
                r_i = m;
                c_i = n;
                d_i = I_moving(m,n);
                alpha_h = (pi-theta_h)/2;
                gamma_h = alpha_h + (c_i*theta_h/n_c);
                x = d_i/tan(gamma_h);
            
                alpha_v = 2*pi-(theta_v/2);
                gamma_v = alpha_v + (r_i*theta_v/n_r);
                y = d_i*tan(gamma_v)*(-1);

                pc_moving = [pc_moving;x y d_i];
            
             end
        end
    end
    
    ptCloud_fixed  = pointCloud(pc_fixed);
    ptCloud_moving = pointCloud(pc_moving);
    
    ptCloud_fixed  = pcdenoise(ptCloud_fixed);
    ptCloud_moving = pcdenoise(ptCloud_moving);
    
    gridSize = 0.1;
    ptCloud_fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
    ptCloud_moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);

    %percentage = 0.1;
    %ptCloud_fixed = pcdownsample(ptCloud_fixed,'random',percentage);
    %ptCloud_moving1 = pcdownsample(ptCloud_moving,'random',percentage);
    
    tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPoint','Extrapolate', true,'MaxIterations',2000);   
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloud_moving, accumTform);
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned,0.1);
    
end



figure;
%ptCloudScene = pcdenoise(ptCloudScene);
pcshow(ptCloudScene);
xlabel('X');
ylabel('Y');
zlabel('Z');
