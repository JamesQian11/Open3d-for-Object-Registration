close all;
clear all;

% theta_h = 1.029;
% theta_v = 0.802;
% n_r = 450;
% n_c = 480;
% 
% filename_fixed = "test.csv";
% filename_moving = "test.csv";
% I_fixed  = csvread(filename_fixed);
% I_moving = csvread(filename_moving);
% I_fixed  = imbilatfilt(I_fixed);%双边滤波函数
% I_moving = imbilatfilt(I_moving);
% I_fixed  = im2double(I_fixed);
% I_moving = im2double(I_moving);
% I_fixed  = I_fixed(:,:,1);
% I_moving = I_moving(:,:,1);
% 
% pc_fixed = []; 
% for m=1:450
%     for n=1:480
%         if(I_fixed(m,n)>50 && I_fixed(m,n)<80)
%             r_i = m;
%             c_i = n;
%             d_i = I_fixed(m,n);
%             alpha_h = (pi-theta_h)/2;
%             gamma_h = alpha_h + (c_i*theta_h/n_c);
%             x = d_i/tan(gamma_h);
%           
%             alpha_v = 2*pi-(theta_v/2);
%             gamma_v = alpha_v + (r_i*theta_v/n_r);
%             y = d_i*tan(gamma_v)*(-1);
%            
%             %if(y>-0.3)
%             pc_fixed = [pc_fixed;x y d_i];
%             
%             %end
%         end
%     end
% end
% %disp(pc_fixed) 
% 
% pc_moving = []; 
% for m=1:450
%     for n=1:480
%         if(I_moving(m,n)>55 && I_moving(m,n)<85)
%             r_i = m;
%             c_i = n;
%             d_i = I_moving(m,n);
%             alpha_h = (pi-theta_h)/2;
%             gamma_h = alpha_h + (c_i*theta_h/n_c);
%             x = d_i/tan(gamma_h);
%             
%             alpha_v = 2*pi-(theta_v/2);
%             gamma_v = alpha_v + (r_i*theta_v/n_r);
%             y = d_i*tan(gamma_v)*(-1);
%             
%             %if (y>-0.3)
%             pc_moving = [pc_moving;x y d_i];
%             %disp(pc_moving(:,3))
%             %end
%         end
%     end
% end
% %ColorPointCloud2PLY(pc_moving(:,1),pc_moving(:,2),pc_moving(:,3),0, "moving")
pc_fixed = pcread('test.ply');
pc_moving = pcread('test1.ply');
ptCloud_fixed  = pointCloud(pc_fixed);
ptCloud_moving = pointCloud(pc_moving);
disp(ptCloud_fixed)
ptCloud_fixed  = pcdenoise(ptCloud_fixed);
ptCloud_moving = pcdenoise(ptCloud_moving);

gridSize = 0.1;
ptCloud_fixed = pcdownsample(ptCloud_fixed, 'gridAverage', gridSize);
ptCloud_moving = pcdownsample(ptCloud_moving, 'gridAverage', gridSize);
%percentage = 0.9;
%ptCloud_fixed = pcdownsample(ptCloud_fixed,'random',percentage);
%ptCloud_moving = pcdownsample(ptCloud_moving,'random',percentage);


tform = pcregistericp(ptCloud_moving, ptCloud_fixed, 'Metric','pointToPlane','Extrapolate', true,'InlierRatio',0.05,'MaxIterations',2000,'Tolerance',[0.1,0.5]);
ptCloudAligned = pctransform(ptCloud_moving,tform);
ptCloudScene = pcmerge(ptCloud_fixed, ptCloudAligned,1);

accumTform = tform;
figure(1);
pcshow(ptCloudScene);
