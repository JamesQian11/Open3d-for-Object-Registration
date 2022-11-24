close all;
clear all;

theta = 1.309;
n_r = 240;
n_c = 180;
for i = 1:25
    filename_fixed = "PhotonIC_1.png";

    I_fixed  = imread(filename_fixed);

    I_fixed  = imbilatfilt(I_fixed);

    I_fixed  = im2double(I_fixed);

    I_fixed  = I_fixed(:,:,1);

    pc_fixed = []; 
    for m=1:240
        for n=1:180
            if (I_fixed(m,n)~=0)
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
                pc_fixed = [pc_fixed;x y d_i];
            end
        end
    end
    %disp(pc_fixed)
    ColorPointCloud2PLY(pc_fixed(:,1),pc_fixed(:,2),pc_fixed(:,3),0, "PhotonIC_"+(i)+".ply") 
end