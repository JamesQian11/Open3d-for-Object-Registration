close all;
clear all;

theta_h = 1.029;
theta_v = 0.802;
n_r = 500;
n_c = 480;
for i = 0:30
    filename = ("sc_model/"+(i)+".csv");
    I_fixed  = csvread(filename);
    I_fixed  = I_fixed(:,:,1);
    pc_fixed = []; 
    for m=1:500
        for n=1:480
            if(I_fixed(m,n)>450 && I_fixed(m,n)<650)
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
    %disp(pc_fixed)
    ColorPointCloud2PLY(pc_fixed(:,1),pc_fixed(:,2),pc_fixed(:,3),0, "sc_model/raw_"+(i)+".ply") 
end
